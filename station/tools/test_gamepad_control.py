"""手柄输入、双推进混合、pitch/roll 按住运动与接管保护的离线验证。"""

import contextlib
from dataclasses import FrozenInstanceError, replace
import io
from pathlib import Path
import struct
import sys
import threading
import unittest
from unittest import mock

STATION_DIR = Path(__file__).resolve().parent.parent
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import contcp_t
import gamepad_control as gamepad
import ugcp_protocol as ugcp


def sample(**changes):
    """默认构造已连接的中立输入；每个测试只写需要改变的字段。"""
    return replace(gamepad.GamepadSnapshot(index=0, connected=True), **changes)


class FakeBackend:
    def __init__(self):
        self.current = sample()
        self.failure = None

    def poll(self, index):
        if index != 0:
            return gamepad.GamepadSnapshot(index=index)
        if self.failure is not None:
            raise self.failure
        return self.current


class GamepadMappingTests(unittest.TestCase):
    def test_config_and_sample_are_immutable(self):
        with self.assertRaises(FrozenInstanceError):
            gamepad.GamepadConfig().deadzone = 0.5
        with self.assertRaises(FrozenInstanceError):
            sample().forward = 1.0

    def test_deadzone_has_exact_zero_and_preserves_both_full_endpoints(self):
        for value in (-0.1, -0.01, 0.0, 0.01, 0.1):
            with self.subTest(value=value):
                self.assertEqual(gamepad.apply_axis_deadzone(value, 0.1), 0.0)
        self.assertAlmostEqual(gamepad.apply_axis_deadzone(0.55, 0.1), 0.5)
        self.assertAlmostEqual(gamepad.apply_axis_deadzone(-0.55, 0.1), -0.5)
        self.assertEqual(gamepad.apply_axis_deadzone(1.0, 0.1), 1.0)
        self.assertEqual(gamepad.apply_axis_deadzone(-1.0, 0.1), -1.0)

    def test_full_forward_reverse_and_horizontal_only(self):
        for forward, turn, expected in (
                (1.0, 0.0, (100, 100)),
                (-1.0, 0.0, (-100, -100)),
                (0.0, 1.0, (0, 0)),
                (0.0, -1.0, (0, 0))):
            with self.subTest(forward=forward, turn=turn):
                self.assertEqual(gamepad.mix_thrusters(forward, turn), expected)

    def test_turn_reduces_inside_output_and_reverse_keeps_bow_turn_direction(self):
        for forward, turn, expected in (
                (1.0, 1.0, (100, 0)),
                (1.0, -1.0, (0, 100)),
                (-1.0, 1.0, (0, -100)),
                (-1.0, -1.0, (-100, 0)),
                (0.6, 0.5, (60, 30)),
                (-0.6, 0.5, (-30, -60))):
            with self.subTest(forward=forward, turn=turn):
                actual = gamepad.mix_thrusters(forward, turn)
                for observed, wanted in zip(actual, expected):
                    self.assertAlmostEqual(observed, wanted)

    def test_configured_maximum_and_axis_deadzone_reach_target(self):
        config = gamepad.GamepadConfig(max_percent=40)
        target = gamepad.map_gamepad_to_target(sample(forward=1.0), config)
        self.assertEqual((target.left_percent, target.right_percent), (40, 40))
        target = gamepad.map_gamepad_to_target(
            sample(forward=0.09, left_x=1.0), config)
        self.assertEqual((target.left_percent, target.right_percent), (0, 0))

    def test_pitch_buttons_require_hold_and_conflicting_buttons_disable(self):
        config = gamepad.GamepadConfig(pitch_rpm=50)
        for y, a, expected in ((True, False, (1, -50)),
                               (False, True, (1, 50)),
                               (False, False, (0, 0)),
                               (True, True, (0, 0))):
            with self.subTest(y=y, a=a):
                target = gamepad.map_gamepad_to_target(
                    sample(button_y=y, button_a=a), config)
                self.assertEqual((target.pitch_enable, target.pitch_rpm), expected)

    def test_direction_pad_requires_hold_and_conflicting_buttons_stop_roll(self):
        config = gamepad.GamepadConfig(roll_rpm=50)
        for left, right, expected in ((True, False, (1, 50)),
                                     (False, True, (1, -50)),
                                     (False, False, (0, 0)),
                                     (True, True, (0, 0))):
            with self.subTest(left=left, right=right):
                target = gamepad.map_gamepad_to_target(
                    sample(dpad_left=left, dpad_right=right), config)
                self.assertEqual((target.roll_enable, target.roll_rpm), expected)
                self.assertEqual((target.pitch_enable, target.pitch_rpm), (0, 0))
                self.assertEqual((target.left_percent, target.right_percent), (0, 0))

    def test_roll_pitch_and_thruster_commands_are_independent(self):
        config = gamepad.GamepadConfig(roll_rpm=75)
        baseline = gamepad.map_gamepad_to_target(
            sample(forward=0.6, left_x=0.4, button_y=True), config)
        for changes, expected in (({'dpad_left': True}, 75),
                                  ({'dpad_right': True}, -75)):
            with self.subTest(changes=changes):
                target = gamepad.map_gamepad_to_target(
                    sample(forward=0.6, left_x=0.4, button_y=True, **changes), config)
                self.assertEqual(target, replace(baseline, roll_enable=1, roll_rpm=expected))
                self.assertEqual(target.pitch_rpm, -config.pitch_rpm)

    def test_invalid_roll_configuration_and_pad_values_are_rejected(self):
        for value in (True, 0, -1, 1.5, 32768, float('nan')):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    gamepad.GamepadConfig(roll_rpm=value)
        for key in ('dpad_left', 'dpad_right'):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    gamepad.map_gamepad_to_target(sample(**{key: 1}), gamepad.GamepadConfig())

    def test_invalid_input_is_rejected_instead_of_becoming_plausible_zero(self):
        config = gamepad.GamepadConfig()
        for value in (float('nan'), float('inf'), -1.01, 1.01):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    gamepad.apply_axis_deadzone(value, 0.1)
                with self.assertRaises(ValueError):
                    gamepad.mix_thrusters(value, 0.0)
        with self.assertRaises(ValueError):
            gamepad.map_gamepad_to_target(sample(connected=False), config)

    def test_x_suppresses_roll_pitch_and_thrust_in_the_same_sample(self):
        target = gamepad.map_gamepad_to_target(
            sample(forward=1.0, button_y=True, dpad_left=True, button_x=True),
            gamepad.GamepadConfig())
        self.assertEqual(target, gamepad.ControlTarget())


class GamepadSamplingTests(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.service = gamepad.GamepadService(backend=self.backend)
        self.now = 10.0

    def poll(self):
        self.now += 0.02
        with mock.patch.object(gamepad.time, 'monotonic', lambda: self.now):
            self.service.poll_once()
        return self.service.snapshot(0)

    def test_unchanged_input_still_gets_a_new_successful_sample_timestamp(self):
        first = self.poll()
        second = self.poll()
        self.assertEqual(first.forward, second.forward)
        self.assertGreater(second.sample_time, first.sample_time)
        self.assertEqual(tuple(self.service.connected_indices()), (0,))

    def test_short_x_press_is_remembered_after_release(self):
        before = self.poll()
        self.backend.current = sample(button_x=True)
        down = self.poll()
        held = self.poll()
        self.backend.current = sample(button_x=False)
        released = self.poll()
        self.assertEqual(down.exit_seq, before.exit_seq + 1)
        self.assertEqual(held.exit_seq, down.exit_seq)
        self.assertEqual(released.exit_seq, down.exit_seq)
        self.assertFalse(released.button_x)

    def test_disconnect_episode_is_remembered_after_reconnect(self):
        before = self.poll()
        self.backend.current = gamepad.GamepadSnapshot(index=0)
        failed = self.poll()
        failed_again = self.poll()
        self.backend.current = sample()
        recovered = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.loss_seq, before.loss_seq + 1)
        self.assertEqual(failed_again.loss_seq, failed.loss_seq)
        self.assertEqual(failed.sample_time, before.sample_time)
        self.assertTrue(recovered.connected)
        self.assertEqual(recovered.loss_seq, failed.loss_seq)

    def test_backend_error_invalidates_input_and_preserves_last_success_time(self):
        before = self.poll()
        self.backend.failure = OSError('离线模拟读取失败')
        failed = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.sample_time, before.sample_time)
        self.assertGreater(failed.loss_seq, before.loss_seq)
        self.assertTrue(failed.error)

    def test_invalid_axis_sample_is_an_input_loss(self):
        before = self.poll()
        self.backend.current = sample(forward=float('nan'))
        failed = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.sample_time, before.sample_time)
        self.assertEqual(failed.loss_seq, before.loss_seq + 1)

    def test_direction_pad_state_is_sampled_and_invalid_state_loses_input(self):
        self.backend.current = sample(dpad_left=True)
        held = self.poll()
        self.assertTrue(held.dpad_left)
        self.assertFalse(held.dpad_right)
        self.backend.current = sample()
        released = self.poll()
        self.assertFalse(released.dpad_left or released.dpad_right)
        self.backend.current = sample(dpad_right='按下')
        failed = self.poll()
        self.assertFalse(failed.connected)
        self.assertEqual(failed.sample_time, released.sample_time)
        self.assertEqual(failed.loss_seq, released.loss_seq + 1)
        self.assertFalse(failed.dpad_left or failed.dpad_right)

    def test_stop_after_worker_start_failure_does_not_join_unstarted_thread(self):
        worker = mock.Mock(ident=None)
        worker.is_alive.return_value = False
        worker.start.side_effect = RuntimeError('离线模拟线程创建失败')
        worker.join.side_effect = RuntimeError('禁止join尚未启动的线程')
        with mock.patch.object(gamepad.threading, 'Thread', return_value=worker):
            with self.assertRaisesRegex(RuntimeError, '创建失败'):
                self.service.start()
            self.service.stop()
        worker.join.assert_not_called()
        self.assertFalse(self.service.is_alive())


class XInputDecodingTests(unittest.TestCase):
    def test_raw_signed_endpoints_and_button_masks_decode_without_loading_dll(self):
        backend = object.__new__(gamepad.XInputBackend)
        raw = {'axis': 0, 'buttons': 0}

        def get_state(index, pointer):
            self.assertEqual(index, 2)
            state = pointer._obj
            state.dwPacketNumber = 7
            state.Gamepad.sThumbLX = raw['axis']
            state.Gamepad.sThumbLY = raw['axis']
            state.Gamepad.wButtons = raw['buttons']
            return 0

        backend._get_state = get_state
        for axis, expected_axis in ((-32768, -1.0), (0, 0.0), (32767, 1.0)):
            for mask, expected_buttons in (
                    (0, (False, False, False, False, False)),
                    (0x4000, (True, False, False, False, False)),
                    (0x8000, (False, True, False, False, False)),
                    (0x1000, (False, False, True, False, False)),
                    (0x0001, (False, False, False, False, False)),
                    (0x0002, (False, False, False, False, False)),
                    (0x0004, (False, False, False, True, False)),
                    (0x0008, (False, False, False, False, True)),
                    (0x000C, (False, False, False, True, True)),
                    (0xD00C, (True, True, True, True, True))):
                with self.subTest(axis=axis, mask=mask):
                    raw.update(axis=axis, buttons=mask)
                    result = backend.poll(2)
                    self.assertTrue(result.connected)
                    self.assertEqual(result.index, 2)
                    self.assertEqual((result.left_x, result.forward),
                                     (expected_axis, expected_axis))
                    self.assertEqual((result.button_x, result.button_y, result.button_a,
                                      result.dpad_left, result.dpad_right),
                                     expected_buttons)

    def test_disconnected_and_other_xinput_error_codes_both_invalidate_input(self):
        backend = object.__new__(gamepad.XInputBackend)
        for error in (1167, 5):
            with self.subTest(error=error):
                backend._get_state = lambda index, pointer: error
                result = backend.poll(0)
                self.assertFalse(result.connected)
                self.assertTrue(result.error)
                if error != 1167:
                    self.assertIn(str(error), result.error)
                with self.assertRaises(ValueError):
                    gamepad.map_gamepad_to_target(result, gamepad.GamepadConfig())


class GamepadPacketTests(unittest.TestCase):
    def test_packet_keeps_independent_channels_and_disables_unused_actuators(self):
        target = gamepad.ControlTarget(25, -60, 1, -50)
        packet = contcp_t._build_auto_sti_packet(target, gamepad.GamepadConfig(pitch_rpm=50))
        self.assertEqual((target.roll_enable, target.roll_rpm), (0, 0))
        payload = ugcp.pack_control21_payload(**packet)
        self.assertEqual((packet['thrustmotor1se_speed_rpm'],
                          packet['thrustmotor2se_speed_rpm']), (25, 60))
        self.assertEqual((packet['thrustmotor_direction'],
                          packet['thrustmotor_direction2']), (0, 0))
        self.assertEqual((packet['pitchmotor_enable'],
                          packet['pitchmotor_speed_rpm']), (1, -50))
        for field in ('rollmotor_enable', 'rollmotor_speed_rpm', 'pump_on_off',
                      'pump_direction', 'backup_for_thrust', 'ask_for_status',
                      'rudder_angle_cdeg'):
            self.assertEqual(packet[field], 0, field)
        self.assertEqual(packet['eth_control_flag'], 1)
        self.assertEqual(len(payload), 21)

    def test_direction_pad_encodes_signed_roll_rpm_in_existing_frame(self):
        config = gamepad.GamepadConfig(roll_rpm=50)
        for left, right, expected in ((True, False, 50), (False, True, -50),
                                     (False, False, 0), (True, True, 0)):
            with self.subTest(left=left, right=right):
                target = gamepad.map_gamepad_to_target(
                    sample(dpad_left=left, dpad_right=right), config)
                packet = contcp_t._build_auto_sti_packet(target, config)
                frame = ugcp.build_frame(ugcp.MSG_CONTROL21_STREAM,
                                        ugcp.pack_control21_payload(**packet), seq=12)
                kind, sequence, payload = ugcp.parse_frame(frame)
                self.assertEqual((kind, sequence, len(payload)), (ugcp.MSG_CONTROL21_STREAM, 12, 21))
                self.assertEqual(payload[5], int(expected != 0))
                self.assertEqual(struct.unpack_from('<i', payload, 6)[0], expected)
                self.assertEqual(payload[:5], bytes(5))
                self.assertEqual(payload[12:14], bytes(2))

    def test_roll_packet_rejects_inconsistent_enable_or_unconfigured_speed(self):
        config = gamepad.GamepadConfig()
        for enabled, rpm in ((0, 50), (0, -50), (1, 0), (1, 49), (1, -49), (2, 50)):
            with self.subTest(enabled=enabled, rpm=rpm):
                target = gamepad.ControlTarget(roll_enable=enabled, roll_rpm=rpm)
                with self.assertRaises(ValueError):
                    contcp_t._build_auto_sti_packet(target, config)

    def test_motor_swap_and_forward_polarity_are_explicit_configuration(self):
        config = gamepad.GamepadConfig(
            motor1_is_left=False, motor1_forward_dir=1, motor2_forward_dir=0)
        packet = contcp_t._build_auto_sti_packet(
            gamepad.ControlTarget(25, -60), config)
        self.assertEqual((packet['thrustmotor1se_speed_rpm'],
                          packet['thrustmotor2se_speed_rpm']), (60, 25))
        self.assertEqual((packet['thrustmotor_direction'],
                          packet['thrustmotor_direction2']), (0, 0))


class AutoStiHarness:
    """只驱动真实模式循环；输入、时钟、链路和板端状态均由本地夹具提供。"""

    def __init__(self, timeline=None, ready_at=0.10, home_state=0,
                 telemetry_until=None, fault_at=None, fault='x',
                 fail_motion=False, fail_stream=False, indices=None):
        self.timeline = timeline or []
        self.ready_at = ready_at
        self.home_state = home_state
        self.telemetry_until = telemetry_until
        self.fault_at = fault_at
        self.fault = fault
        self.fail_motion = fail_motion
        self.fail_stream = fail_stream
        self.indices = [0] if indices is None else indices
        self.now = 10.0
        self.origin = self.now
        self.last_telemetry = -1.0
        self.sent = []
        self.attempts = []
        self.link_closed = threading.Event()
        self.ctl_store = contcp_t.UgctlStore()
        self.wrm_store = contcp_t.UgwrmStore()
        self.old_mask = ugcp.STREAM_EKF
        self.stream_mask = self.old_mask
        self.controller_index = None
        self.read_indices = []
        self.snapshot_delay = 0.0
        self.pause_at = None
        self.pause_done = False
        self.max_runtime = 5.0

    @property
    def elapsed(self):
        return self.now - self.origin

    def fault_active(self):
        return self.fault_at is not None and self.elapsed + 1e-9 >= self.fault_at

    def connected_indices(self):
        return list(self.indices)

    def is_alive(self):
        return not (self.fault == 'worker' and self.fault_active())

    def snapshot(self, index):
        self.read_indices.append(index)
        self.now += self.snapshot_delay
        if index not in self.indices:
            raise AssertionError('模式读取了未指定的控制器')
        values = {'index': index, 'connected': True, 'sample_time': self.now}
        for at, changes in self.timeline:
            if self.elapsed + 1e-9 >= at:
                values.update(changes)
        if self.fault_active():
            if self.fault == 'x':
                values.update(button_x=True, exit_seq=1)
            elif self.fault == 'missed_x':
                values.update(button_x=False, exit_seq=1)
            elif self.fault == 'reconnect':
                values.update(connected=True, loss_seq=1)
            elif self.fault == 'disconnect':
                values.update(connected=False, loss_seq=1)
            elif self.fault == 'stale':
                values['sample_time'] = self.now - 0.25
        return gamepad.GamepadSnapshot(**values)

    @staticmethod
    def is_motion(payload):
        # 此处表示非中立指令；非零舵角可能正在保持，并非机械运动反馈。
        return (payload[0] != 0 or payload[5] != 0 or any(payload[12:14]) or
                struct.unpack_from('<h', payload, 19)[0] != 0)

    def sendall(self, frame):
        parsed = ugcp.parse_frame(frame)
        self.attempts.append((self.elapsed, parsed))
        kind, _, payload = parsed
        if kind == ugcp.MSG_CONTROL21_STREAM:
            if self.fail_stream or (self.fail_motion and self.is_motion(payload)):
                raise OSError('离线模拟控制包发送失败')
        if kind == ugcp.MSG_STREAM_CONTROL:
            self.stream_mask = payload[0]
        self.sent.append((self.elapsed, parsed))

    def sleep(self, duration):
        self.now += duration
        if self.pause_at is not None and not self.pause_done and self.elapsed >= self.pause_at:
            self.pause_done = True
            self.now += 0.30
        if self.elapsed > self.max_runtime:
            raise AssertionError('auto_sti 未在虚拟时限内退出')
        if self.fault_active() and self.fault == 'link':
            self.link_closed.set()
        self.publish_telemetry()

    def publish_telemetry(self):
        if self.ready_at is None or self.elapsed + 1e-9 < self.ready_at:
            return
        if self.telemetry_until is not None and self.elapsed > self.telemetry_until:
            return
        if self.elapsed - self.last_telemetry < 0.099:
            return
        self.last_telemetry = self.elapsed
        ctl = {'mode': 0, 'armed': 0}
        wrm = {'home_state': self.home_state, 'home_fail': 0}
        if self.fault_active() and self.fault == 'pid':
            ctl.update(mode=1, armed=1)
        if self.fault_active() and self.fault == 'homing':
            wrm['home_state'] = 1
        if self.stream_mask & ugcp.STREAM_CTL:
            self.ctl_store.publish(ctl)
        if self.stream_mask & ugcp.STREAM_WRM:
            self.wrm_store.publish(wrm)

    def run(self):
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t.time, 'monotonic', lambda: self.now))
            stack.enter_context(mock.patch.object(contcp_t.time, 'sleep', self.sleep))
            stack.enter_context(mock.patch.object(contcp_t, '_auto_sti_escape_pressed',
                lambda: self.fault == 'escape' and self.fault_active()))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(mock.patch.object(contcp_t, 'TELEMETRY_STREAM_MASK', self.old_mask))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            # 旧的合格缓存不能直接满足本次准备阶段。
            self.ctl_store.publish({'mode': 0, 'armed': 0})
            self.wrm_store.publish({'home_state': 0, 'home_fail': 0})
            contcp_t.run_auto_sti_mode(
                self, self, self.ctl_store, self.wrm_store, self.link_closed,
                config=gamepad.GamepadConfig(), controller_index=self.controller_index)

    def controls(self):
        return [(at, parsed[2]) for at, parsed in self.sent
                if parsed[0] == ugcp.MSG_CONTROL21_STREAM]

    def motions(self):
        return [(at, payload) for at, payload in self.controls() if self.is_motion(payload)]


class AutoStiLoopTests(unittest.TestCase):
    def assert_cleanup(self, harness):
        releases = [parsed[2] for _, parsed in harness.sent
                    if parsed[0] == ugcp.MSG_CONTROL19_LATCHED]
        self.assertEqual(releases[-2:], [bytes(18) + b'\x01', bytes(19)])
        masks = [parsed[2][0] for _, parsed in harness.sent
                 if parsed[0] == ugcp.MSG_STREAM_CONTROL]
        self.assertEqual(masks[0], harness.old_mask | ugcp.STREAM_CTL | ugcp.STREAM_WRM | ugcp.STREAM_BAT)
        self.assertEqual(masks[-1], harness.old_mask | ugcp.STREAM_BAT)

    def test_initial_non_neutral_waits_without_sending_then_requires_new_board_state(self):
        harness = AutoStiHarness(
            [(0.0, {'forward': 1.0}), (0.20, {'forward': 0.0}),
             (0.32, {'forward': 1.0}), (0.52, {'forward': 0.0}),
             (0.60, {'forward': 1.0})], ready_at=0.50, fault_at=0.80)
        harness.run()
        self.assertTrue(harness.motions())
        self.assertTrue(all(at >= 0.199 for at, _ in harness.controls()))
        self.assertTrue(all(not harness.is_motion(payload)
                            for at, payload in harness.controls() if at < 0.499))
        self.assertGreaterEqual(harness.motions()[0][0], 0.499)
        self.assert_cleanup(harness)

    def test_active_streams_at_ten_hz_when_target_is_unchanged(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})], fault_at=0.82)
        harness.run()
        times = [at for at, _ in harness.motions()]
        self.assertGreaterEqual(len(times), 5)
        for earlier, later in zip(times, times[1:]):
            self.assertGreaterEqual(later - earlier, 0.079)
            self.assertLessEqual(later - earlier, 0.121)
        self.assert_cleanup(harness)

    def test_initial_direction_pad_hold_requires_release_before_takeover(self):
        for initial in ({'dpad_left': True}, {'dpad_right': True},
                        {'dpad_left': True, 'dpad_right': True}):
            with self.subTest(initial=initial):
                harness = AutoStiHarness(
                    [(0.0, initial), (0.20, {'dpad_left': False, 'dpad_right': False}),
                     (0.32, {'dpad_right': True})], fault_at=0.54)
                harness.run()
                self.assertTrue(harness.motions())
                self.assertTrue(all(at >= 0.199 for at, _ in harness.controls()))
                self.assertTrue(all(struct.unpack_from('<i', payload, 6)[0] == -gamepad.GamepadConfig().roll_rpm
                                    for _, payload in harness.motions()))
                self.assert_cleanup(harness)

    def test_roll_release_or_conflict_stops_without_waiting_for_heartbeat(self):
        for changes in ({'dpad_left': False}, {'dpad_right': True}):
            with self.subTest(changes=changes):
                harness = AutoStiHarness(
                    [(0.20, {'forward': 1.0, 'button_y': True, 'dpad_left': True}),
                     (0.26, changes)], fault_at=0.44)
                harness.run()
                self.assertTrue(any(payload[5] for _, payload in harness.controls()))
                stopped = [(at, payload) for at, payload in harness.controls()
                           if at >= 0.259 and payload[5] == 0]
                self.assertTrue(stopped)
                self.assertLessEqual(stopped[0][0], 0.281)
                self.assertEqual(struct.unpack_from('<i', stopped[0][1], 6)[0], 0)
                self.assertEqual(stopped[0][1][0], 1)
                self.assertEqual(stopped[0][1][12:14], bytes((100, 100)))
                self.assert_cleanup(harness)

    def test_center_and_pitch_release_send_without_waiting_for_next_heartbeat(self):
        harness = AutoStiHarness(
            [(0.20, {'forward': 1.0, 'button_y': True}),
             (0.26, {'forward': 0.0, 'button_y': False})], fault_at=0.44)
        harness.run()
        zeros = [(at, payload) for at, payload in harness.controls()
                 if at >= 0.259 and not harness.is_motion(payload)]
        self.assertTrue(zeros)
        self.assertLessEqual(zeros[0][0], 0.281)
        self.assertEqual(struct.unpack_from('<i', zeros[0][1], 1)[0], 0)
        self.assert_cleanup(harness)

    def test_input_worker_and_link_failures_exit_without_resuming_motion(self):
        for fault in ('x', 'missed_x', 'reconnect', 'disconnect', 'stale',
                      'worker', 'link', 'escape'):
            with self.subTest(fault=fault):
                harness = AutoStiHarness([(0.20, {'forward': 1.0, 'dpad_left': True})],
                                        fault_at=0.36, fault=fault)
                harness.run()
                self.assertTrue(harness.motions())
                self.assertTrue(any(payload[5] for _, payload in harness.motions()))
                self.assertTrue(all(at < 0.361 for at, _ in harness.motions()))
                self.assertLess(harness.elapsed, 0.50)
                self.assert_cleanup(harness)

    def test_prepare_timeout_rejects_old_cached_off_and_home_idle(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})], ready_at=None)
        harness.run()
        self.assertFalse(harness.motions())
        self.assertGreaterEqual(harness.elapsed, 3.0)
        self.assertLess(harness.elapsed, 3.2)
        self.assert_cleanup(harness)

    def test_homing_during_prepare_never_sends_motion(self):
        for state in (1, 2, 99):
            with self.subTest(home_state=state):
                harness = AutoStiHarness([(0.20, {'forward': 1.0})], home_state=state)
                harness.run()
                self.assertFalse(harness.motions())
                self.assert_cleanup(harness)

    def test_active_pid_or_homing_takeover_causes_exit(self):
        for fault in ('pid', 'homing'):
            with self.subTest(fault=fault):
                harness = AutoStiHarness([(0.20, {'forward': 1.0, 'dpad_right': True})],
                                        fault_at=0.40, fault=fault)
                harness.run()
                self.assertTrue(harness.motions())
                self.assertLess(harness.elapsed, 0.65)
                self.assert_cleanup(harness)

    def test_active_telemetry_timeout_stops_further_motion(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0, 'dpad_left': True})],
                                telemetry_until=0.30)
        harness.run()
        self.assertTrue(harness.motions())
        self.assertLess(harness.elapsed, 1.50)
        self.assert_cleanup(harness)

    def test_send_failure_attempts_cleanup_and_never_retries_motion(self):
        for fail_motion, fail_stream in ((True, False), (False, True)):
            with self.subTest(fail_motion=fail_motion):
                harness = AutoStiHarness([(0.20, {'dpad_left': True})],
                                        fail_motion=fail_motion, fail_stream=fail_stream)
                harness.run()
                self.assertFalse(harness.motions())
                attempts = [(at, parsed[2]) for at, parsed in harness.attempts
                            if parsed[0] == ugcp.MSG_CONTROL21_STREAM and
                            harness.is_motion(parsed[2])]
                self.assertEqual(len(attempts), int(fail_motion))
                if fail_motion:
                    self.assertEqual(attempts[0][1][5], 1)
                    self.assertEqual(struct.unpack_from('<i', attempts[0][1], 6)[0], gamepad.GamepadConfig().roll_rpm)
                self.assert_cleanup(harness)

    def test_multiple_controllers_require_explicit_selection(self):
        harness = AutoStiHarness(indices=[0, 1])
        harness.run()
        self.assertEqual(harness.sent, [])

    def test_explicit_controller_binding_is_not_changed_during_mode(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})],
                                indices=[0, 1], fault_at=0.40)
        harness.controller_index = 1
        harness.run()
        self.assertTrue(harness.motions())
        self.assertEqual(set(harness.read_indices), {1})
        self.assert_cleanup(harness)

    def test_long_control_loop_pause_exits_even_if_input_reader_is_fresh(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})])
        harness.pause_at = 0.30
        harness.run()
        self.assertTrue(harness.motions())
        self.assertTrue(all(at < 0.30 for at, _ in harness.motions()))
        self.assert_cleanup(harness)

    def test_input_published_after_old_loop_time_does_not_look_future_dated(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})], fault_at=0.50)
        harness.snapshot_delay = 0.001
        harness.run()
        self.assertTrue(harness.motions())
        self.assertGreaterEqual(harness.elapsed, 0.50)
        self.assert_cleanup(harness)

    def test_telemetry_published_during_snapshot_is_compared_with_new_clock(self):
        harness = AutoStiHarness([(0.20, {'forward': 1.0})], fault_at=0.50)
        original = harness.wrm_store.snapshot_with_time

        def racing_snapshot():
            harness.now += 0.001
            if harness.elapsed >= 0.10:
                harness.wrm_store.publish({'home_state': 0, 'home_fail': 0})
            return original()

        with mock.patch.object(harness.wrm_store, 'snapshot_with_time', racing_snapshot):
            harness.run()
        self.assertTrue(harness.motions())
        self.assertGreaterEqual(harness.elapsed, 0.50)
        self.assert_cleanup(harness)

    def test_x_before_center_exits_without_taking_control(self):
        harness = AutoStiHarness([(0.0, {'forward': 1.0})], fault_at=0.10)
        harness.run()
        self.assertEqual(harness.sent, [])


class AutoStiRudderTests(unittest.TestCase):
    assert_cleanup = AutoStiLoopTests.assert_cleanup

    @staticmethod
    def angles(harness):
        return [(at, struct.unpack_from('<h', payload, 19)[0])
                for at, payload in harness.controls()]

    def test_packet_encodes_absolute_angle_and_rejects_invalid_targets(self):
        config = gamepad.GamepadConfig()
        for angle in (-4500, -1234, 0, 4500):
            packet = contcp_t._build_auto_sti_packet(gamepad.ControlTarget(), config, angle)
            self.assertEqual(struct.unpack_from('<h', ugcp.pack_control21_payload(**packet), 19)[0], angle)
        for angle in (-4501, 4501, 1.5, True, float('nan')):
            with self.assertRaises(ValueError):
                contcp_t._build_auto_sti_packet(gamepad.ControlTarget(), config, angle)

    def test_entry_requires_each_trigger_and_b_released(self):
        for held in ({'right_trigger': 1.0}, {'left_trigger': 1.0},
                     {'right_trigger': 1.0, 'left_trigger': 1.0}, {'button_b': True}):
            with self.subTest(held=held):
                harness = AutoStiHarness(
                    [(0.0, held), (0.20, {'right_trigger': 0.0, 'left_trigger': 0.0, 'button_b': False}),
                     (0.40, {'right_trigger': 1.0})], fault_at=0.70)
                harness.run()
                self.assertTrue(all(at >= 0.199 for at, _ in harness.controls()))
                self.assertTrue(any(angle > 0 for _, angle in self.angles(harness)))
                self.assert_cleanup(harness)

    def test_preparing_does_not_integrate_or_accept_held_triggers(self):
        harness = AutoStiHarness(
            [(0.20, {'right_trigger': 1.0}), (0.60, {'right_trigger': 0.0}),
             (0.70, {'right_trigger': 1.0})], ready_at=0.40, fault_at=0.90)
        harness.run()
        self.assertTrue(all(angle == 0 for at, angle in self.angles(harness) if at < 0.699))
        moving = [(at, angle) for at, angle in self.angles(harness) if angle > 0]
        self.assertTrue(moving)
        self.assertLessEqual(moving[0][1], 360)
        self.assert_cleanup(harness)

    def test_full_and_partial_triggers_follow_rate_at_ten_hz(self):
        for changes, rate in (({'right_trigger': 1.0}, 30.0),
                              ({'left_trigger': 1.0}, -30.0),
                              ({'right_trigger': 0.55}, 15.0)):
            with self.subTest(changes=changes):
                harness = AutoStiHarness([(0.20, changes)], fault_at=0.90)
                harness.run()
                moving = [(at, angle) for at, angle in self.angles(harness) if angle]
                self.assertGreaterEqual(len(moving), 5)
                for (at1, a1), (at2, a2) in zip(moving, moving[1:]):
                    self.assertAlmostEqual((a2 - a1) / 100, rate * (at2 - at1), delta=0.011)
                    self.assertGreaterEqual(at2 - at1, 0.079)
                    self.assertLessEqual(at2 - at1, 0.121)

    def test_release_or_dual_trigger_immediately_holds_nonzero_target(self):
        for changes in ({'right_trigger': 0.0}, {'left_trigger': 1.0}):
            with self.subTest(changes=changes):
                harness = AutoStiHarness(
                    [(0.20, {'right_trigger': 1.0}), (0.26, changes)], fault_at=0.60)
                harness.run()
                held = [(at, angle) for at, angle in self.angles(harness) if at >= 0.259]
                self.assertTrue(held)
                self.assertLessEqual(held[0][0], 0.281)
                self.assertGreater(held[0][1], 0)
                self.assertEqual(len({angle for _, angle in held}), 1)

    def test_b_centers_without_affecting_other_axes_and_requires_full_release(self):
        harness = AutoStiHarness([
            (0.20, {'right_trigger': 1.0, 'forward': 1.0, 'button_y': True, 'dpad_left': True}),
            (0.36, {'button_b': True, 'rudder_center_seq': 1}),
            (0.50, {'button_b': False}),
            (0.60, {'right_trigger': 0.0, 'left_trigger': 1.0}),
            (0.70, {'left_trigger': 0.0}), (0.76, {'right_trigger': 1.0})], fault_at=1.0)
        harness.run()
        after_b = [(at, angle) for at, angle in self.angles(harness) if at >= 0.359]
        self.assertLessEqual(after_b[0][0], 0.381)
        self.assertTrue(all(angle == 0 for at, angle in after_b if at < 0.759))
        self.assertTrue(any(angle > 0 for at, angle in after_b if at >= 0.759))
        for at, payload in harness.controls():
            if 0.359 <= at < 0.759:
                self.assertEqual((payload[0], payload[5]), (1, 1))
                self.assertEqual(payload[12:14], bytes((100, 100)))
        # B持续按住不将发送频率升到主循环的50Hz。
        times = [at for at, _ in after_b if at < 0.70]
        self.assertTrue(all(b - a >= 0.079 for a, b in zip(times, times[1:])))
        self.assert_cleanup(harness)

    def test_b_sequence_catches_short_press_between_control_iterations(self):
        harness = AutoStiHarness([
            (0.20, {'right_trigger': 1.0}),
            (0.36, {'button_b': False, 'rudder_center_seq': 1}),
            (0.50, {'right_trigger': 0.0}), (0.60, {'left_trigger': 1.0})], fault_at=0.90)
        harness.run()
        angles = self.angles(harness)
        self.assertTrue(any(angle > 0 for at, angle in angles if at < 0.359))
        self.assertTrue(all(angle == 0 for at, angle in angles if 0.359 <= at < 0.599))
        self.assertTrue(any(angle < 0 for at, angle in angles if at >= 0.599))

    def test_endpoint_clamps_integral_and_reverses_without_windup(self):
        harness = AutoStiHarness(
            [(0.20, {'right_trigger': 1.0}),
             (2.30, {'right_trigger': 0.0, 'left_trigger': 1.0})], fault_at=4.0)
        harness.run()
        angles = self.angles(harness)
        self.assertTrue(all(abs(angle) <= 4500 for _, angle in angles))
        self.assertTrue(any(angle == 4500 for _, angle in angles))
        reverse = [(at, angle) for at, angle in angles if at >= 2.299]
        self.assertLess(reverse[0][1], 4500)
        self.assertTrue(any(angle < 0 for _, angle in reverse))

    def test_tiny_input_accumulates_fractional_centidegrees(self):
        harness = AutoStiHarness([(0.20, {'right_trigger': 0.101})], fault_at=1.20)
        harness.run()
        angles = self.angles(harness)
        self.assertGreaterEqual(angles[-1][1], 2)
        self.assertLessEqual(angles[-1][1], 4)

    def test_rudder_faults_exit_and_send_existing_neutral_release(self):
        for fault in ('x', 'missed_x', 'reconnect', 'disconnect', 'stale',
                      'worker', 'link', 'escape', 'pid', 'homing'):
            with self.subTest(fault=fault):
                harness = AutoStiHarness([(0.20, {'right_trigger': 1.0})],
                                        fault_at=0.40, fault=fault)
                harness.run()
                self.assertTrue(any(angle > 0 for _, angle in self.angles(harness)))
                self.assertTrue(all(at < 0.401 for at, angle in self.angles(harness) if angle))
                self.assert_cleanup(harness)

    def test_rudder_send_failure_does_not_retry_nonzero_angle(self):
        harness = AutoStiHarness([(0.20, {'right_trigger': 1.0})], fail_motion=True)
        harness.run()
        attempts = [parsed[2] for _, parsed in harness.attempts
                    if parsed[0] == ugcp.MSG_CONTROL21_STREAM and harness.is_motion(parsed[2])]
        self.assertEqual(len(attempts), 1)
        self.assertTrue(all(angle == 0 for _, angle in self.angles(harness)))
        self.assert_cleanup(harness)

    def test_new_session_and_long_pause_never_restore_previous_angle(self):
        harness = AutoStiHarness([(0.20, {'right_trigger': 1.0})])
        harness.pause_at = 0.30
        harness.run()
        self.assertTrue(any(angle > 0 for _, angle in self.angles(harness)))
        self.assertTrue(all(at < 0.30 for at, _ in harness.controls()))
        self.assert_cleanup(harness)
        second = AutoStiHarness(fault_at=0.40)
        second.run()
        self.assertTrue(all(angle == 0 for _, angle in self.angles(second)))


class TelemetryStoreCompatibilityTests(unittest.TestCase):
    def test_ctl_snapshot_legacy_pair_and_new_timestamp_are_both_available(self):
        store = contcp_t.UgctlStore()
        with mock.patch.object(contcp_t.time, 'monotonic', lambda: 12.5):
            store.publish({'mode': 0, 'armed': 0})
        generation, row = store.snapshot()
        timed_generation, timed_row, received_at = store.snapshot_with_time()
        self.assertEqual(generation, timed_generation)
        self.assertEqual(row, timed_row)
        self.assertEqual(received_at, 12.5)
        row['mode'] = 2
        self.assertEqual(store.snapshot()[1]['mode'], 0)

    def test_unique_receiver_publishes_fragmented_worm_telemetry_and_marks_eof(self):
        row = {name: 0 for name in contcp_t.UGWRM_FIELDS_V2}
        row.update(schema=2, seq=7, home_state=3, pulse=123)
        body = 'UGWRM,' + ','.join(str(row[name]) for name in contcp_t.UGWRM_FIELDS_V2)
        checksum = 0
        for character in body:
            checksum ^= ord(character)
        line = ('$%s*%02X\r\n' % (body, checksum)).encode('ascii')
        wire = b'$UGWRM,bad*00\r\n' + line
        conn = mock.Mock()
        conn.recv.side_effect = [wire[:9], wire[9:31], wire[31:], b'']
        ctl_store, wrm_store = contcp_t.UgctlStore(), contcp_t.UgwrmStore()
        closed = threading.Event()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(mock.patch.object(contcp_t.os, 'makedirs'))
            stack.enter_context(mock.patch.object(contcp_t, 'open', mock.mock_open(), create=True))
            stack.enter_context(mock.patch.object(contcp_t.time, 'monotonic', lambda: 12.5))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.receive_thread(conn, ctl_store, wrm_store, closed)
        generation, received, received_at = wrm_store.snapshot_with_time()
        self.assertEqual(generation, 1)
        self.assertEqual(received['seq'], 7)
        self.assertEqual(received['home_state'], 3)
        self.assertEqual(received_at, 12.5)
        self.assertEqual(ctl_store.snapshot(), (0, None))
        self.assertTrue(closed.is_set())
        conn.close.assert_called()

    def test_receiver_initialization_failure_also_signals_disconnection(self):
        conn, closed = mock.Mock(), threading.Event()
        with mock.patch.object(contcp_t.os, 'makedirs', side_effect=OSError('日志目录不可写')):
            with contextlib.redirect_stdout(io.StringIO()):
                contcp_t.receive_thread(conn, link_closed=closed)
        self.assertTrue(closed.is_set())
        conn.close.assert_called_once()


class MainResourceCleanupTests(unittest.TestCase):
    def setUp(self):
        recorder = mock.patch.object(contcp_t, '_start_experiment_recorder', return_value=None)
        recorder.start()
        self.addCleanup(recorder.stop)
        # 控制台生命周期测试不创建本机 HTTP 监听；端到端监控另有真实 socket 测试。
        self.monitor_bridge = mock.Mock()
        patcher = mock.patch.object(contcp_t, '_start_monitor_bridge', return_value=self.monitor_bridge)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_waiting_for_connection_interruption_stops_sampler_and_closes_listener(self):
        service, listener = mock.Mock(), mock.Mock()
        listener.accept.side_effect = KeyboardInterrupt()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'GamepadService', return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, 'socket', return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.main()
        service.start.assert_called_once()
        service.stop.assert_called_once()
        listener.close.assert_called_once()
        self.monitor_bridge.stop.assert_called_once()

    def test_listener_bind_error_releases_already_started_sampler(self):
        service, listener = mock.Mock(), mock.Mock()
        listener.bind.side_effect = OSError('离线模拟端口占用')
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'GamepadService', return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, 'socket', return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaises(OSError):
                contcp_t.main()
        service.stop.assert_called_once()
        listener.close.assert_called_once()

    def test_optional_sampler_failure_preserves_listener_and_keyboard_path(self):
        service, listener = mock.Mock(), mock.Mock()
        service.start.side_effect = RuntimeError('离线模拟XInput缺失')
        listener.accept.side_effect = KeyboardInterrupt()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'GamepadService', return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, 'socket', return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.main()
        listener.accept.assert_called_once()
        service.stop.assert_called_once()
        listener.close.assert_called_once()

    def test_console_eof_cleans_all_started_resources_without_real_threads(self):
        service, listener, conn, receiver = (mock.Mock() for _ in range(4))
        listener.accept.return_value = (conn, ('127.0.0.1', 12345))
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'GamepadService', return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, 'socket', return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t.threading, 'Thread', return_value=receiver))
            stack.enter_context(mock.patch.object(contcp_t, '_run_console', side_effect=EOFError()))
            stack.enter_context(mock.patch.object(contcp_t, 'send_stream_control', return_value=True))
            neutral = stack.enter_context(mock.patch.object(
                contcp_t, 'send_neutral_and_release', return_value=True))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.main()
        neutral.assert_called_once_with(conn)
        service.stop.assert_called_once()
        receiver.start.assert_called_once()
        receiver.join.assert_called_once_with(timeout=2)
        conn.shutdown.assert_called_once()
        conn.close.assert_called_once()
        listener.close.assert_called_once()
        self.monitor_bridge.set_link.assert_called_once_with('connected', peer='127.0.0.1:12345')
        self.monitor_bridge.stop.assert_called_once()

    def test_receiver_start_failure_still_closes_connection_listener_and_sampler(self):
        service, listener, conn = (mock.Mock() for _ in range(3))
        receiver = mock.Mock(ident=None)
        receiver.start.side_effect = RuntimeError('离线模拟接收线程创建失败')
        receiver.join.side_effect = RuntimeError('禁止join尚未启动的线程')
        listener.accept.return_value = (conn, ('127.0.0.1', 12345))
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'GamepadService', return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, 'socket', return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t.threading, 'Thread', return_value=receiver))
            stack.enter_context(mock.patch.object(contcp_t, 'send_stream_control', return_value=True))
            stack.enter_context(mock.patch.object(contcp_t, 'send_neutral_and_release', return_value=True))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaisesRegex(RuntimeError, '接收线程创建失败'):
                contcp_t.main()
        service.stop.assert_called_once()
        receiver.join.assert_not_called()
        conn.shutdown.assert_called_once()
        conn.close.assert_called_once()
        listener.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
