"""尾舵 AUTO 键盘、协议和退出保护的离线验证，不连接真实控制板。"""

import contextlib
import io
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

STATION_DIR = Path(__file__).resolve().parent.parent
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import contcp_t
import ugcp_protocol as ugcp


class AutoLoopHarness:
    """用虚拟键盘和时钟驱动真实 AUTO 循环，同时记录最终线上的字节。"""

    def __init__(self, frames, fault=None):
        self.frames = frames
        self.fault = fault
        self.loop = 0
        self.now = 0.0
        self.sent = []
        self.attempts = []

    def GetAsyncKeyState(self, vk):
        keys = self.frames[min(self.loop, len(self.frames) - 1)]
        return 0x8000 if any(contcp_t.AUTO_VK_MAP[key] == vk for key in keys) else 0

    def sendall(self, frame):
        parsed = ugcp.parse_frame(frame)
        self.attempts.append(parsed)
        if self.fault == 'send' and parsed[0] == ugcp.MSG_CONTROL21_STREAM:
            raise OSError('离线模拟断线')
        self.sent.append((self.now, parsed, bytes(frame)))

    def sleep(self, duration):
        self.now += duration
        if duration == contcp_t.AUTO_KEY_POLL_S:
            self.loop += 1
            if self.fault == 'interrupt':
                raise KeyboardInterrupt()
            if self.fault == 'running_false':
                contcp_t.running = False
            if self.loop >= len(self.frames):
                raise AssertionError('AUTO 未按预定退出')

    def run(self):
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t.os, 'name', 'nt'))
            stack.enter_context(mock.patch.object(
                contcp_t.ctypes, 'windll', SimpleNamespace(user32=self), create=True))
            stack.enter_context(mock.patch.object(contcp_t.time, 'monotonic', lambda: self.now))
            stack.enter_context(mock.patch.object(contcp_t.time, 'sleep', self.sleep))
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.run_auto_mode(self)

    def controls(self):
        return [(when, parsed[2]) for when, parsed, _ in self.sent
                if parsed[0] == ugcp.MSG_CONTROL21_STREAM]


class AutoRudderProtocolTests(unittest.TestCase):
    def test_control21_appends_signed_angle_without_changing_legacy_layout(self):
        old = (1, -50, 1, -7, 0, 1, 25, 30, 0, 1, 1, 1, 1)
        for angle in (-4500, -1000, 0, 1000, 4500):
            with self.subTest(angle=angle):
                payload = ugcp.pack_control21_payload(*old, angle)
                self.assertEqual(payload[:19], ugcp.pack_control_payload(*old))
                self.assertEqual(len(payload), 21)
                self.assertEqual(struct.unpack_from('<h', payload, 19)[0], angle)
                frame = ugcp.build_frame(ugcp.MSG_CONTROL21_STREAM, payload, 0x1234)
                self.assertEqual(len(frame), 35)
                self.assertEqual(frame[5], 0x07)
                self.assertEqual(ugcp.parse_frame(frame), (0x07, 0x1234, payload))

    def test_control21_rejects_bad_angles(self):
        packet = contcp_t._build_default_auto_packet()
        for angle in (-4501, 4501, 1.5, float('nan'), float('inf'), None):
            with self.subTest(angle=angle):
                packet['rudder_angle_cdeg'] = angle
                with self.assertRaises(ugcp.UgcpProtocolError):
                    ugcp.pack_control21_payload(**packet)

    def test_snapshot_reports_pwm_command_and_mode_without_claiming_feedback(self):
        snap = contcp_t.EthernetUartSystemSnapshot()
        snap.servo_mode = 2
        snap.servo_pulse_us = 1611
        text = contcp_t.format_rudder_status(snap)
        self.assertIn('applied_pulse_us=1611', text)
        self.assertIn('网线命令', text)
        self.assertIn('非实测角度', text)


class AutoRudderTargetTests(unittest.TestCase):
    def setUp(self):
        self.packet = contcp_t._build_default_auto_packet()
        self.state = {'angle_deg': 0.0, 'direction': 0}
        self.previous = set()

    def step(self, keys=(), dt=0.1):
        keys = set(keys)
        pressed = {key: key in keys for key in contcp_t.AUTO_VK_MAP}
        rising = {key: key in keys and key not in self.previous
                  for key in contcp_t.AUTO_VK_MAP}
        self.previous = keys
        return contcp_t._update_auto_rudder(
            self.packet, self.state, pressed, rising, dt)

    def test_key_map_preserves_existing_controls_and_has_no_conflicts(self):
        self.assertEqual(set(contcp_t.AUTO_VK_MAP), set('WSXEDCIKJLUNZHQRFVT') | {'ESC'})
        self.assertEqual(len(set(contcp_t.AUTO_VK_MAP.values())), len(contcp_t.AUTO_VK_MAP))

    def test_single_press_runs_until_stop_then_holds_target(self):
        self.assertTrue(self.step('R'))
        self.step()
        self.assertEqual(self.packet['rudder_angle_cdeg'], 200)
        self.assertTrue(self.step('V'))
        for _ in range(20):
            self.step()
        self.assertEqual(self.packet['rudder_angle_cdeg'], 200)
        self.assertEqual(self.state['direction'], 0)
        self.step('F')
        self.assertEqual(self.packet['rudder_angle_cdeg'], 100)

    def test_endpoints_and_pc_pause_do_not_allow_large_jump(self):
        self.step('R', dt=60.0)
        self.assertEqual(self.packet['rudder_angle_cdeg'], 100)
        for _ in range(100):
            self.step()
        self.assertEqual(self.packet['rudder_angle_cdeg'], 4500)
        self.assertEqual(self.state['direction'], 0)
        self.step('F', dt=0.0)
        self.step()
        self.assertEqual(self.packet['rudder_angle_cdeg'], 4400)
        for _ in range(100):
            self.step()
        self.assertEqual(self.packet['rudder_angle_cdeg'], -4500)
        self.assertEqual(self.state['direction'], 0)

    def test_stop_center_and_zero_override_held_and_new_direction_keys(self):
        for stop in ('V', 'T', 'Z'):
            with self.subTest(stop=stop):
                self.setUp()
                self.step('R')
                self.step(stop)
                self.step(stop + 'F')
                self.step('F')
                self.assertEqual(self.state['direction'], 0)
                self.assertEqual(self.packet['rudder_angle_cdeg'], 100 if stop == 'V' else 0)
                self.step()
                self.step('F')
                self.assertEqual(self.state['direction'], -1)

    def test_simultaneous_directions_stop_and_require_new_press(self):
        self.step('R')
        self.step('RF')
        self.step('F')
        self.assertEqual(self.packet['rudder_angle_cdeg'], 100)
        self.assertEqual(self.state['direction'], 0)
        self.step()
        self.step('F')
        self.assertEqual(self.packet['rudder_angle_cdeg'], 0)


class AutoRudderLoopTests(unittest.TestCase):
    def assert_neutral_release(self, harness):
        final = [entry[1] for entry in harness.sent[-2:]]
        self.assertEqual([entry[0] for entry in final], [0x04, 0x04])
        self.assertEqual(final[0][2], bytes(18) + b'\x01')
        self.assertEqual(final[1][2], bytes(19))

    def test_auto_streams_10hz_and_direction_stop_center_send_promptly(self):
        frames = [set(), {'R'}] + [set()] * 11 + [{'V'}] + [set()] * 5
        frames += [{'F'}] + [set()] * 5 + [{'T'}, set(), {'Q'}]
        harness = AutoLoopHarness(frames)
        harness.run()
        controls = harness.controls()
        self.assertTrue(all(len(payload) == 21 for _, payload in controls))
        self.assertTrue(all(b - a <= 0.121 for (a, _), (b, _) in zip(controls, controls[1:])))
        for index in (1, 13, 19, 25):
            self.assertTrue(any(abs(when - index * 0.02) < 1e-8 for when, _ in controls))
        stop_payloads = [payload for when, payload in controls if 0.26 <= when < 0.38]
        self.assertGreater(len(stop_payloads), 0)
        self.assertEqual(len({struct.unpack_from('<h', payload, 19)[0] for payload in stop_payloads}), 1)
        self.assertEqual(struct.unpack_from('<h', controls[-1][1], 19)[0], 0)
        self.assert_neutral_release(harness)

    def test_z_overrides_other_actions_in_same_poll(self):
        harness = AutoLoopHarness([set(), {'R', 'W', 'E'}, {'Z', 'F', 'S', 'I'}]
                                  + [{'F', 'I'}] * 6 + [{'Q'}])
        harness.run()
        self.assertEqual(harness.controls()[-1][1], bytes(18) + b'\x01\x00\x00')
        self.assert_neutral_release(harness)

    def test_q_and_escape_exit_send_neutral_release(self):
        for key in ('Q', 'ESC'):
            with self.subTest(key=key):
                harness = AutoLoopHarness([set(), {'R'}, {key}])
                harness.run()
                self.assertGreater(struct.unpack_from('<h', harness.controls()[-1][1], 19)[0], 0)
                self.assert_neutral_release(harness)

    def test_send_failure_exits_without_more_motion_commands(self):
        harness = AutoLoopHarness([{'R'}, {'Q'}], fault='send')
        harness.run()
        self.assertEqual([item[0] for item in harness.attempts], [0x07, 0x04, 0x04])
        self.assert_neutral_release(harness)

    def test_disconnect_and_keyboard_interrupt_always_attempt_release(self):
        for fault in ('running_false', 'interrupt'):
            with self.subTest(fault=fault):
                harness = AutoLoopHarness([set(), {'Q'}], fault=fault)
                if fault == 'interrupt':
                    with self.assertRaises(KeyboardInterrupt):
                        harness.run()
                else:
                    harness.run()
                self.assert_neutral_release(harness)


if __name__ == '__main__':
    unittest.main()
