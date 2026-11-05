"""验证尾舵发送记录的真实发送边界、只读显示和实验CSV，不连接控制板。"""

import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import contcp_t
from experiment_recorder import ExperimentRecorder
import monitor
from monitor_bridge import MonitorBridge
import ugcp_protocol as ugcp


class FakeSocket:
    def __init__(self, error=None):
        self.error = error
        self.frames = []

    def sendall(self, frame):
        self.frames.append(bytes(frame))
        if self.error is not None:
            raise self.error


class RudderMonitorTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.now = 100.0
        self.bridge = MonitorBridge(port=0, clock=lambda: self.now)
        self.bridge.set_link('connected', peer='offline-test')
        self.addCleanup(self.bridge.stop)
        for name, value in (
                ('CONTROL_MONITOR_BRIDGE', self.bridge),
                ('EXPERIMENT_RECORDER', None), ('EXPERIMENT_CONTEXT', threading.local()),
                ('UGCP_TX_SEQUENCE', ugcp.SequenceCounter())):
            self.stack.enter_context(mock.patch.object(contcp_t, name, value))
        self.stack.enter_context(mock.patch.object(
            contcp_t.time, 'monotonic_ns', lambda: int(self.now * 1e9)))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        contcp_t._experiment_context(source='auto', phase='CONSOLE')

    def command(self):
        return self.bridge.snapshot()['commands']['rudder']

    def send(self, angle, conn=None):
        packet = contcp_t._build_default_auto_packet()
        packet['rudder_angle_cdeg'] = angle
        return contcp_t.send_auto_control_packet(conn or FakeSocket(), packet)

    def test_success_uses_actual_wire_target_and_owned_snapshot(self):
        self.assertIsNone(self.command())
        for angle in (-4500, 4500):
            conn = FakeSocket()
            self.assertTrue(self.send(angle, conn))
            command = self.command()
            self.assertEqual(command['angle_cdeg'], angle)
            self.assertEqual(command['angle_deg'], angle / 100.0)
            self.assertEqual(ugcp.parse_frame(conn.frames[-1])[0], 0x07)
            self.assertEqual(command['source'], 'auto')
            self.assertEqual(command['phase'], 'CONSOLE')
            self.assertTrue(command['sent_at'])
            self.assertTrue(command['fresh'])
            self.assertEqual(command['age_ms'], 0)
            panel = monitor.render_panel(self.bridge.snapshot(), width=150)
            self.assertIn(f'{angle / 100.0:+.2f}°', panel)
            self.assertIn('来源：auto/CONSOLE', panel)
            self.assertIn('非板端执行确认/实测角度', panel)
            command['angle_deg'] = 99
            self.assertEqual(self.command()['angle_deg'], angle / 100.0)

    def test_failed_and_interrupted_sends_never_publish_or_refresh_success(self):
        self.assertFalse(self.send(4500, FakeSocket(OSError('partial send'))))
        self.assertIsNone(self.command())
        self.assertTrue(self.send(3000))
        original = self.command()
        self.now += 1.0
        self.assertFalse(self.send(-4500, FakeSocket(OSError('partial send'))))
        with self.assertRaises(KeyboardInterrupt):
            self.send(-4500, FakeSocket(KeyboardInterrupt()))
        current = self.command()
        self.assertEqual(current['angle_deg'], 30.0)
        self.assertEqual(current['seq'], original['seq'])
        self.assertEqual(current['sent_at'], original['sent_at'])
        self.assertEqual(current['age_ms'], 1000)

    def test_old_stream_neutral_and_release_publish_zero_with_distinct_meaning(self):
        self.assertTrue(self.send(4500))
        values = (0,) * 12 + (1,)
        self.assertTrue(contcp_t.send_control_packet(FakeSocket(), *values))
        self.assertEqual(self.command()['angle_deg'], 0)
        self.assertEqual(self.command()['msg_type'], 0x01)
        self.assertTrue(self.send(-4500))
        with mock.patch.object(contcp_t.time, 'sleep'):
            self.assertTrue(contcp_t.send_neutral_and_release(FakeSocket()))
        command = self.command()
        self.assertEqual(command['angle_deg'], 0)
        self.assertEqual(command['phase'], 'RELEASE')
        self.assertEqual(command['eth_control_flag'], 0)
        self.assertIn('已发释放请求', monitor.render_panel(self.bridge.snapshot()))

    def test_stale_disconnect_and_reconnect_do_not_claim_current_position(self):
        self.send(3500)
        self.now += 2.501
        self.assertFalse(self.command()['fresh'])
        panel = monitor.render_panel(self.bridge.snapshot(), width=150)
        self.assertIn('+35.00°', panel)
        self.assertIn('发送记录超龄', panel)
        self.bridge.set_link('disconnected')
        self.assertFalse(self.command()['fresh'])
        panel = monitor.render_panel(self.bridge.snapshot(), width=150)
        self.assertIn('板端断开，仅历史记录', panel)
        self.assertIn('+35.00°', panel)
        self.bridge.set_link('connected')
        self.assertIsNone(self.command())
        self.assertNotIn('+35.00°', monitor.render_panel(self.bridge.snapshot()))

    def test_monitor_unavailable_old_session_and_invalid_angle_hide_value(self):
        self.send(4500)
        snapshot = self.bridge.snapshot()
        self.assertNotIn('+45.00°', monitor.render_panel(snapshot, error='接口失联'))
        snapshot['commands']['rudder']['session_id'] = 'old-session'
        self.assertNotIn('+45.00°', monitor.render_panel(snapshot))
        for angle in (float('nan'), 99, True):
            snapshot = self.bridge.snapshot()
            snapshot['commands']['rudder']['angle_deg'] = angle
            self.assertIn('数值无效', monitor.render_panel(snapshot))

    def test_monitor_failure_does_not_change_success_or_block_release(self):
        with mock.patch.object(self.bridge, 'publish_rudder_command', side_effect=RuntimeError('bridge')):
            self.assertTrue(self.send(4500))
            with mock.patch.object(contcp_t.time, 'sleep'):
                self.assertTrue(contcp_t.send_neutral_and_release(FakeSocket()))

    def test_real_csv_keeps_degrees_failure_and_release(self):
        buildtmp = STATION_DIR.parents[1] / '.buildtmp'
        buildtmp.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='rudder_record_', dir=buildtmp) as directory:
            recorder = ExperimentRecorder(Path(directory))
            saved = Path(recorder.start(peer='offline-test'))
            try:
                contcp_t.EXPERIMENT_RECORDER = recorder
                self.send(4500)
                self.send(-1234, FakeSocket(OSError('partial')))
                with mock.patch.object(contcp_t.time, 'sleep'):
                    contcp_t.send_neutral_and_release(FakeSocket())
            finally:
                recorder.stop()
                contcp_t.EXPERIMENT_RECORDER = None
            with (saved / 'commands.csv').open(encoding='utf-8-sig', newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([float(row['rudder_angle_deg']) for row in rows], [45, -12.34, 0, 0])
            self.assertEqual([row['rudder_angle_cdeg'] for row in rows], ['4500', '-1234', '0', '0'])
            self.assertEqual([row['success'] for row in rows], ['True', 'False', 'True', 'True'])
            self.assertEqual([row['phase'] for row in rows[-2:]], ['NEUTRAL', 'RELEASE'])

    def test_ozone_source_is_named_without_claiming_pc_knows_its_angle(self):
        snap = contcp_t.EthernetUartSystemSnapshot()
        snap.servo_mode = 3
        snap.servo_pulse_us = 2000
        self.assertIn('Ozone 调试', contcp_t.format_rudder_status(snap))
        self.assertIsNone(self.command())

    def test_gamepad_loop_records_sent_angles_inputs_and_center_in_csv_and_monitor(self):
        # 复用真实模式循环和UGCP编码，只用本地输入/时钟/套接字夹具替代硬件。
        from test_gamepad_control import AutoStiHarness
        harness = AutoStiHarness([
            (0.20, {'right_trigger': 1.0}),
            (0.36, {'button_b': True, 'rudder_center_seq': 1}),
            (0.50, {'button_b': False, 'right_trigger': 0.0})], fault_at=0.70)
        observed = []
        original_send = contcp_t.send_auto_control_packet

        def send_and_observe(*args, **kwargs):
            result = original_send(*args, **kwargs)
            observed.append(self.command())
            return result

        buildtmp = STATION_DIR.parents[1] / '.buildtmp'
        buildtmp.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='gamepad_rudder_', dir=buildtmp) as directory:
            recorder = ExperimentRecorder(Path(directory))
            saved = Path(recorder.start(peer='offline-gamepad'))
            try:
                contcp_t.EXPERIMENT_RECORDER = recorder
                with mock.patch.object(contcp_t, 'send_auto_control_packet', send_and_observe):
                    harness.run()
            finally:
                recorder.stop()
                contcp_t.EXPERIMENT_RECORDER = None
            with (saved / 'commands.csv').open(encoding='utf-8-sig', newline='') as handle:
                rows = list(csv.DictReader(handle))
        active = [row for row in rows if row['source'] == 'auto_sti' and row['phase'] == 'ACTIVE']
        self.assertTrue(active)
        moving = [row for row in active if float(row['rudder_angle_deg']) > 0]
        self.assertTrue(moving)
        intent = json.loads(moving[0]['input_json'])
        self.assertEqual(intent['right_trigger'], 1.0)
        self.assertEqual(intent['rudder_rate_deg_s'], 30.0)
        centered = [row for row in active if json.loads(row['input_json'])['button_b']]
        self.assertTrue(centered)
        self.assertTrue(all(float(row['rudder_angle_deg']) == 0 for row in centered))
        self.assertTrue(all(json.loads(row['input_json'])['rudder_wait_release'] for row in centered))
        self.assertTrue(all(row['success'] == 'True' for row in active))
        self.assertTrue(any(cmd['angle_deg'] > 0 for cmd in observed))
        self.assertTrue(all(cmd['source'] == 'auto_sti' for cmd in observed))
        self.assertEqual(observed[-1]['angle_deg'], 0)


if __name__ == '__main__':
    unittest.main()
