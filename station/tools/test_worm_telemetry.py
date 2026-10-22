"""双轴遥测版本兼容、校准提示与手柄接管的实际站端链路测试。"""

import contextlib
import csv
import io
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import contcp_t
from groundstation.ethernet_link import EthernetRealtimeLink
from groundstation.legacy_telemetry import decode_telemetry_line
from groundstation.session_recorder import SessionRecorder
from groundstation.state_store import StateStore
from groundstation.telemetry_quality import evaluate_telemetry_quality
from station.tools.test_gamepad_control import AutoStiHarness


def checksummed(values):
    """使用真实校验，不让长度错误被坏校验提前遮住。"""
    body = 'UGWRM,' + ','.join(str(value) for value in values)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '$%s*%02X' % (body, checksum)


def worm_line(schema=3, **changes):
    fields = contcp_t.UGWRM_FIELDS_V2 if schema == 2 else contcp_t.UGWRM_FIELDS_V3
    row = dict.fromkeys(fields, 0)
    row.update(schema=schema, seq=9, t_ms=123, span_valid=1, data_fresh=1,
               home_state=3, pulse=-377, stroke_percent=0.1333)
    if schema == 3:
        row.update(roll_pulse=500, roll_pulse_min=-500, roll_pulse_max=1500,
                   roll_span_pulses=2000, roll_percent=50.0, roll_valid=1,
                   roll_home_state=6, calib_state=2)
    row.update(changes)
    return checksummed([row[name] for name in fields])


class WormTelemetryTests(unittest.TestCase):
    def test_v2_preserves_prefix_and_missing_roll_is_none(self):
        self.assertEqual(len(contcp_t.UGWRM_FIELDS_V2), 30)
        self.assertEqual(len(contcp_t.UGWRM_FIELDS_V3), 41)
        self.assertEqual(contcp_t.UGWRM_FIELDS_V3[:30], contcp_t.UGWRM_FIELDS_V2)
        row = contcp_t.ugwrm_parse(worm_line(2))
        self.assertEqual(row['pulse'], -377)
        self.assertAlmostEqual(row['stroke_percent'], 0.1333)
        for name in contcp_t.UGWRM_FIELDS_V3[30:]:
            self.assertIsNone(row[name])

    def test_v3_preserves_signed_integer_precision_and_percent(self):
        row = contcp_t.ugwrm_parse(worm_line(roll_pulse=-2147483648,
            roll_pulse_min=-2147483648, roll_pulse_max=2147483647,
            roll_span_pulses=2147483647, roll_percent=12.345))
        self.assertEqual(row['roll_pulse'], -2147483648)
        self.assertEqual(row['roll_pulse_max'], 2147483647)
        self.assertIsInstance(row['roll_span_pulses'], int)
        self.assertEqual(row['roll_percent'], 12.345)

    def test_version_and_field_count_are_checked_together(self):
        for schema, count in ((2, 41), (3, 30), (3, 40), (3, 42), (1, 30), (4, 41)):
            with self.subTest(schema=schema, count=count):
                self.assertIsNone(contcp_t.ugwrm_parse(checksummed([schema] + [0] * (count - 1))))
        self.assertIsNone(contcp_t.ugwrm_parse(checksummed([])))

    def test_bad_checksum_or_numeric_field_is_rejected(self):
        line = worm_line()
        self.assertIsNone(contcp_t.ugwrm_parse(line[:-2] + ('00' if line[-2:] != '00' else '01')))
        self.assertIsNone(contcp_t.ugwrm_parse(worm_line(roll_pulse='1.5')))
        self.assertIsNone(contcp_t.ugwrm_parse(worm_line(calib_state='bad')))

    def test_csv_has_stable_columns_and_empty_legacy_tail(self):
        rows = [contcp_t.ugwrm_csv_row(contcp_t.ugwrm_parse(worm_line(schema))) for schema in (2, 3)]
        for row in rows:
            self.assertEqual(len(row), len(contcp_t.UGWRM_CSV_HEADER))
        self.assertEqual(rows[0][31:], [''] * 11)
        self.assertEqual(rows[1][contcp_t.UGWRM_CSV_HEADER.index('roll_percent')], 50.0)

    def test_pitch_done_does_not_report_combined_completion(self):
        row = contcp_t.ugwrm_parse(worm_line(calib_state=1, roll_home_state=5))
        key, notice = contcp_t.ugwrm_calibration_notice(row)
        self.assertIn('校准中', notice)
        self.assertIn('返回 50%', notice)
        self.assertNotIn('双轴校准完成', notice)
        row.update(calib_state=2, roll_home_state=6, calib_hold=1)
        next_key, notice = contcp_t.ugwrm_calibration_notice(row)
        self.assertNotEqual(key, next_key)
        self.assertIn('双轴校准完成', notice)
        self.assertIn('等待中立重新接管', notice)

    def test_failure_cancel_and_legacy_notices_are_distinct(self):
        for state, fail, expected in ((3, 2, 'roll 失败'), (4, 3, '已取消'), (4, 4, 'PID 接管')):
            with self.subTest(state=state, fail=fail):
                _, notice = contcp_t.ugwrm_calibration_notice(contcp_t.ugwrm_parse(
                    worm_line(calib_state=state, calib_fail=fail, roll_home_fail=7)))
                self.assertIn(expected, notice)
                self.assertNotIn('双轴校准完成', notice)
        _, notice = contcp_t.ugwrm_calibration_notice(contcp_t.ugwrm_parse(worm_line(2)))
        self.assertIn('未报告 roll 状态', notice)

    def test_auto_ready_uses_total_state_and_waits_for_hold_release(self):
        ctl = (1, {'mode': 0, 'armed': 0}, 10.0)
        for state, hold, ready, rejected in ((1, 0, False, True), (2, 1, False, False),
                (2, 0, True, False), (3, 0, True, False), (4, 0, True, False),
                (0, 0, True, False), (99, 0, False, True), (2, None, False, True)):
            with self.subTest(state=state, hold=hold):
                row = contcp_t.ugwrm_parse(worm_line(calib_state=state))
                row['calib_hold'] = hold
                result, reason = contcp_t._check_auto_sti_ready(ctl, (1, row, 10.0), 0, 0, 10.1, .5)
                self.assertEqual(result, ready)
                self.assertEqual(bool(reason), rejected)

    def test_new_pitch_failure_codes_are_explained_and_change_notice_key(self):
        previous = None
        for fail, expected in ((6, '两端限位同时触发'), (7, 'roll 失败，联合停止')):
            row = contcp_t.ugwrm_parse(worm_line(calib_state=3, calib_fail=1, home_state=4, home_fail=fail))
            key, notice = contcp_t.ugwrm_calibration_notice(row)
            self.assertIn(expected, notice)
            self.assertNotEqual(key, previous)
            previous = key

    def test_hold_waiting_notice_only_describes_terminal_states(self):
        for state in range(5):
            with self.subTest(state=state):
                row = contcp_t.ugwrm_parse(worm_line(calib_state=state, calib_hold=1))
                _, notice = contcp_t.ugwrm_calibration_notice(row)
                self.assertEqual('等待中立重新接管' in notice, state in (2, 3, 4))

    def test_completed_notice_does_not_present_invalid_position_as_zero(self):
        for changes, expected in (({'data_fresh': 0}, 'pitch=不可用，roll=50.00%'),
                ({'roll_valid': 0}, 'roll=不可用'),
                ({'stroke_percent': 'inf', 'roll_percent': 'nan'}, 'pitch=不可用，roll=不可用')):
            with self.subTest(changes=changes):
                _, notice = contcp_t.ugwrm_calibration_notice(contcp_t.ugwrm_parse(worm_line(**changes)))
                self.assertIn(expected, notice)
        _, legacy = contcp_t.ugwrm_calibration_notice(contcp_t.ugwrm_parse(worm_line(2, span_valid=0)))
        self.assertIn('位置=不可用', legacy)

    def test_pitch_and_roll_validity_are_independent(self):
        row = contcp_t.ugwrm_parse(worm_line(data_fresh=0, roll_valid=1))
        _, invalid, _ = evaluate_telemetry_quality('ugwrm', row)
        self.assertIn('stroke_percent', invalid)
        self.assertNotIn('roll_percent', invalid)
        row.update(data_fresh=1, roll_valid=0)
        _, invalid, _ = evaluate_telemetry_quality('ugwrm', row)
        self.assertEqual(invalid, {'roll_percent': 'roll_reference_invalid'})
        self.assertNotIn('roll_pulse', invalid)

    def test_legacy_roll_missing_and_non_finite_roll_are_not_valid(self):
        row = contcp_t.ugwrm_parse(worm_line(2))
        clean, _, quality = evaluate_telemetry_quality('ugwrm', row)
        self.assertIsNone(clean['roll_percent'])
        self.assertIn('roll_validity_not_reported', quality['reasons'])
        row = contcp_t.ugwrm_parse(worm_line(roll_percent='nan'))
        clean, invalid, _ = evaluate_telemetry_quality('ugwrm', row)
        self.assertIsNone(clean['roll_percent'])
        self.assertEqual(invalid['roll_percent'], 'non_finite')

    def test_cli_receiver_handles_fragmented_mixed_versions_and_writes_blank_tail(self):
        diagnostic = {name: 2147483647 for name in contcp_t.UGWRM_FIELDS_V2[18:]}
        line = worm_line(**diagnostic, pulse_min=-2147483648, pulse_max=2147483647,
            roll_pulse=-2147483648, roll_pulse_min=-2147483648,
            roll_pulse_max=2147483647, roll_span_pulses=2147483647)
        self.assertGreater(len(line), 256)
        wire = (worm_line(2) + '\r\n' + line + '\r\n').encode('ascii')
        conn, wrm, closed = mock.Mock(), contcp_t.UgwrmStore(), threading.Event()
        conn.recv.side_effect = [wire[:31], wire[31:180], wire[180:270], wire[270:], b'']
        with tempfile.TemporaryDirectory() as temporary, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, 'running', True))
            for name in ('EKF_LOG_DIR', 'CONTROL_LOG_DIR', 'BATTERY_LOG_DIR'):
                stack.enter_context(mock.patch.object(contcp_t, name, temporary))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.receive_thread(conn, wrm_store=wrm, link_closed=closed)
            with next(Path(temporary).glob('worm_*.csv')).open(encoding='utf-8', newline='') as handle:
                rows = list(csv.reader(handle))
        self.assertEqual(wrm.snapshot()[0], 2)
        self.assertEqual(wrm.snapshot()[1]['schema'], 3)
        self.assertEqual(rows[0], contcp_t.UGWRM_CSV_HEADER)
        self.assertEqual(rows[1][31:], [''] * 11)
        self.assertEqual(rows[2][35], '50.0')
        self.assertTrue(closed.is_set())

    def test_recording_keeps_both_versions_in_one_compatible_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            try:
                recorder.begin_session('mixed', 'local-test')
                for schema in (2, 3):
                    line = worm_line(schema)
                    recorder.record(line, decode_telemetry_line(line))
                recorder.end_session('test_finished')
            finally:
                recorder.stop()
            path = recorder.snapshot()['files']['ugwrm']
            with Path(path).open(encoding='utf-8', newline='') as handle:
                rows = list(csv.reader(handle))
            self.assertEqual([len(row) for row in rows], [42, 42, 42])
            self.assertEqual(rows[1][31:], [''] * 11)
            self.assertEqual(rows[2][35], '50.0')

    def test_urgc_receives_fragmented_v3_without_new_topic_or_subscription(self):
        store = StateStore()
        link = EthernetRealtimeLink(store, host='127.0.0.1', port=0)
        client = None
        try:
            link.start()
            self.assertTrue(link.wait_until_listening(2.0))
            client = socket.create_connection(link.bound_endpoint, timeout=2.0)
            client.recv(64)
            wire = (worm_line(calib_state=1, roll_home_state=5) + '\r\n').encode('ascii')
            for part in (wire[:10], wire[10:40], wire[40:]):
                client.sendall(part)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and 'ugwrm' not in store.read()[1]['telemetry']:
                time.sleep(.01)
            sample = store.read()[1]['telemetry']['ugwrm']
            self.assertEqual(sample['data']['calib_state'], 1)
            self.assertEqual(sample['data']['roll_percent'], 50.0)
        finally:
            if client is not None:
                client.close()
            link.stop()


class CombinedCalibHarness(AutoStiHarness):
    """复用真实 auto_sti 循环，模拟中立命令解除保持及校准开始。"""

    def __init__(self, state=2, release_at=.35, start_at=None):
        super().__init__([(0.50, {'dpad_left': True})], fault_at=.80)
        self.calib_state = state
        self.release_at = release_at
        self.start_at = start_at

    def publish_telemetry(self):
        before = self.wrm_store.snapshot()[0]
        super().publish_telemetry()
        if self.wrm_store.snapshot()[0] != before:
            neutral_sent = any(not self.is_motion(payload) for _, payload in self.controls())
            hold = int(not neutral_sent or self.elapsed < self.release_at)
            state = 1 if self.start_at is not None and self.elapsed >= self.start_at else self.calib_state
            self.wrm_store.publish({'schema': 3, 'home_state': 3, 'calib_state': state,
                                    'calib_hold': hold, 'roll_home_state': 5 if state == 1 else 6})


class CombinedCalibControlTests(unittest.TestCase):
    def test_preparing_sends_neutral_until_board_releases_hold(self):
        harness = CombinedCalibHarness()
        harness.run()
        self.assertTrue(harness.motions())
        self.assertTrue(any(at < .35 for at, _ in harness.controls()))
        self.assertTrue(all(at >= .35 for at, _ in harness.motions()))

    def test_roll_busy_blocks_takeover_even_when_pitch_is_done(self):
        harness = CombinedCalibHarness(state=1)
        harness.run()
        self.assertEqual(harness.motions(), [])

    def test_calibration_start_during_active_mode_stops_roll_and_releases(self):
        harness = CombinedCalibHarness(release_at=0, start_at=.65)
        harness.run()
        self.assertTrue(harness.motions())
        self.assertTrue(all(at < .75 for at, _ in harness.motions()))
        self.assertEqual(harness.sent[-3][1][2], bytes(18) + b'\x01')
        self.assertEqual(harness.sent[-2][1][2], bytes(19))


if __name__ == '__main__':
    unittest.main()
