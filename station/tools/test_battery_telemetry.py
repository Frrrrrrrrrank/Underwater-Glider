"""电池解析、主动告警和唯一接收线程的离线回归。"""

import contextlib
import io
import os
import socket
import sys
import threading
import unittest
from unittest import mock


STATION_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if STATION_DIR not in sys.path:
    sys.path.insert(0, STATION_DIR)

import battery_telemetry as battery
import contcp_t
import ugcp_protocol as ugcp


def frame(**updates):
    row = dict(schema=1, seq=1, t_ms=1000, adc_raw=21000,
               battery_mv=23500, filtered_mv=23490, valid=1,
               state=battery.BATTERY_NORMAL, sample_age_ms=10, error_count=0)
    row.update(updates)
    return sentence("UGBAT," + ",".join(str(row[name]) for name in battery.UGBAT_FIELDS))


def sentence(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}\r\n"


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class BatteryParseTests(unittest.TestCase):
    def test_valid_frame_and_csv_keep_integer_millivolts(self):
        row = battery.ugbat_parse(frame())
        self.assertEqual(row["filtered_mv"], 23490)
        self.assertEqual(row["adc_raw"], 21000)
        values = battery.ugbat_csv_row(row)
        self.assertEqual(len(values), len(battery.UGBAT_CSV_HEADER))
        self.assertTrue(all(type(value) is int for value in values))
        self.assertEqual(values[battery.UGBAT_CSV_HEADER.index("filtered_mv")], 23490)

    def test_rejects_bad_crc_missing_crc_trailing_garbage_and_whitespace(self):
        valid = frame()
        for invalid in (valid.replace("23490", "23491"), valid.split("*")[0],
                        valid.rstrip() + "junk", " " + valid, valid.rstrip() + " ",
                        valid.replace("*", "**"), valid.replace("UGBAT", "UGBATX")):
            with self.subTest(invalid=invalid):
                self.assertIsNone(battery.ugbat_parse(invalid))

    def test_rejects_wrong_schema_integer_ranges_and_noninteger_fields(self):
        cases = [("schema", 0), ("schema", 2), ("seq", 0x100000000),
                 ("t_ms", -1), ("adc_raw", 65536), ("battery_mv", 100001),
                 ("filtered_mv", 100001), ("valid", 2), ("state", 5),
                 ("sample_age_ms", 0x100000000), ("error_count", 0x100000000),
                 ("battery_mv", "nan"), ("battery_mv", "21.0"),
                 ("battery_mv", "+21000"), ("battery_mv", " 21000"),
                 ("battery_mv", "２１０００"), ("battery_mv", ""),
                 ("battery_mv", "0" * 100)]
        for name, value in cases:
            with self.subTest(name=name, value=value):
                self.assertIsNone(battery.ugbat_parse(frame(**{name: value})))

    def test_rejects_missing_or_extra_fields_even_with_valid_crc(self):
        body = frame()[1:].split("*")[0]
        for invalid in (body.rsplit(",", 1)[0], body + ",0"):
            self.assertIsNone(battery.ugbat_parse(sentence(invalid)))

    def test_accepts_boundaries_and_invalid_state_for_fault_reporting(self):
        row = battery.ugbat_parse(frame(seq=0xFFFFFFFF, t_ms=0xFFFFFFFF,
            adc_raw=65535, battery_mv=100000, filtered_mv=0,
            valid=0, state=battery.BATTERY_INVALID,
            sample_age_ms=0xFFFFFFFF, error_count=0xFFFFFFFF))
        self.assertIsNotNone(row)
        self.assertEqual(row["valid"], 0)


class BatteryAlarmTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.messages = []
        self.monitor = battery.BatteryAlarmMonitor(self.messages.append, self.clock)
        self.sequence = 0

    def feed(self, state=battery.BATTERY_NORMAL, **changes):
        self.sequence += 1
        row = battery.ugbat_parse(frame(seq=self.sequence,
            t_ms=int(self.clock() * 1000), state=state, **changes))
        self.assertIsNotNone(row)
        self.monitor.ingest(row)

    def test_normal_frames_print_ready_once(self):
        for _ in range(10):
            self.feed()
            self.clock.advance(1)
            self.monitor.poll()
        self.assertEqual(len(self.messages), 1)
        self.assertIn("监测已就绪", self.messages[0])

    def test_warn_is_immediate_and_repeats_every_thirty_seconds(self):
        self.feed(battery.BATTERY_WARN, filtered_mv=21800)
        self.assertIn("低压预警", self.messages[0])
        self.assertIn("21.80 V", self.messages[0])
        for _ in range(29):
            self.clock.advance(1)
            self.feed(battery.BATTERY_WARN, filtered_mv=21800)
        self.assertEqual(len(self.messages), 1)
        self.clock.advance(1)
        self.feed(battery.BATTERY_WARN, filtered_mv=21800)
        self.assertEqual(len(self.messages), 2)

    def test_critical_upgrades_immediately_and_repeats_every_ten_seconds(self):
        self.feed(battery.BATTERY_WARN)
        self.clock.advance(1)
        self.feed(battery.BATTERY_CRITICAL, filtered_mv=20800)
        self.assertIn("严重低压", self.messages[-1])
        self.assertEqual(len(self.messages), 2)
        for _ in range(9):
            self.clock.advance(1)
            self.feed(battery.BATTERY_CRITICAL, filtered_mv=20800)
        self.assertEqual(len(self.messages), 2)
        self.clock.advance(1)
        self.feed(battery.BATTERY_CRITICAL, filtered_mv=20800)
        self.assertEqual(len(self.messages), 3)

    def test_downgrade_remains_warning_and_normal_recovers_once(self):
        self.feed(battery.BATTERY_CRITICAL)
        self.feed(battery.BATTERY_WARN)
        self.assertIn("但仍处于低压预警", self.messages[-1])
        self.assertNotIn("低压解除", self.messages[-1])
        self.feed()
        self.assertIn("低压解除", self.messages[-1])
        self.feed()
        self.assertEqual(len(self.messages), 3)

    def test_fault_does_not_clear_previous_low_warning(self):
        self.feed(battery.BATTERY_CRITICAL)
        self.feed(battery.BATTERY_INVALID, valid=0)
        self.assertIn("不可用", self.messages[-1])
        self.assertIn("尚未确认解除", self.messages[-1])
        self.assertFalse(any("低压解除" in item for item in self.messages))
        self.feed(battery.BATTERY_WARN)
        self.assertIn("但仍处于低压预警", self.messages[-1])
        self.assertIn("监测已恢复", self.messages[-1])
        self.feed()
        self.assertIn("新的有效样本", self.messages[-1])

    def test_init_never_displays_normal_and_grace_is_finite(self):
        for _ in range(8):
            self.feed(battery.BATTERY_INIT, valid=1)
            self.clock.advance(1)
        self.assertEqual(self.messages, [])
        self.monitor.poll()
        self.assertIn("判定中", self.messages[-1])
        self.assertNotIn("正常", self.messages[-1])
        self.feed()
        self.assertIn("监测恢复", self.messages[-1])

    def test_valid_bit_invalid_state_and_sample_age_are_separate_gates(self):
        for changes in ({"valid": 0}, {"state": battery.BATTERY_INVALID},
                        {"sample_age_ms": 501}):
            with self.subTest(changes=changes):
                messages = []
                monitor = battery.BatteryAlarmMonitor(messages.append, self.clock)
                monitor.ingest(battery.ugbat_parse(frame(**changes)))
                self.assertIn("不可用", messages[-1])
                self.assertNotIn("正常", messages[-1])
        self.feed(sample_age_ms=500)
        self.assertIn("正常", self.messages[-1])

    def test_missing_initial_telemetry_alerts_after_eight_seconds_and_is_limited(self):
        self.clock.advance(7.9)
        self.monitor.poll()
        self.assertEqual(self.messages, [])
        self.clock.advance(0.1)
        self.monitor.poll()
        self.assertIn("尚未收到", self.messages[-1])
        self.clock.advance(29)
        self.monitor.poll()
        self.assertEqual(len(self.messages), 1)
        self.clock.advance(1)
        self.monitor.poll()
        self.assertEqual(len(self.messages), 2)

    def test_missing_stream_marks_unknown_and_never_recovers_low_alarm(self):
        self.feed(battery.BATTERY_WARN)
        self.clock.advance(3)
        self.monitor.poll()
        self.assertEqual(len(self.messages), 1)
        self.clock.advance(0.01)
        self.monitor.poll()
        self.assertIn("超过 3 秒", self.messages[-1])
        self.assertIn("尚未确认解除", self.messages[-1])

    def test_duplicate_frames_do_not_extend_freshness(self):
        row = battery.ugbat_parse(frame())
        self.monitor.ingest(row)
        for _ in range(4):
            self.clock.advance(1)
            self.monitor.ingest(row)
        self.assertIn("超过 3 秒", self.messages[-1])

    def test_poll_can_repeat_low_alert_between_fresh_frames(self):
        self.feed(battery.BATTERY_CRITICAL)
        for _ in range(9):
            self.clock.advance(1)
            self.feed(battery.BATTERY_CRITICAL)
        self.clock.advance(1)
        self.monitor.poll()
        self.assertEqual(len(self.messages), 2)
        self.assertIn("严重低压", self.messages[-1])

    def test_disconnect_is_once_and_does_not_claim_battery_is_switched_off(self):
        self.feed(battery.BATTERY_WARN)
        self.monitor.disconnect()
        self.monitor.disconnect()
        self.clock.advance(100)
        self.monitor.poll()
        self.assertEqual(len(self.messages), 2)
        self.assertIn("无法区分断电和通信断开", self.messages[-1])
        self.assertIn("尚未确认解除", self.messages[-1])

    def test_default_console_output_has_newline_and_flush(self):
        monitor = battery.BatteryAlarmMonitor(clock=self.clock)
        with mock.patch("builtins.print") as output:
            monitor.ingest(battery.ugbat_parse(frame(state=battery.BATTERY_WARN)))
        self.assertTrue(output.call_args.args[0].startswith("\n[电池低压预警]"))
        self.assertEqual(output.call_args.kwargs, {"flush": True})


class BatteryReceiverTests(unittest.TestCase):
    def run_receiver(self, events, fail_battery_log=None):
        clock = FakeClock()
        pending = iter(events)
        conn, closed = mock.Mock(), threading.Event()
        ekf_file, battery_file = mock.Mock(), mock.Mock()
        output = io.StringIO()

        def receive(_size):
            delay, item = next(pending)
            clock.advance(delay)
            if isinstance(item, Exception):
                raise item
            return item

        def open_log(path, *_args, **_kwargs):
            if "battery_" in path:
                if fail_battery_log == "open":
                    raise OSError("电池目录只读")
                return battery_file
            return ekf_file

        if fail_battery_log in ("write", "flush"):
            getattr(battery_file, fail_battery_log).side_effect = OSError("电池磁盘写满")
        conn.recv.side_effect = receive
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, "running", True))
            stack.enter_context(mock.patch.object(contcp_t.os, "makedirs"))
            stack.enter_context(mock.patch.object(contcp_t, "open", side_effect=open_log, create=True))
            stack.enter_context(mock.patch.object(battery.time, "monotonic", clock))
            stack.enter_context(contextlib.redirect_stdout(output))
            contcp_t.receive_thread(conn, link_closed=closed)
        self.assertTrue(closed.is_set())
        conn.close.assert_called()
        conn.sendall.assert_not_called()
        self.assertNotIn("接收错误", output.getvalue())
        self.assertNotIn("接收线程异常", output.getvalue())
        return output.getvalue(), battery_file

    def test_fragmented_and_coalesced_frames_alarm_without_console_input(self):
        wire = (frame(state=battery.BATTERY_WARN, filtered_mv=21800) +
                frame(seq=2, state=battery.BATTERY_CRITICAL, filtered_mv=20800)).encode("ascii")
        output, logfile = self.run_receiver([(0, wire[:7]), (0, wire[7:35]),
                                            (0, wire[35:]), (0, b"")])
        self.assertEqual(output.count("[电池低压预警]"), 1)
        self.assertEqual(output.count("[电池严重低压]"), 1)
        self.assertIn("[电池监测中断]", output)
        self.assertEqual(logfile.write.call_count, 3)
        self.assertIn("21800", logfile.write.call_args_list[1].args[0])

    def test_other_stream_keeps_arriving_but_missing_battery_still_alerts(self):
        events = [(1, b"$UGMAG,ignore\r\n") for _ in range(9)] + [(0, b"")]
        output, _ = self.run_receiver(events)
        self.assertEqual(output.count("[电池监测不可用]"), 1)
        self.assertIn("尚未收到电池遥测", output)

    def test_socket_timeout_also_checks_missing_telemetry(self):
        output, _ = self.run_receiver([(1, socket.timeout()) for _ in range(9)] + [(0, b"")])
        self.assertIn("尚未收到电池遥测", output)

    def test_bad_frames_and_nonascii_corruption_cannot_refresh_monitor(self):
        valid = frame().encode("ascii")
        corrupted = valid[:10] + b"\xff" + valid[10:]
        events = [(0, valid)] + [(1, corrupted) for _ in range(5)] + [(0, b"")]
        output, logfile = self.run_receiver(events)
        self.assertEqual(output.count("监测已就绪"), 1)
        self.assertIn("超过 3 秒", output)
        self.assertEqual(logfile.write.call_count, 2)

    def test_battery_log_failures_do_not_stop_alarm_upgrades_or_receive(self):
        for failure in ("open", "write", "flush"):
            with self.subTest(failure=failure):
                output, _ = self.run_receiver([
                    (0, frame(state=battery.BATTERY_WARN).encode("ascii")),
                    (1, frame(seq=2, state=battery.BATTERY_CRITICAL).encode("ascii")),
                    (0, b""),
                ], fail_battery_log=failure)
                self.assertIn("[电池低压预警]", output)
                self.assertIn("[电池严重低压]", output)
                self.assertEqual(output.count("[电池记录失败]"), 1)


class BatterySubscriptionTests(unittest.TestCase):
    def test_main_subscribes_battery_before_console_and_turns_all_streams_off_on_exit(self):
        conn, listener, service = mock.Mock(), mock.Mock(), mock.Mock()
        listener.accept.return_value = (conn, ("127.0.0.1", 8800))
        worker = mock.Mock(ident=None)

        def console(*_args):
            masks = [ugcp.parse_frame(call.args[0])[2][0]
                     for call in conn.sendall.call_args_list]
            self.assertEqual(masks, [ugcp.STREAM_EKF, ugcp.STREAM_EKF | ugcp.STREAM_BAT])
            self.assertEqual(contcp_t.get_stream_control_mask(), ugcp.STREAM_EKF | ugcp.STREAM_BAT)

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, "running", True))
            stack.enter_context(mock.patch.object(contcp_t, "TELEMETRY_STREAM_MASK", 0))
            stack.enter_context(mock.patch.object(contcp_t, "GamepadService", return_value=service))
            stack.enter_context(mock.patch.object(contcp_t.socket, "socket", return_value=listener))
            stack.enter_context(mock.patch.object(contcp_t.threading, "Thread", return_value=worker))
            stack.enter_context(mock.patch.object(contcp_t, "_run_console", side_effect=console))
            stack.enter_context(mock.patch.object(contcp_t.time, "sleep"))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            contcp_t.main()
        kind, _seq, payload = ugcp.parse_frame(conn.sendall.call_args.args[0])
        self.assertEqual((kind, payload), (ugcp.MSG_STREAM_CONTROL, b"\x00"))

    def test_console_stream_changes_preserve_battery_until_explicit_shutdown(self):
        conn = mock.Mock()
        with mock.patch.object(contcp_t, "TELEMETRY_STREAM_MASK", 0), \
                contextlib.redirect_stdout(io.StringIO()):
            contcp_t.send_stream_control(conn, ugcp.STREAM_EKF | ugcp.STREAM_WRM)
            contcp_t.send_telem_control(conn, False)
            contcp_t.send_telem_control(conn, True)
            contcp_t.send_stream_control(conn, 0)
            contcp_t.send_stream_control(conn, 0, keep_battery=False)
            self.assertEqual(contcp_t.get_stream_control_mask(), 0)
        masks = [ugcp.parse_frame(call.args[0])[2][0] for call in conn.sendall.call_args_list]
        self.assertEqual(masks, [0x19, 0x18, 0x19, 0x10, 0])

    def test_failed_send_does_not_update_local_mask(self):
        conn = mock.Mock()
        conn.sendall.side_effect = OSError("链路断开")
        with mock.patch.object(contcp_t, "TELEMETRY_STREAM_MASK", ugcp.STREAM_BAT), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(contcp_t.send_stream_control(conn, ugcp.STREAM_EKF))
            self.assertEqual(contcp_t.get_stream_control_mask(), ugcp.STREAM_BAT)

    def test_pid_enter_and_restore_keep_battery(self):
        conn = mock.Mock()
        with mock.patch.object(contcp_t, "TELEMETRY_STREAM_MASK", ugcp.STREAM_EKF), \
                mock.patch("builtins.input", return_value="back"), \
                contextlib.redirect_stdout(io.StringIO()):
            contcp_t.run_pid_mode(conn, contcp_t.UgctlStore())
        packets = [ugcp.parse_frame(call.args[0]) for call in conn.sendall.call_args_list]
        masks = [payload[0] for kind, _seq, payload in packets if kind == ugcp.MSG_STREAM_CONTROL]
        self.assertEqual(masks, [ugcp.STREAM_EKF | ugcp.STREAM_CTL | ugcp.STREAM_BAT,
                                 ugcp.STREAM_EKF | ugcp.STREAM_BAT])


if __name__ == "__main__":
    unittest.main()
