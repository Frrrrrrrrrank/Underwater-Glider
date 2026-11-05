"""只读面板的字段语义、失效隐藏、终端恢复与网络访问边界。"""

import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import unittest
from unittest import mock


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import monitor


def snapshot():
    data = {
        "ugekf": dict(fl=2, pitch_deg=12.34, roll_deg=-4.56, yaw_deg=87.65, depth=3.210),
        "ugctl": dict(mode=2, armed=1, output_valid=1, phase=0, fault=0, core_fault=0,
                      target_depth=10.2, target_pitch_deg=-8.3, pitch_sp_deg=-7.4,
                      pid_valid=1, active_crc=0x1234ABCD, thrust_pct=99.9,
                      pitch_deg=999, depth=999),
        "ugwrm": dict(schema=3, stroke_percent=25.5, ref_kind=2, data_fresh=1,
                      span_valid=1, roll_percent=67.8, roll_valid=1, calib_state=2,
                      calib_hold=1, home_state=3, roll_home_state=6, calib_fail=0,
                      limit_bits=10),
        "ugbat": dict(filtered_mv=27234, valid=1, state=3, sample_age_ms=120),
    }
    return {
        "schema_version": 1,
        "device_link": {"status": "connected", "session_id": "test-session"},
        "telemetry": {
            key: {"data": value, "age_ms": 123, "fresh": True,
                  "invalid_fields": {}, "quality": {"status": "valid"},
                  "session_id": "test-session"}
            for key, value in data.items()
        },
    }


def row(panel, prefix):
    return next(line for line in panel.splitlines() if line.startswith(prefix))


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = snapshot()

    def render(self):
        return monitor.render_panel(self.snapshot, width=120)

    def data(self, topic):
        return self.snapshot["telemetry"][topic]["data"]

    def test_fresh_values_use_ekf_and_do_not_invent_pwm_or_parameter_readback(self):
        panel = self.render()
        for text in ("25.50%（校准行程）", "67.80%（脉冲估算）", "+12.34°",
                     "-4.56°", "+87.65°", "3.210 m", "27.234 V", "10.200 m",
                     "-8.30°", "-7.40°", "0x1234ABCD", "PB3 roll 0%：触发",
                     "PB4 pitch头：释放", "PB5 roll100%：触发", "PD7 pitch尾：释放"):
            self.assertIn(text, panel)
        self.assertIn("PID 整套参数：当前固件未上报", panel)
        self.assertIn("左右实际 PWM：当前固件未上报", panel)
        self.assertNotIn("999", panel)
        self.assertNotIn("99.9", panel)
        self.assertNotIn("SOC", panel)

    def test_pitch_observed_range_is_explicit_and_reference_zero_is_hidden(self):
        self.data("ugwrm")["ref_kind"] = 1
        self.assertIn("25.50%（观测范围/未校准）", self.render())
        self.data("ugwrm")["ref_kind"] = 0
        self.assertNotIn("25.50%", self.render())
        self.assertIn("无有效参考", self.render())

    def test_roll_remains_available_when_pitch_feedback_is_invalid(self):
        self.data("ugwrm")["data_fresh"] = 0
        panel = self.render()
        self.assertNotIn("25.50%", panel)
        self.assertIn("67.80%", panel)
        self.data("ugwrm")["roll_valid"] = 0
        self.assertIn("Roll ：--（未标定）", self.render())

    def test_old_protocol_reason_survives_bridge_invalid_field(self):
        self.data("ugwrm").update(schema=2, roll_percent=None, roll_valid=None)
        self.snapshot["telemetry"]["ugwrm"]["invalid_fields"]["roll_percent"] = "roll_reference_unknown"
        panel = self.render()
        self.assertIn("Roll ：--（旧协议未上报）", panel)
        self.assertIn("25.50%", panel)
        self.assertIn("校准：--（旧协议未上报）", panel)

    def test_calibration_hold_only_waits_for_neutral_after_running(self):
        self.data("ugwrm")["calib_state"] = 1
        self.assertIn("保持：校准接管", self.render())
        self.assertNotIn("等待中立", self.render())
        self.data("ugwrm")["calib_state"] = 2
        self.assertIn("保持：等待中立", self.render())

    def test_disconnect_and_http_error_hide_all_previous_values(self):
        self.snapshot["device_link"]["status"] = "disconnected"
        for panel in (self.render(), monitor.render_panel(self.snapshot, error="拒绝连接")):
            for old in ("25.50%", "67.80%", "27.234 V", "+12.34°", "0x1234ABCD"):
                self.assertNotIn(old, panel)
        self.assertIn("板端未连接", self.render())
        self.assertIn("接口不可用", monitor.render_panel(None, error="拒绝连接"))

    def test_stale_or_previous_session_topic_does_not_hide_other_topics(self):
        self.snapshot["telemetry"]["ugekf"]["fresh"] = False
        self.assertNotIn("+12.34°", self.render())
        self.assertIn("数据过期", self.render())
        self.assertIn("27.234 V", self.render())
        self.snapshot["telemetry"]["ugwrm"]["session_id"] = "older-session"
        self.assertNotIn("67.80%", self.render())
        self.assertIn("旧连接数据", self.render())

    def test_invalid_fields_nonfinite_and_missing_values_are_not_numbers(self):
        self.snapshot["telemetry"]["ugekf"]["invalid_fields"]["pitch_deg"] = "ekf_not_valid"
        self.data("ugekf").update(roll_deg=float("nan"), yaw_deg=None, depth=10 ** 400)
        panel = self.render()
        self.assertNotIn("+12.34°", panel)
        self.assertNotIn("nan", panel)
        self.assertIn("--（数值无效）", panel)
        self.assertIn("--（未上报）", panel)

    def test_battery_only_shows_voltage_and_combined_validity(self):
        line = row(self.render(), "电池 ")
        self.assertEqual(line, "电池  滤波电压：27.234 V  有效：是")
        for changes in ({"sample_age_ms": 501}, {"sample_age_ms": None},
                        {"valid": 0}, {"filtered_mv": None}):
            with self.subTest(changes=changes):
                self.data("ugbat").update(filtered_mv=27234, valid=1, sample_age_ms=120)
                self.data("ugbat").update(changes)
                line = row(self.render(), "电池 ")
                self.assertNotIn("27.234 V", line)
                self.assertIn("有效：否", line)
        self.data("ugbat").update(filtered_mv=27234, valid=1, sample_age_ms=120)
        self.snapshot["telemetry"]["ugbat"]["invalid_fields"]["filtered_mv"] = "battery_invalid"
        self.assertIn("有效：否", row(self.render(), "电池 "))

    def test_missing_battery_is_explicit(self):
        del self.snapshot["telemetry"]["ugbat"]
        self.assertEqual(row(self.render(), "电池 "), "电池  滤波电压：--（未收到）  有效：--（未收到）")

    def test_pid_off_and_pitch_hold_do_not_advertise_a_depth_phase(self):
        self.data("ugctl").update(mode=0, armed=0)
        panel = self.render()
        self.assertIn("阶段：--（未运行）", panel)
        self.assertNotIn("下潜", panel)
        self.assertNotIn("-7.40°", panel)
        self.data("ugctl").update(mode=1, armed=1)
        self.assertNotIn("下潜", self.render())
        self.assertIn("-7.40°", self.render())
        self.data("ugctl")["output_valid"] = 0
        self.assertNotIn("-7.40°", self.render())
        self.assertIn("算法输出无效", self.render())

    def add_rdss(self):
        self.snapshot["telemetry"]["ugrds"] = {
            "data": dict(has_cached_fix=1, target_valid=1, signal_good=1, ready_to_send=1,
                         pending=0, waiting_ack=0, last_result=3, sent_count=15, send_ok_count=13,
                         attempt_count=20, reject_count=2, timeout_count=3, uart_fail_count=0,
                         card_id_known=1, card_mismatch=0, goto_received_count=8,
                         local_latitude_deg_e7=221234567, local_longitude_deg_e7=1147654321,
                         target_latitude_deg_e7=-224567890, target_longitude_deg_e7=-1141234567),
            "age_ms": 900, "fresh": True, "invalid_fields": {},
            "quality": {"status": "valid"}, "session_id": "test-session",
        }

    def test_rdss_reports_cached_coordinates_and_module_acknowledgement(self):
        self.add_rdss()
        panel = self.render()
        for text in ("缓存定位（非实时）", "+22.1234567°", "+114.7654321°",
                     "-22.4567890°", "-114.1234567°", "结果：模块确认",
                     "模块确认：15", "SEND OK：13", "目标接收：8", "RDSS：900ms/有效",
                     "北斗模块确认不代表对端送达"):
            self.assertIn(text, panel)
        self.data("ugrds")["has_cached_fix"] = 0
        panel = self.render()
        self.assertNotIn("+22.1234567°", panel)
        self.assertIn("无缓存定位", panel)
        self.assertIn("-22.4567890°", panel)
        self.data("ugrds")["target_valid"] = 0
        self.assertNotIn("-22.4567890°", self.render())

    def test_rdss_unknown_stale_disconnected_and_previous_session_hide_coordinates(self):
        self.assertIn("北斗  信号：--（未收到）", self.render())
        for field, value in (("fresh", False), ("session_id", "old")):
            self.add_rdss()
            self.snapshot["telemetry"]["ugrds"][field] = value
            self.assertNotIn("+22.1234567°", self.render())
        self.add_rdss()
        self.snapshot["device_link"]["status"] = "disconnected"
        self.assertNotIn("+22.1234567°", self.render())

    def test_experiment_status_is_optional_and_visible_near_top(self):
        self.assertNotIn("实验记录：", self.render())
        self.snapshot["experiment"] = dict(status="recording", directory="logs/session",
                                            dropped_records=7)
        panel = self.render()
        self.assertEqual(panel.splitlines()[3], "实验记录：记录中  丢记录：7")
        self.assertIn("记录目录：logs/session", panel)
        self.snapshot["experiment"].update(status="error", last_error="disk full")
        self.assertIn("记录错误：disk full", self.render())
        self.assertIn("实验记录：接口不可用", monitor.render_panel(self.snapshot, error="断连"))

    def test_render_is_pure_width_bounded_and_sanitizes_control_characters(self):
        before = copy.deepcopy(self.snapshot)
        for width in (1, 20, 60, 100):
            panel = monitor.render_panel(self.snapshot, width=width, endpoint="\x1b[2J恶意\n地址")
            self.assertEqual(len(panel.splitlines()), 32)
            self.assertNotIn("\x1b", panel)
            self.assertTrue(all(monitor.display_width(line) <= width for line in panel.splitlines()))
        self.assertEqual(self.snapshot, before)


class TerminalAndFetchTests(unittest.TestCase):
    def test_unchanged_frame_is_not_rewritten_and_resize_invalidates_cache(self):
        stream = io.StringIO()
        terminal = monitor._TerminalPanel(stream)
        terminal.draw("第一行\n第二行\n第三行", 4, 40)
        stream.seek(0)
        stream.truncate()
        terminal.draw("第一行\n第二行\n第三行", 4, 40)
        self.assertEqual(stream.getvalue(), "")
        terminal.draw("第一行\n第二行\n第三行", 2, 20)
        self.assertTrue(stream.getvalue().startswith("\x1b[H\x1b[J"))
        self.assertNotIn("\x1b[3;1H", stream.getvalue())
        self.assertEqual(len(terminal.previous), 2)
        self.assertLessEqual(monitor.display_width(terminal.previous[-1]), 20)

    def test_context_restores_cursor_on_interrupt(self):
        stream = io.StringIO()
        with mock.patch.object(monitor.os, "name", "posix"):
            with self.assertRaises(KeyboardInterrupt):
                with monitor._TerminalPanel(stream):
                    raise KeyboardInterrupt
        self.assertTrue(stream.getvalue().endswith("\x1b[?25h\x1b[?1049l"))

    def test_entry_write_failure_restores_console_mode(self):
        # 预置模式夹具，验证 enter 失败也走恢复；不改变宿主真实控制台。
        stream = mock.Mock()
        stream.write.side_effect = [OSError("输出失败"), None]
        kernel = mock.Mock()
        terminal = monitor._TerminalPanel(stream)
        terminal.console_mode = kernel, 123, 7
        with mock.patch.object(monitor.os, "name", "posix"):
            with self.assertRaisesRegex(OSError, "输出失败"):
                terminal.__enter__()
        kernel.SetConsoleMode.assert_called_once_with(123, 7)
        self.assertEqual(stream.write.call_args_list[-1].args[0], "\x1b[?25h\x1b[?1049l")

    def test_fetch_is_get_only_with_proxy_disabled(self):
        response = mock.MagicMock()
        response.__enter__.return_value.read.return_value = json.dumps(snapshot()).encode("utf-8")
        opener = mock.Mock()
        opener.open.return_value = response
        with mock.patch.object(monitor, "build_opener", return_value=opener) as factory:
            result = monitor.fetch_snapshot()
        self.assertEqual(result, snapshot())
        self.assertEqual(factory.call_args.args[0].proxies, {})
        request = opener.open.call_args.args[0]
        self.assertEqual(request.get_method(), "GET")
        self.assertIsNone(request.data)
        self.assertEqual(request.full_url, monitor.DEFAULT_URL)

    def test_fetch_rejects_remote_urls_redirects_and_incompatible_body(self):
        for url in ("http://192.168.57.100:8800/snapshot", "https://127.0.0.1/snapshot",
                    "http://127.0.0.1/control", "http://127.0.0.1/snapshot?a=1"):
            with self.subTest(url=url), mock.patch.object(monitor, "build_opener") as factory:
                with self.assertRaises(ValueError):
                    monitor.fetch_snapshot(url)
                factory.assert_not_called()
        with self.assertRaises(ValueError):
            monitor._NoRedirect().redirect_request(None, None, 302, "", {}, "http://example.com")
        response = mock.MagicMock()
        opener = mock.Mock()
        opener.open.return_value = response
        for body in (b"not-json", b"{}", b"x" * (monitor.MAX_SNAPSHOT_BYTES + 1)):
            with self.subTest(body=body[:20]):
                response.__enter__.return_value.read.return_value = body
                with mock.patch.object(monitor, "build_opener", return_value=opener):
                    with self.assertRaises(ValueError):
                        monitor.fetch_snapshot()

    def test_once_success_and_failure_are_plain_text_with_distinct_exit_status(self):
        for result, expected in ((snapshot(), 0), (OSError("桥未启动"), 1)):
            output = io.StringIO()
            fetch = mock.Mock(side_effect=result if isinstance(result, Exception) else None,
                              return_value=result)
            with contextlib.redirect_stdout(output), mock.patch.object(monitor, "fetch_snapshot", fetch):
                self.assertEqual(monitor.main(["--once", "--port", "9123"]), expected)
            fetch.assert_called_once_with("http://127.0.0.1:9123/snapshot")
            self.assertNotIn("\x1b", output.getvalue())
            if expected:
                self.assertIn("接口不可用", output.getvalue())
                self.assertNotIn("27.234 V", output.getvalue())


if __name__ == "__main__":
    unittest.main()
