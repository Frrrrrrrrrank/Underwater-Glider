"""用本地设备连接验证唯一接收线程、HTTP 快照和第二终端的完整只读链路。"""

import contextlib
import http.client
import io
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import battery_telemetry as battery
import contcp_t
import monitor
from monitor_bridge import MonitorBridge
import ugcp_protocol as ugcp


def telemetry(topic, seq=1, **changes):
    """按实际字段表制作离线板端输入，仍由生产解析器校验和转换。"""
    fields = {
        "ugekf": contcp_t.EKF_FIELDS,
        "ugctl": contcp_t.UGCTL_FIELDS,
        "ugwrm": (contcp_t.UGWRM_FIELDS_V2 if changes.get("schema") == 2
                  else contcp_t.UGWRM_FIELDS_V3),
        "ugbat": battery.UGBAT_FIELDS,
        "ugrds": contcp_t.UGRDS_FIELDS,
    }[topic]
    defaults = {
        "ugekf": dict(status=4, fl=3, roll_deg=3.5, pitch_deg=-2.25,
                       yaw_deg=42.0, depth=1.875),
        "ugctl": dict(schema=2, mode=2, armed=1, output_valid=1, pid_valid=1,
                       target_depth=2.5, target_pitch_deg=-4.0, pitch_sp_deg=-3.0,
                       active_crc=0x12345678, thrust_pct=61.0),
        "ugwrm": dict(schema=3, pulse=-70332, stroke_percent=25.0,
                       span_valid=1, data_fresh=1, ref_kind=2, span_pulses=281321,
                       home_state=3, limit_bits=10, roll_percent=62.5,
                       roll_valid=1, roll_home_state=6, calib_state=2),
        "ugbat": dict(schema=1, adc_raw=21000, battery_mv=23500,
                       filtered_mv=23490, valid=1, state=1, sample_age_ms=10),
        "ugrds": dict(schema=2, has_cached_fix=1, signal_good=1,
                       local_latitude_deg_e7=220000001, local_longitude_deg_e7=1145000001),
    }[topic]
    row = dict.fromkeys(fields, 0)
    row.update(defaults, seq=seq, t_ms=seq * 1000)
    row.update(changes)
    body = topic.upper() + "," + ",".join(str(row[name]) for name in fields)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


class ManualClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class MonitorIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        directory = self.stack.enter_context(tempfile.TemporaryDirectory())
        for setting in ("EKF_LOG_DIR", "CONTROL_LOG_DIR", "BATTERY_LOG_DIR", "EXPERIMENT_LOG_DIR", "RDSS_LOG_DIR"):
            self.stack.enter_context(mock.patch.object(contcp_t, setting, directory))
        self.stack.enter_context(mock.patch.object(contcp_t, "running", True))
        self.output = self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.clock = ManualClock()
        self.bridge = MonitorBridge(port=0, clock=self.clock)
        self.bridge.start()
        self.bridge.set_link("connected", peer="offline-socketpair")
        self.addCleanup(self.bridge.stop)
        self.url = "http://{}:{}/snapshot".format(*self.bridge.address)
        self.receiver_socket, self.board_socket = socket.socketpair()
        self.ctl_store, self.wrm_store = contcp_t.UgctlStore(), contcp_t.UgwrmStore()
        self.closed = threading.Event()
        self.receiver = None
        self.addCleanup(self.close_receiver)

    def close_receiver(self):
        self.board_socket.close()
        if self.receiver is not None:
            self.receiver.join(timeout=2)
        self.receiver_socket.close()
        if self.receiver is not None:
            self.assertFalse(self.receiver.is_alive(), "离线接收线程必须完整退出")

    def start_receiver(self, bridge=None):
        self.receiver = threading.Thread(
            target=contcp_t.receive_thread,
            args=(self.receiver_socket, self.ctl_store, self.wrm_store,
                  self.closed, self.bridge if bridge is None else bridge),
            daemon=True,
        )
        self.receiver.start()

    def wait_for(self, predicate, message="接收线程未及时发布预期遥测"):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.005)
        self.fail(message + "\n" + self.output.getvalue())

    def wait_seq(self, topic, seq):
        def match():
            snapshot = self.bridge.snapshot()
            sample = snapshot["telemetry"].get(topic, {})
            return snapshot if sample.get("data", {}).get("seq") == seq else None
        return self.wait_for(match)

    def send_all(self, seq=1):
        self.board_socket.sendall(b"".join(
            telemetry(topic, seq) for topic in ("ugekf", "ugctl", "ugwrm", "ugbat")))
        self.wait_seq("ugbat", seq)

    def assert_no_board_commands(self):
        # 读取设备一侧的反向字节，避免只验证某个发送函数没有被调用。
        self.board_socket.settimeout(0.05)
        with self.assertRaises(socket.timeout):
            self.board_socket.recv(4096)

    def test_fragmented_five_topics_reach_http_and_panel_without_board_writes(self):
        self.start_receiver()
        wire = b"".join(telemetry(topic) for topic in
                        ("ugekf", "ugctl", "ugwrm", "ugbat", "ugrds"))
        for begin in range(0, len(wire), 13):
            self.board_socket.sendall(wire[begin:begin + 13])
        self.wait_seq("ugrds", 1)
        snapshot = monitor.fetch_snapshot(self.url)
        self.assertEqual(set(snapshot["telemetry"]), {"ugekf", "ugctl", "ugwrm", "ugbat", "ugrds"})
        self.assertTrue(all(item["fresh"] for item in snapshot["telemetry"].values()))
        panel = monitor.render_panel(snapshot)
        for text in ("25.00%", "62.50%", "+3.50°", "-2.25°", "+42.00°",
                     "1.875 m", "23.490 V", "2.500 m", "-4.00°", "0x12345678",
                     "PB3 roll 0%：触发", "PB4 pitch头：释放",
                     "PB5 roll100%：触发", "PD7 pitch尾：释放",
                     "PID 整套参数：当前固件未上报", "左右实际 PWM：当前固件未上报",
                     "缓存定位（非实时）", "+22.0000001°", "+114.5000001°"):
            self.assertIn(text, panel)
        battery_lines = "\n".join(line for line in panel.splitlines()
                                  if "电池 " in line or "有效：" in line and "PID" not in line)
        self.assertNotIn("状态：", battery_lines)
        self.assertNotIn("SOC", panel)
        self.assertNotIn("61.00%", panel)
        self.assertEqual(self.ctl_store.snapshot()[1]["seq"], 1)
        self.assertEqual(self.wrm_store.snapshot()[1]["seq"], 1)
        self.assert_no_board_commands()

    def test_each_bad_checksum_is_dropped_before_snapshot_publication(self):
        self.start_receiver()
        self.send_all()
        for topic in ("ugekf", "ugctl", "ugwrm", "ugbat"):
            with self.subTest(topic=topic):
                bad = bytearray(telemetry(topic, seq=10))
                bad[-4] = ord("0") if bad[-4] != ord("0") else ord("1")
                barrier = "ugctl" if topic != "ugctl" else "ugwrm"
                self.board_socket.sendall(bytes(bad) + telemetry(barrier, seq=20))
                self.wait_seq(barrier, 20)
                self.assertNotEqual(monitor.fetch_snapshot(self.url)["telemetry"][topic]["data"]["seq"], 10)
                # 下一轮用新的屏障序号，保证等待的是本轮已处理的有效行。
                self.board_socket.sendall(telemetry(barrier, seq=1))
                self.wait_seq(barrier, 1)
        self.assert_no_board_commands()

    def test_wrm2_missing_roll_remains_unknown_until_wrm3_arrives(self):
        self.start_receiver()
        self.board_socket.sendall(telemetry("ugwrm", schema=2))
        self.wait_seq("ugwrm", 1)
        snapshot = monitor.fetch_snapshot(self.url)
        self.assertIsNone(snapshot["telemetry"]["ugwrm"]["data"]["roll_percent"])
        panel = monitor.render_panel(snapshot)
        self.assertIn("Roll ：--（旧协议未上报）", panel)
        self.assertIn("25.00%", panel)
        self.board_socket.sendall(telemetry("ugwrm", seq=2))
        self.wait_seq("ugwrm", 2)
        self.assertIn("62.50%", monitor.render_panel(monitor.fetch_snapshot(self.url)))
        self.assert_no_board_commands()

    def test_running_calibration_hold_is_not_advertised_as_neutral_handoff(self):
        self.start_receiver()
        self.board_socket.sendall(telemetry("ugwrm", calib_state=1, calib_hold=1,
                                            home_state=1, roll_home_state=1))
        self.wait_seq("ugwrm", 1)
        panel = monitor.render_panel(monitor.fetch_snapshot(self.url))
        self.assertIn("校准中", panel)
        self.assertNotIn("等待中立", panel)
        self.board_socket.sendall(telemetry("ugwrm", seq=2, calib_state=2, calib_hold=1))
        self.wait_seq("ugwrm", 2)
        self.assertIn("等待中立", monitor.render_panel(monitor.fetch_snapshot(self.url)))

    def test_device_disconnect_retains_diagnostics_but_hides_current_values(self):
        self.start_receiver()
        self.send_all()
        self.board_socket.shutdown(socket.SHUT_WR)
        self.assertTrue(self.closed.wait(timeout=2))
        self.wait_for(lambda: self.bridge.snapshot()["device_link"]["status"] == "disconnected")
        snapshot = monitor.fetch_snapshot(self.url)
        self.assertEqual(snapshot["device_link"]["status"], "disconnected")
        self.assertFalse(any(item["fresh"] for item in snapshot["telemetry"].values()))
        self.assertEqual(snapshot["telemetry"]["ugwrm"]["data"]["roll_percent"], 62.5)
        panel = monitor.render_panel(snapshot)
        self.assertIn("板端未连接", panel)
        for old in ("62.50%", "25.00%", "23.490 V", "1.875 m"):
            self.assertNotIn(old, panel)

    def test_stopped_topic_expires_without_expiring_slower_battery(self):
        self.start_receiver()
        self.send_all()
        self.clock.now += 0.6
        self.board_socket.sendall(telemetry("ugctl", seq=2))
        self.wait_seq("ugctl", 2)
        snapshot = monitor.fetch_snapshot(self.url)
        self.assertFalse(snapshot["telemetry"]["ugekf"]["fresh"])
        self.assertFalse(snapshot["telemetry"]["ugwrm"]["fresh"])
        self.assertTrue(snapshot["telemetry"]["ugbat"]["fresh"])
        panel = monitor.render_panel(snapshot)
        self.assertIn("数据过期", panel)
        self.assertIn("23.490 V", panel)
        self.assertNotIn("62.50%", panel)
        self.assert_no_board_commands()

    def test_slow_http_reader_and_monitor_exit_do_not_stop_device_receive(self):
        self.start_receiver()
        self.send_all()
        with socket.create_connection(self.bridge.address, timeout=0.4) as slow:
            slow.sendall(b"GET /snapshot HTTP/1.1\r\nHost: localhost\r\n")
            self.board_socket.sendall(telemetry("ugwrm", seq=2, roll_percent=70.0))
            self.wait_seq("ugwrm", 2)
            self.assertIn("70.00%", monitor.render_panel(monitor.fetch_snapshot(self.url)))
        # 真实面板入口单次读取后退出，也没有关闭控制连接。
        self.assertEqual(monitor.main(["--port", str(self.bridge.address[1]), "--once"]), 0)
        self.board_socket.sendall(telemetry("ugwrm", seq=3, roll_percent=75.0))
        self.wait_seq("ugwrm", 3)
        self.assertTrue(self.receiver.is_alive())
        self.assertFalse(self.closed.is_set())
        self.assert_no_board_commands()

    def test_http_post_is_rejected_without_device_side_effect(self):
        self.start_receiver()
        self.send_all()
        connection = http.client.HTTPConnection(*self.bridge.address, timeout=0.4)
        self.addCleanup(connection.close)
        connection.request("POST", "/snapshot", body=b'{"roll_rpm":300}')
        response = connection.getresponse()
        self.assertEqual(response.status, 405)
        self.assertEqual(response.getheader("Allow"), "GET")
        response.read()
        self.assert_no_board_commands()

    def test_stopping_http_bridge_keeps_original_control_stores_receiving(self):
        self.start_receiver()
        self.send_all()
        self.bridge.stop()
        self.board_socket.sendall(telemetry("ugctl", seq=2) + telemetry("ugwrm", seq=2))
        self.wait_for(lambda: self.wrm_store.snapshot()[1].get("seq") == 2)
        self.assertEqual(self.ctl_store.snapshot()[1]["seq"], 2)
        self.assertTrue(self.receiver.is_alive())
        self.assertFalse(self.closed.is_set())
        self.assert_no_board_commands()

    def test_publish_failure_is_isolated_from_original_receive_and_csv(self):
        broken = mock.Mock()
        broken.publish.side_effect = RuntimeError("离线快照发布故障")
        self.start_receiver(bridge=broken)
        self.board_socket.sendall(telemetry("ugctl") + telemetry("ugwrm") +
                                  telemetry("ugctl", seq=2) + telemetry("ugwrm", seq=2))
        self.wait_for(lambda: (self.wrm_store.snapshot()[1] or {}).get("seq") == 2)
        self.assertEqual(self.ctl_store.snapshot()[1]["seq"], 2)
        broken.publish.assert_called_once()
        broken.set_link.assert_called_once_with("error", last_error="离线快照发布故障")
        self.assertEqual(self.output.getvalue().count("数据发布失败"), 1)
        self.assertTrue(self.receiver.is_alive())
        self.assert_no_board_commands()

    def run_console_main(self):
        """仅替换硬件监听和手柄采集，保留真实收包、控制台和 UGCP 发送。"""
        listener = mock.Mock()
        listener.accept.return_value = (self.receiver_socket, ("127.0.0.1", 12345))
        socket_proxy = SimpleNamespace(**{name: getattr(socket, name) for name in (
            "AF_INET", "SOCK_STREAM", "SOL_SOCKET", "SO_REUSEADDR", "SHUT_RDWR", "timeout")})
        socket_proxy.socket = mock.Mock(return_value=listener)
        with contextlib.ExitStack() as local:
            local.enter_context(mock.patch.object(contcp_t, '_start_experiment_recorder', return_value=None))
            local.enter_context(mock.patch.object(contcp_t, "socket", socket_proxy))
            local.enter_context(mock.patch.object(contcp_t, "GamepadService", return_value=mock.Mock()))
            entered = local.enter_context(mock.patch("builtins.input", return_value="quit"))
            local.enter_context(mock.patch.object(contcp_t, "TELEMETRY_STREAM_MASK", 0))
            contcp_t.main()
        entered.assert_called_once()
        listener.close.assert_called_once()
        return listener

    def test_startup_status_failure_closes_optional_bridge(self):
        broken = mock.Mock()
        broken.start.return_value = ("127.0.0.1", 12345)
        broken.set_link.side_effect = RuntimeError("离线等待状态故障")
        with mock.patch.object(contcp_t, "MonitorBridge", return_value=broken):
            self.assertIsNone(contcp_t._start_monitor_bridge())
        broken.start.assert_called_once()
        broken.set_link.assert_called_once_with("waiting")
        broken.stop.assert_called_once()
        self.assertIn("控制台继续运行", self.output.getvalue())

    def test_connected_status_failure_closes_bridge_but_preserves_console(self):
        broken = mock.Mock()
        broken.set_link.side_effect = RuntimeError("离线连接状态故障")
        with mock.patch.object(contcp_t, "_start_monitor_bridge", return_value=broken):
            self.run_console_main()
        broken.set_link.assert_called_once_with("connected", peer="127.0.0.1:12345")
        broken.stop.assert_called_once()
        broken.publish.assert_not_called()
        self.assertIn("连接状态发布失败", self.output.getvalue())
        self.assertIn("控制台继续运行", self.output.getvalue())

    def test_monitor_port_conflict_preserves_console_and_only_existing_ugcp_commands(self):
        # 已启动的真实 HTTP 服务占住临时端口；控制端设备连接仍由本地 socketpair 提供。
        with mock.patch.dict(os.environ, {"UG_MONITOR_PORT": str(self.bridge.address[1])}):
            self.run_console_main()
        self.assertIn("控制台继续运行", self.output.getvalue())
        self.assertTrue(self.bridge.snapshot()["monitor"]["read_only"])
        self.board_socket.settimeout(1)
        wire = bytearray()
        while True:
            chunk = self.board_socket.recv(4096)
            if not chunk:
                break
            wire.extend(chunk)
        frames = []
        while wire:
            self.assertGreaterEqual(len(wire), 14)
            size = 14 + struct.unpack_from("<H", wire, 8)[0]
            frames.append(ugcp.parse_frame(bytes(wire[:size])))
            del wire[:size]
        self.assertEqual([kind for kind, _seq, _payload in frames], [
            ugcp.MSG_STREAM_CONTROL,
            ugcp.MSG_STREAM_CONTROL, ugcp.MSG_STREAM_CONTROL,
            ugcp.MSG_CONTROL19_LATCHED, ugcp.MSG_CONTROL19_LATCHED,
            ugcp.MSG_STREAM_CONTROL,
        ])
        self.assertEqual([payload for kind, _seq, payload in frames
                          if kind == ugcp.MSG_STREAM_CONTROL], [b"\x0d", b"\x1d", b"\x3d", b"\x00"])
        self.assertEqual(frames[3][2], bytes(18) + b"\x01")
        self.assertEqual(frames[4][2], bytes(19))


if __name__ == "__main__":
    unittest.main()
