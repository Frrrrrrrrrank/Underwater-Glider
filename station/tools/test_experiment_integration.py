"""真实协议解析与实验日志集成；假 socket 不打开任何设备连接。"""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

import battery_telemetry
import contcp_t
from experiment_recorder import ExperimentRecorder
import ugcp_protocol as ugcp


def telemetry(topic, seq=1, **changes):
    """使用生产字段表，检查生产校验/解析路径而非直接注入解析结果。"""
    fields = {
        "ugwrm": contcp_t.UGWRM_FIELDS_V3,
        "ugrds": contcp_t.UGRDS_FIELDS,
        "ugctl": contcp_t.UGCTL_FIELDS,
        "ugekf": contcp_t.EKF_FIELDS,
        "ugbat": battery_telemetry.UGBAT_FIELDS,
    }[topic]
    row = dict.fromkeys(fields, 0)
    row.update(seq=seq, t_ms=1000 * seq)
    row.update({
        "ugwrm": dict(schema=3, pulse=120, data_fresh=1, span_valid=1,
                      stroke_percent=25.0, ref_kind=2, home_state=3,
                      roll_valid=1, roll_percent=50.0, roll_home_state=6, calib_state=2),
        "ugrds": dict(schema=2, signal_good=1, ready_to_send=1),
        "ugctl": dict(schema=2),
        "ugekf": dict(status=4, fl=3, depth=1.5),
        "ugbat": dict(schema=1, battery_mv=25000, filtered_mv=25000, valid=1, state=1),
    }[topic])
    row.update(changes)
    body = topic.upper() + "," + ",".join(str(row[name]) for name in fields)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return ("$%s*%02X\r\n" % (body, checksum)).encode("ascii")


class FakeSocket:
    def __init__(self, chunks=(), send_error=None, supported_mask=None):
        self.chunks = iter(chunks)
        self.send_error = send_error
        self.supported_mask = supported_mask
        self.stream_mask = 0
        self.frames = []
        self.stream_attempts = []
        self.partial = b""
        self.closed = False
        self.recv_count = 0

    def sendall(self, frame):
        self.send_started = time.monotonic_ns()
        self.frames.append(bytes(frame))
        if self.send_error is not None:
            self.partial = bytes(frame[:7])
            raise self.send_error
        msg_type, _, payload = ugcp.parse_frame(frame)
        if msg_type == ugcp.MSG_STREAM_CONTROL:
            mask = payload[0]
            self.stream_attempts.append(mask)
            # 模拟旧固件：未知位让整包被拒，TCP sendall 依然成功。
            if self.supported_mask is None or not mask & ~self.supported_mask:
                self.stream_mask = mask
        self.send_ended = time.monotonic_ns()

    def recv(self, size):
        self.recv_count += 1
        item = next(self.chunks, b"")
        if isinstance(item, BaseException):
            raise item
        return item

    def settimeout(self, timeout):
        self.timeout = timeout

    def close(self):
        self.closed = True

    def shutdown(self, how):
        pass


class FakeBridge:
    def __init__(self):
        self.rows = []
        self.links = []

    def publish(self, topic, row):
        self.rows.append((topic, row))

    def set_link(self, state, **kwargs):
        self.links.append(state)


class ExplodingRecorder:
    def record(self, event):
        raise OSError("模拟记录器故障")

    def snapshot(self):
        return {"status": "error", "last_error": "模拟记录器故障"}


class ExperimentIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        buildtmp = STATION_DIR.parents[1] / ".buildtmp"
        buildtmp.mkdir(exist_ok=True)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory(
            prefix="experiment_integration_", dir=buildtmp)))
        for name, value in (
            ("EXPERIMENT_RECORDER", None), ("EXPERIMENT_LAST_ERROR", None),
            ("EXPERIMENT_CONTEXT", threading.local()), ("EXPERIMENT_MANAGED", False),
            ("UGCP_TX_SEQUENCE", ugcp.SequenceCounter(65535)),
            ("TELEMETRY_STREAM_MASK", 0), ("running", True),
        ):
            self.stack.enter_context(mock.patch.object(contcp_t, name, value, create=True))
        for name in ("EKF_LOG_DIR", "CONTROL_LOG_DIR", "BATTERY_LOG_DIR",
                     "RDSS_LOG_DIR", "EXPERIMENT_LOG_DIR"):
            self.stack.enter_context(mock.patch.object(contcp_t, name, str(self.root / name)))
        self.output = self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def start_recorder(self):
        recorder = ExperimentRecorder(self.root / "experiments", rdss_root=self.root / "logs" / "rsdd")
        directory = Path(recorder.start(peer="fake-device"))
        self.addCleanup(recorder.stop)
        contcp_t.EXPERIMENT_RECORDER = recorder
        return recorder, directory

    @staticmethod
    def events(directory):
        return [json.loads(line) for line in
                (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()]

    def run_receiver(self, conn):
        ctl, wrm = contcp_t.UgctlStore(), contcp_t.UgwrmStore()
        closed, bridge = threading.Event(), FakeBridge()
        # 只拦截 contcp 内的旧同步 open；记录器使用 pathlib 在后台打开文件。
        with mock.patch.object(contcp_t, "open", side_effect=AssertionError("不得同步写旧日志"), create=True) as legacy:
            contcp_t.receive_thread(conn, ctl, wrm, closed, bridge)
            legacy.assert_not_called()
        self.assertTrue(closed.is_set())
        self.assertTrue(conn.closed)
        return ctl, wrm, bridge

    def test_actual_control_payload_sequence_and_transport_times_are_recorded(self):
        recorder, directory = self.start_recorder()
        contcp_t._experiment_context(source="auto_sti", phase="ACTIVE", run_id="run-1",
                                     input={"sample_time": 12.5})
        packet = contcp_t._build_default_auto_packet()
        packet.update(pitchmotor_enable=1, pitchmotor_speed_rpm=-100,
                      rollmotor_enable=1, rollmotor_speed_rpm=300,
                      thrustmotor1se_speed_rpm=23, thrustmotor2se_speed_rpm=17,
                      thrustmotor_direction2=1)
        conn = FakeSocket()
        self.assertTrue(contcp_t.send_auto_control_packet(conn, packet, label="AUTO_STI"))
        first_socket_begin, first_socket_end = conn.send_started, conn.send_ended
        self.assertTrue(contcp_t.send_auto_control_packet(conn, packet, label="AUTO_STI"))
        recorder.stop()
        events = self.events(directory)
        self.assertEqual([event["seq"] for event in events], [65535, 0])
        for event, frame in zip(events, conn.frames):
            msg_type, seq, payload = ugcp.parse_frame(frame)
            self.assertEqual((event["msg_type"], event["seq"]), (msg_type, seq))
            self.assertEqual(event["payload_hex"], payload.hex())
            self.assertEqual(event["fields"], packet)
            self.assertTrue(event["success"])
            self.assertEqual(event["run_id"], "run-1")
            self.assertEqual(event["input"]["sample_time"], 12.5)
        self.assertLessEqual(events[0]["tx_start_ns"], first_socket_begin)
        self.assertLessEqual(first_socket_end, events[0]["tx_end_ns"])
        self.assertEqual(events[0]["t_ns"], events[0]["tx_start_ns"])

    def test_partial_send_error_is_recorded_as_failure_without_claiming_ack(self):
        recorder, directory = self.start_recorder()
        conn = FakeSocket(send_error=OSError("connection failed after partial bytes"))
        self.assertFalse(contcp_t.send_auto_control_packet(conn, contcp_t._build_default_auto_packet()))
        recorder.stop()
        event = self.events(directory)[0]
        self.assertEqual(conn.partial, conn.frames[0][:7])
        self.assertFalse(event["success"])
        self.assertIn("partial", event["error"])
        self.assertNotEqual(event.get("delivery_status"), "not_received")
        self.assertNotEqual(event.get("acknowledged"), True)
        self.assertLessEqual(event["tx_start_ns"], event["tx_end_ns"])

    def test_keyboard_interrupt_during_send_is_not_logged_as_success(self):
        recorder, directory = self.start_recorder()
        conn = FakeSocket(send_error=KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            contcp_t.send_auto_control_packet(conn, contcp_t._build_default_auto_packet())
        recorder.stop()
        event = self.events(directory)[0]
        self.assertFalse(event["success"])
        self.assertTrue(event["error"])

    def test_neutral_and_release_control19_are_recorded_with_distinct_phases(self):
        recorder, directory = self.start_recorder()
        contcp_t._experiment_context(source="auto_sti", phase="EXIT", run_id="run-1")
        conn = FakeSocket()
        with mock.patch.object(contcp_t.time, "sleep"):
            self.assertTrue(contcp_t.send_neutral_and_release(conn))
        recorder.stop()
        events = self.events(directory)
        self.assertEqual([event["phase"] for event in events], ["NEUTRAL", "RELEASE"])
        self.assertEqual([event["fields"]["eth_control_flag"] for event in events], [1, 0])
        self.assertEqual([event["msg_type"] for event in events], [ugcp.MSG_CONTROL19_LATCHED] * 2)
        self.assertTrue(all(len(bytes.fromhex(event["payload_hex"])) == 19 for event in events))
        self.assertTrue(all(event["fields"]["pitchmotor_speed_rpm"] == 0 for event in events))
        self.assertEqual(contcp_t.EXPERIMENT_CONTEXT.value["phase"], "EXIT")

    def test_record_exception_cannot_change_success_or_prevent_release(self):
        contcp_t.EXPERIMENT_RECORDER = ExplodingRecorder()
        conn = FakeSocket()
        self.assertTrue(contcp_t.send_auto_control_packet(conn, contcp_t._build_default_auto_packet()))
        with mock.patch.object(contcp_t.time, "sleep"):
            self.assertTrue(contcp_t.send_neutral_and_release(conn))
        self.assertEqual(len(conn.frames), 3)
        self.assertIn("模拟记录器故障", contcp_t.EXPERIMENT_LAST_ERROR)
        _, _, release = ugcp.parse_frame(conn.frames[-1])
        self.assertEqual(release[-1], 0)

    def test_fragmented_rx_common_batch_time_bad_checksum_and_rsdd_tail(self):
        recorder, directory = self.start_recorder()
        wrm = telemetry("ugwrm")
        good = telemetry("ugrds", seq=2)
        bad = bytearray(telemetry("ugrds", seq=3))
        bad[-4] = ord("0") if bad[-4] != ord("0") else ord("1")
        tail = b"$UGRDS,1,99,123"
        conn = FakeSocket([wrm[:17], wrm[17:] + good + bytes(bad) + tail, b""])
        _, wrm_store, bridge = self.run_receiver(conn)
        self.assertEqual(wrm_store.snapshot()[1]["pulse"], 120)
        self.assertEqual([row["seq"] for topic, row in bridge.rows if topic == "ugrds"], [2])
        recorder.stop()
        rx = [event for event in self.events(directory) if event["kind"] == "rx"]
        self.assertEqual(len(rx), 4)
        self.assertEqual([event["line_index"] for event in rx[:3]], [0, 1, 2])
        self.assertEqual(len({event["t_ns"] for event in rx[:3]}), 1)
        self.assertEqual(len({event["host_iso"] for event in rx[:3]}), 1)
        self.assertEqual(len({event["batch_id"] for event in rx[:3]}), 1)
        self.assertEqual(rx[0]["raw"], wrm.decode("ascii"))
        self.assertEqual(rx[1]["row"]["seq"], 2)
        self.assertIsNone(rx[2]["row"])
        self.assertEqual(rx[2]["raw"], bytes(bad).decode("ascii"))
        self.assertEqual(rx[3]["topic"], "ugrds")
        self.assertFalse(rx[3]["complete"])
        self.assertEqual(rx[3]["raw"], tail.decode("ascii"))
        self.assertEqual(rx[3]["t_ns"], rx[2]["t_ns"])
        self.assertEqual(rx[3]["batch_id"], rx[2]["batch_id"])
        rsdd = [json.loads(line) for line in
                Path(recorder.snapshot()["rdss_log_path"]).read_text(encoding="utf-8").splitlines()]
        self.assertEqual([event["raw"] for event in rsdd],
                         [good.decode("ascii"), bytes(bad).decode("ascii"), tail.decode("ascii")])
        self.assertEqual([event["complete"] for event in rsdd], [True, True, False])

    def test_all_telemetry_uses_background_recorder_without_legacy_open(self):
        recorder, directory = self.start_recorder()
        topics = ("ugekf", "ugctl", "ugwrm", "ugbat", "ugrds")
        conn = FakeSocket([b"".join(telemetry(topic) for topic in topics)])
        ctl, wrm, bridge = self.run_receiver(conn)
        self.assertEqual(ctl.snapshot()[1]["schema"], 2)
        self.assertEqual(wrm.snapshot()[1]["schema"], 3)
        self.assertEqual({topic for topic, row in bridge.rows}, set(topics))
        recorder.stop()
        self.assertEqual({event["topic"] for event in self.events(directory) if event["kind"] == "rx"}, set(topics))

    def test_startup_recording_failure_does_not_fall_back_to_sync_files_or_break_receive(self):
        contcp_t.EXPERIMENT_MANAGED = True
        contcp_t.EXPERIMENT_LAST_ERROR = "模拟日志目录不可写"
        topics = ("ugekf", "ugctl", "ugwrm", "ugbat", "ugrds")
        conn = FakeSocket([b"".join(telemetry(topic) for topic in topics)])
        ctl, wrm, bridge = self.run_receiver(conn)
        self.assertEqual({topic for topic, row in bridge.rows}, set(topics))
        self.assertEqual(ctl.snapshot()[1]["seq"], 1)
        self.assertEqual(wrm.snapshot()[1]["seq"], 1)
        self.assertGreaterEqual(conn.recv_count, 2)
        self.assertIn("模拟日志目录不可写", self.output.getvalue())

    def test_recorder_exception_during_receive_keeps_telemetry_alive(self):
        contcp_t.EXPERIMENT_RECORDER = ExplodingRecorder()
        conn = FakeSocket([telemetry("ugwrm", seq=1), telemetry("ugwrm", seq=2)])
        _, wrm, _ = self.run_receiver(conn)
        self.assertEqual(wrm.snapshot()[1]["seq"], 2)
        self.assertIn("模拟记录器故障", contcp_t.EXPERIMENT_LAST_ERROR)

    def test_overlong_ugrds_fragment_is_saved_and_next_valid_frame_recovers(self):
        recorder, directory = self.start_recorder()
        long_fragment = b"$UGRDS," + b"9" * 65536
        chunks = [long_fragment[start:start + 4096] for start in range(0, len(long_fragment), 4096)]
        conn = FakeSocket(chunks + [telemetry("ugrds", seq=7)])
        _, _, bridge = self.run_receiver(conn)
        recorder.stop()
        rx = [event for event in self.events(directory) if event["kind"] == "rx"]
        self.assertEqual(len(rx), 2)
        self.assertFalse(rx[0]["complete"])
        self.assertEqual(rx[0]["error"], "line_too_long")
        self.assertEqual(rx[0]["raw"], long_fragment.decode("ascii"))
        self.assertEqual(rx[1]["row"]["seq"], 7)
        self.assertEqual([row["seq"] for topic, row in bridge.rows if topic == "ugrds"], [7])
        independent = [json.loads(line) for line in
                       Path(recorder.snapshot()["rdss_log_path"]).read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(independent), 2)
        self.assertEqual(independent[0]["raw"], long_fragment.decode("ascii"))

    def test_stream_fallbacks_work_for_three_firmware_generations_on_every_change(self):
        for supported in (0x0F, 0x1F, 0x3F):
            # 本轮新增 RDSS 路径始终从 0x3D 继承主题；BAT-only 保持旧行为。
            requested_masks = (0x0D, 0x3D) if supported == 0x0F else (0x0D, 0x1D, 0x3D)
            for requested in requested_masks:
                with self.subTest(supported=supported, requested=requested):
                    conn = FakeSocket(supported_mask=supported)
                    # 开启、关EKF、关WRM、恢复全主题、最终全关。
                    for desired in (requested, requested & ~ugcp.STREAM_EKF,
                                    requested & ~ugcp.STREAM_WRM, requested, 0):
                        self.assertTrue(contcp_t.send_stream_control(conn, desired, keep_battery=False))
                        self.assertEqual(conn.stream_mask, desired & supported)
                        self.assertEqual(contcp_t.get_stream_control_mask(), desired)
        conn = FakeSocket(supported_mask=0x3F)
        self.assertTrue(contcp_t.send_stream_control(conn, 0x3D))
        self.assertEqual(conn.stream_attempts, [0x0D, 0x1D, 0x3D])

    def test_main_startup_failure_continues_and_cleans_up_without_real_threads(self):
        conn = FakeSocket()
        server = mock.Mock()
        server.accept.return_value = (conn, ("192.0.2.10", 12345))
        receiver = mock.Mock(ident=None)
        receiver.is_alive.return_value = False
        receiver.start.side_effect = lambda: None
        observed = []
        def console(*args):
            observed.append((contcp_t.EXPERIMENT_MANAGED, contcp_t.EXPERIMENT_RECORDER))
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, "_start_monitor_bridge", return_value=None))
            stack.enter_context(mock.patch.object(contcp_t, "_start_experiment_recorder", return_value=None))
            stack.enter_context(mock.patch.object(contcp_t, "GamepadService", side_effect=OSError("无手柄")))
            stack.enter_context(mock.patch.object(contcp_t.socket, "socket", return_value=server))
            stack.enter_context(mock.patch.object(contcp_t.threading, "Thread", return_value=receiver))
            stack.enter_context(mock.patch.object(contcp_t, "_run_console", side_effect=console))
            stack.enter_context(mock.patch.object(contcp_t.time, "sleep"))
            contcp_t.main()
        self.assertEqual(observed, [(True, None)])
        self.assertTrue(conn.closed)
        server.close.assert_called_once()
        receiver.start.assert_called_once()
        frames = [ugcp.parse_frame(frame) for frame in conn.frames]
        self.assertEqual([payload[-1] for kind, _, payload in frames
                          if kind == ugcp.MSG_CONTROL19_LATCHED], [1, 0])

    def test_main_receiver_join_timeout_marks_recording_incomplete_before_stop(self):
        recorder, directory = self.start_recorder()
        conn = FakeSocket()
        server = mock.Mock()
        server.accept.return_value = (conn, ("192.0.2.10", 12345))
        receiver = mock.Mock(ident=1234)
        receiver.is_alive.return_value = True
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(contcp_t, "_start_monitor_bridge", return_value=None))
            stack.enter_context(mock.patch.object(contcp_t, "_start_experiment_recorder", return_value=recorder))
            stack.enter_context(mock.patch.object(contcp_t, "GamepadService", side_effect=OSError("无手柄")))
            stack.enter_context(mock.patch.object(contcp_t.socket, "socket", return_value=server))
            stack.enter_context(mock.patch.object(contcp_t.threading, "Thread", return_value=receiver))
            stack.enter_context(mock.patch.object(contcp_t, "_run_console", return_value=None))
            stack.enter_context(mock.patch.object(contcp_t.time, "sleep"))
            contcp_t.main()
        receiver.join.assert_called_once_with(timeout=2)
        summary = json.loads((directory / "session.json").read_text(encoding="utf-8"))
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["status"], "error")
        self.assertIn("接收线程", summary["last_error"])
        self.assertTrue(any(event.get("status") == "closed" for event in self.events(directory)))


if __name__ == "__main__":
    unittest.main()
