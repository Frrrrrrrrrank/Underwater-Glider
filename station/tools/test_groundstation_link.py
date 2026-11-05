#!/usr/bin/env python3
"""URGC 旧遥测适配器和 Ethernet 唯一接收者回归测试。"""

import csv
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from groundstation.ethernet_link import (  # noqa: E402
    DEFAULT_STREAM_MASK,
    EthernetRealtimeLink,
)
from groundstation.legacy_telemetry import (  # noqa: E402
    EKF_FIELDS,
    EKF_INT_FIELDS,
    UGCTL_FIELDS,
    UGCTL_INT_FIELDS,
    UGWRM_FIELDS_V2 as UGWRM_FIELDS,
    UGWRM_INT_FIELDS,
    UGRDS_FIELDS,
    UGRDS_INT_FIELDS,
    decode_telemetry_line,
)
from groundstation.state_store import StateStore  # noqa: E402
from groundstation.session_recorder import SessionRecorder  # noqa: E402
from ugcp_protocol import (  # noqa: E402
    MSG_STREAM_CONTROL,
    STREAM_EKF,
    STREAM_RDSS,
    STREAM_WRM,
    build_frame,
    parse_frame,
)


def make_telemetry_line(prefix, fields, integer_fields, overrides=None):
    """按现有板端格式生成一行带 XOR 校验的测试遥测。"""

    values = []
    overrides = overrides or {}
    for index, name in enumerate(fields):
        if name in overrides:
            value = overrides[name]
        elif name == "schema":
            value = 2
        elif prefix == "UGRDS":
            # RDSS 标志与枚举不能用字段序号代替合法值。
            value = 0
        elif name in integer_fields:
            value = index + 1
        else:
            value = index + 0.25
        values.append(str(value))

    body = prefix + "," + ",".join(values)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "$" + body + "*{:02X}".format(checksum)


class LegacyTelemetryAdapterTests(unittest.TestCase):
    def test_decodes_the_four_telemetry_topics(self):
        cases = (
            ("UGEKF", EKF_FIELDS, EKF_INT_FIELDS,
             {"t_ms": 123, "status": 4, "fl": 3}, "ugekf"),
            ("UGCTL", UGCTL_FIELDS, UGCTL_INT_FIELDS,
             {"schema": 2, "t_ms": 234, "armed": 0}, "ugctl"),
            ("UGWRM", UGWRM_FIELDS, UGWRM_INT_FIELDS,
             {"schema": 2, "t_ms": 345}, "ugwrm"),
            ("UGRDS", UGRDS_FIELDS, UGRDS_INT_FIELDS,
             {"schema": 2, "t_ms": 456, "last_result": 3,
              "has_cached_fix": 1,
              "local_latitude_deg_e7": 220000000,
              "local_longitude_deg_e7": 1145000000,
              "target_valid": 1, "goto_received_count": 2,
              "target_latitude_deg_e7": 220000000,
              "target_longitude_deg_e7": 1145000000}, "ugrds"),
        )

        for prefix, fields, integer_fields, overrides, expected_topic in cases:
            with self.subTest(prefix=prefix):
                line = make_telemetry_line(prefix, fields, integer_fields, overrides)
                decoded = decode_telemetry_line(line)
                self.assertIsNotNone(decoded)
                self.assertEqual(decoded.topic, expected_topic)
                self.assertEqual(decoded.device_tick_ms, overrides["t_ms"])
                self.assertEqual(decoded.data["t_ms"], overrides["t_ms"])

    def test_rejects_bad_checksum(self):
        line = make_telemetry_line("UGEKF", EKF_FIELDS, EKF_INT_FIELDS)
        self.assertIsNone(decode_telemetry_line(line[:-2] + "00"))


class EthernetRealtimeLinkTests(unittest.TestCase):
    @staticmethod
    def receive_subscription(client):
        """TCP 可拆包或粘包，按单字节订阅载荷的帧长读取。"""
        frame_size = len(build_frame(MSG_STREAM_CONTROL, b"\x00"))
        data = bytearray()
        while len(data) < frame_size:
            chunk = client.recv(frame_size - len(data))
            if not chunk:
                raise AssertionError("连接在订阅帧接收完成前关闭")
            data.extend(chunk)
        return parse_frame(bytes(data))

    def test_old_firmware_keeps_base_topics_after_rejecting_rdss(self):
        store = StateStore()
        link = EthernetRealtimeLink(store)
        active_mask = 0
        observed_masks = []
        sequences = []

        def firmware_receive(frame):
            nonlocal active_mask
            msg_type, sequence, payload = parse_frame(frame)
            self.assertEqual(msg_type, MSG_STREAM_CONTROL)
            self.assertEqual(len(payload), 1)
            mask = payload[0]
            observed_masks.append(mask)
            sequences.append(sequence)
            # 模拟旧固件：含未知位的整条订阅被拒绝，原有订阅保持。
            if not mask & STREAM_RDSS:
                active_mask = mask

        connection = Mock()
        connection.sendall.side_effect = firmware_receive
        link._send_stream_subscription(connection)

        self.assertEqual(observed_masks, [0x0D, 0x2D])
        self.assertEqual(active_mask, 0x0D)
        self.assertEqual(sequences[1], sequences[0] + 1)

    def test_subscription_preserves_requested_topics_and_disabled_mask(self):
        cases = (
            (0, [0]),
            (STREAM_EKF, [STREAM_EKF]),
            (STREAM_WRM, [STREAM_WRM]),
            (STREAM_WRM | STREAM_RDSS, [STREAM_WRM, STREAM_WRM | STREAM_RDSS]),
            (STREAM_RDSS, [0, STREAM_RDSS]),
        )
        for requested, expected in cases:
            with self.subTest(requested=requested):
                connection = Mock()
                link = EthernetRealtimeLink(StateStore(), stream_mask=requested)
                link._send_stream_subscription(connection)
                frames = [parse_frame(call.args[0])
                          for call in connection.sendall.call_args_list]
                self.assertEqual([frame[0] for frame in frames],
                                 [MSG_STREAM_CONTROL] * len(expected))
                self.assertEqual([frame[2] for frame in frames],
                                 [bytes((mask,)) for mask in expected])

    def test_loopback_receives_fragmented_line_and_publishes_state(self):
        store = StateStore()
        link = EthernetRealtimeLink(
            store, host="127.0.0.1", port=0, retry_delay_s=0.1
        )
        client = None
        try:
            link.start()
            self.assertTrue(link.wait_until_listening(2.0))
            endpoint = link.bound_endpoint
            self.assertIsNotNone(endpoint)

            client = socket.create_connection(endpoint, timeout=2.0)
            for mask in (DEFAULT_STREAM_MASK & ~STREAM_RDSS, DEFAULT_STREAM_MASK):
                msg_type, _sequence, payload = self.receive_subscription(client)
                self.assertEqual(msg_type, MSG_STREAM_CONTROL)
                self.assertEqual(payload, bytes((mask,)))

            line = make_telemetry_line(
                "UGEKF", EKF_FIELDS, EKF_INT_FIELDS,
                {"t_ms": 456, "status": 4, "fl": 3},
            ).encode("ascii") + b"\r\n"
            midpoint = len(line) // 2
            client.sendall(line[:midpoint])
            client.sendall(line[midpoint:])

            deadline = time.monotonic() + 2.0
            snapshot = None
            while time.monotonic() < deadline:
                _generation, snapshot = store.read()
                if "ugekf" in snapshot["telemetry"]:
                    break
                time.sleep(0.01)

            self.assertIsNotNone(snapshot)
            self.assertIn("ugekf", snapshot["telemetry"])
            self.assertEqual(snapshot["device_link"]["status"], "connected")
            self.assertEqual(
                snapshot["telemetry"]["ugekf"]["device_tick_ms"], 456
            )
            self.assertTrue(snapshot["telemetry"]["ugekf"]["fresh"])
        finally:
            if client is not None:
                client.close()
            link.stop()

        _generation, stopped = store.read()
        self.assertEqual(stopped["device_link"]["status"], "stopped")

    def test_rdss_receive_records_valid_rows_and_preserves_bad_raw_lines(self):
        valid_line = make_telemetry_line(
            "UGRDS", UGRDS_FIELDS, UGRDS_INT_FIELDS,
            {"seq": 10, "t_ms": 456, "last_result": 3,
             "has_cached_fix": 1, "local_latitude_deg_e7": -220000000,
             "local_longitude_deg_e7": 1145000000},
        )
        damaged_line = make_telemetry_line(
            "UGRDS", UGRDS_FIELDS, UGRDS_INT_FIELDS,
            {"seq": 11, "t_ms": 457},
        )
        # 合法帧先到；后续额外 CR 的坏帧不能刷新状态或进入 CSV。
        connection = Mock()
        connection.recv.side_effect = [
            (valid_line + "\r\n" + damaged_line + "\r\r\n").encode("ascii"), b"",
        ]
        store = StateStore()
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(Path(directory))
            link = EthernetRealtimeLink(store, recorder=recorder)
            recorder.begin_session("rdss_test", "127.0.0.1:12345")
            try:
                self.assertEqual(link._receive_lines(connection, "rdss_test"), "peer_closed")
            finally:
                recorder.end_session("peer_closed")
                recorder.stop()

            _generation, snapshot = store.read()
            rdss = snapshot["telemetry"]["ugrds"]
            self.assertEqual(rdss["device_tick_ms"], 456)
            self.assertEqual(rdss["data"]["seq"], 10)
            self.assertEqual(rdss["data"]["local_latitude_deg_e7"], -220000000)

            recording = recorder.snapshot()
            self.assertIsNone(recording["last_error"])
            self.assertEqual(recording["raw_lines"], 2)
            self.assertEqual(recording["rows"], {"ugrds": 1})
            with Path(recording["files"]["ugrds"]).open(
                    encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["seq"], "10")
            self.assertEqual(rows[0]["local_latitude_deg_e7"], "-220000000")
            raw = Path(recording["files"]["raw"]).read_bytes()
            self.assertIn((damaged_line + "\r\n").encode("ascii"), raw)


if __name__ == "__main__":
    unittest.main()
