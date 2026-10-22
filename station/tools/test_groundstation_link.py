#!/usr/bin/env python3
"""URGC 旧遥测适配器和 Ethernet 唯一接收者回归测试。"""

import socket
import sys
import time
import unittest
from pathlib import Path


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
    decode_telemetry_line,
)
from groundstation.state_store import StateStore  # noqa: E402
from ugcp_protocol import MSG_STREAM_CONTROL, parse_frame  # noqa: E402


def make_telemetry_line(prefix, fields, integer_fields, overrides=None):
    """按现有板端格式生成一行带 XOR 校验的测试遥测。"""

    values = []
    overrides = overrides or {}
    for index, name in enumerate(fields):
        if name in overrides:
            value = overrides[name]
        elif name == "schema":
            value = 2
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
    def test_decodes_the_three_initial_telemetry_topics(self):
        cases = (
            ("UGEKF", EKF_FIELDS, EKF_INT_FIELDS,
             {"t_ms": 123, "status": 4, "fl": 3}, "ugekf"),
            ("UGCTL", UGCTL_FIELDS, UGCTL_INT_FIELDS,
             {"schema": 2, "t_ms": 234, "armed": 0}, "ugctl"),
            ("UGWRM", UGWRM_FIELDS, UGWRM_INT_FIELDS,
             {"schema": 2, "t_ms": 345}, "ugwrm"),
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
            msg_type, _sequence, payload = parse_frame(client.recv(64))
            self.assertEqual(msg_type, MSG_STREAM_CONTROL)
            self.assertEqual(payload, bytes((DEFAULT_STREAM_MASK,)))

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


if __name__ == "__main__":
    unittest.main()
