#!/usr/bin/env python3
"""会话日志的实际落盘、故障和跨连接隔离测试。"""

import csv
import json
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path

STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from groundstation.ethernet_link import EthernetRealtimeLink
from groundstation.legacy_telemetry import (
    DecodedTelemetry, EKF_FIELDS, EKF_CSV_HEADER, UGCTL_FIELDS,
    UGCTL_CSV_HEADER, UGWRM_FIELDS, UGWRM_CSV_HEADER,
)
from groundstation.session_recorder import SessionRecorder
from groundstation.state_store import StateStore


class SessionRecorderTests(unittest.TestCase):
    def test_raw_fault_values_and_three_compatible_csv_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            recorder.begin_session("session1", "127.0.0.1:1234")
            cases = (
                ("ugekf", EKF_FIELDS, EKF_CSV_HEADER),
                ("ugctl", UGCTL_FIELDS, UGCTL_CSV_HEADER),
                ("ugwrm", UGWRM_FIELDS, UGWRM_CSV_HEADER),
            )
            for topic, fields, _header in cases:
                data = dict.fromkeys(fields, 0)
                data["t_ms"] = 123
                if topic == "ugekf":
                    data["roll_deg"] = float("nan")
                recorder.record("raw-" + topic, DecodedTelemetry(topic, data, 123, 500))
            recorder.record("bad-checksum-line", None)
            recorder.end_session("peer_closed")
            recorder.stop()
            snapshot = recorder.snapshot()
            self.assertEqual(snapshot["status"], "stopped")
            self.assertEqual(snapshot["raw_lines"], 4)
            self.assertEqual(snapshot["rows"], {"ugekf": 1, "ugctl": 1, "ugwrm": 1})
            for topic, _fields, header in cases:
                with Path(snapshot["files"][topic]).open(encoding="utf-8", newline="") as handle:
                    rows = list(csv.reader(handle))
                self.assertEqual(rows[0], header)
                self.assertEqual(len(rows[1]), len(header))
                self.assertEqual(rows[1][header.index("t_ms")], "123")
            events = [json.loads(line) for line in Path(snapshot["files"]["events"]).read_text(encoding="utf-8").splitlines()]
            self.assertEqual([event["type"] for event in events], ["session.started", "session.ended"])
            self.assertIn("bad-checksum-line", Path(snapshot["files"]["raw"]).read_text(encoding="utf-8"))

    def test_reconnect_gets_separate_session_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            for session in ("first", "second"):
                recorder.begin_session(session, "loopback")
                recorder.record(session, None)
                recorder.end_session("peer_closed")
            recorder.stop()
            files = list(Path(temporary).glob("*/raw.log"))
            self.assertEqual(len(files), 2)
            contents = [file.read_text(encoding="utf-8") for file in files]
            self.assertEqual(sum("first" in content for content in contents), 1)
            self.assertEqual(sum("second" in content for content in contents), 1)

    def test_disk_error_is_visible_and_does_not_escape_receiver(self):
        with tempfile.TemporaryDirectory() as temporary:
            # 故意把根目录指向已有普通文件，模拟无法创建日志目录。
            invalid_root = Path(temporary) / "not-a-directory"
            invalid_root.touch()
            recorder = SessionRecorder(invalid_root)
            recorder.begin_session("failure", "loopback")
            recorder.record("telemetry", None)
            recorder.stop()
            snapshot = recorder.snapshot()
            self.assertEqual(snapshot["status"], "error")
            self.assertTrue(snapshot["last_error"])

    def test_tcp_receiver_records_the_same_line_it_publishes(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = SessionRecorder(Path(temporary))
            store = StateStore()
            link = EthernetRealtimeLink(store, host="127.0.0.1", port=0, recorder=recorder)
            client = None
            try:
                link.start()
                self.assertTrue(link.wait_until_listening(2.0))
                client = socket.create_connection(link.bound_endpoint, timeout=2.0)
                client.recv(64)
                values = ["4" if name == "status" else "3" if name == "fl" else "0" for name in EKF_FIELDS]
                body = "UGEKF," + ",".join(values)
                checksum = 0
                for character in body:
                    checksum ^= ord(character)
                line = "$" + body + "*{:02X}\r\n".format(checksum)
                client.sendall(line.encode("ascii"))
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if "ugekf" in store.read()[1]["telemetry"] and recorder.snapshot()["rows"].get("ugekf") == 1:
                        break
                    time.sleep(0.01)
                self.assertIn("ugekf", store.read()[1]["telemetry"])
                self.assertEqual(recorder.snapshot()["rows"].get("ugekf"), 1)
            finally:
                if client is not None:
                    client.close()
                link.stop()
            self.assertEqual(recorder.snapshot()["status"], "stopped")


if __name__ == "__main__":
    unittest.main()
