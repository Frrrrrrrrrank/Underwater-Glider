"""本机只读桥接的时间、质量、会话隔离与 HTTP 接口回归测试。"""

import http.client
import json
from pathlib import Path
import socket
import sys
import threading
import unittest
from unittest.mock import patch


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from monitor_bridge import MonitorBridge  # noqa: E402
import monitor_bridge  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class MonitorBridgeStateTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.bridge = MonitorBridge(port=0, clock=self.clock)
        self.bridge.set_link("connected", peer="192.0.2.1:12345")
        self.addCleanup(self.bridge.stop)

    def sample(self, topic):
        return self.bridge.snapshot()["telemetry"][topic]

    def test_different_topic_periods_and_duplicate_frames(self):
        self.assertTrue(self.bridge.publish("ugekf", {"seq": 1, "t_ms": 10, "fl": 2}))
        self.assertTrue(self.bridge.publish("ugbat", {
            "seq": 1, "t_ms": 10, "valid": 1, "filtered_mv": 27000, "sample_age_ms": 30,
        }))
        self.clock.now += 0.6
        self.assertFalse(self.bridge.publish("ugekf", {"seq": 1, "t_ms": 10, "fl": 2}))
        self.assertFalse(self.sample("ugekf")["fresh"])
        self.assertTrue(self.sample("ugbat")["fresh"])
        self.assertGreaterEqual(self.sample("ugekf")["age_ms"], 599)
        self.bridge.publish("ugctl", {"seq": 9, "t_ms": 999, "output_valid": 0})
        self.assertFalse(self.sample("ugekf")["fresh"])
        self.clock.now += 2.5
        self.assertFalse(self.bridge.publish("ugbat", {
            "seq": 1, "t_ms": 10, "valid": 1, "filtered_mv": 27000, "sample_age_ms": 30,
        }))
        self.assertFalse(self.sample("ugbat")["fresh"])
        self.assertTrue(self.bridge.publish("ugekf", {"seq": 2, "t_ms": 20, "fl": 2}))
        self.assertTrue(self.sample("ugekf")["fresh"])

    def test_disconnect_and_new_connection_do_not_reuse_old_samples(self):
        row = {"seq": 1, "t_ms": 10, "fl": 2, "pitch_deg": 2.5}
        self.bridge.publish("ugekf", row)
        session = self.bridge.snapshot()["device_link"]["session_id"]
        self.bridge.set_link("disconnected", last_error="peer_closed")
        snapshot = self.bridge.snapshot()
        self.assertFalse(snapshot["state"]["overall_fresh"])
        self.assertFalse(self.sample("ugekf")["fresh"])
        self.assertEqual(self.sample("ugekf")["data"]["pitch_deg"], 2.5)
        self.assertFalse(self.bridge.publish("ugekf", dict(row, seq=2)))
        self.bridge.set_link("connected", peer="192.0.2.2:12345")
        snapshot = self.bridge.snapshot()
        self.assertNotEqual(snapshot["device_link"]["session_id"], session)
        self.assertEqual(snapshot["telemetry"], {})
        self.assertTrue(self.bridge.publish("ugekf", row))

    def test_repeated_connected_call_deliberately_opens_new_session(self):
        self.bridge.publish("ugctl", {"seq": 1, "t_ms": 0})
        previous = self.bridge.snapshot()["device_link"]["session_id"]
        self.bridge.set_link("connected")
        snapshot = self.bridge.snapshot()
        self.assertNotEqual(snapshot["device_link"]["session_id"], previous)
        self.assertEqual(snapshot["telemetry"], {})

    def test_rdss_coordinates_and_freshness_are_independent(self):
        row = {"seq": 1, "t_ms": 100, "has_cached_fix": 0, "target_valid": 1,
               "local_latitude_deg_e7": 2147483647, "local_longitude_deg_e7": 1140000000,
               "target_latitude_deg_e7": 220000000, "target_longitude_deg_e7": -1140000000}
        self.assertTrue(self.bridge.publish("ugrds", row))
        sample = self.sample("ugrds")
        self.assertEqual(sample["invalid_fields"]["local_latitude_deg_e7"], "local_fix_unavailable")
        self.assertNotIn("target_latitude_deg_e7", sample["invalid_fields"])
        self.clock.now += 1.9
        self.assertTrue(self.sample("ugrds")["fresh"])
        self.assertFalse(self.bridge.publish("ugrds", row))
        self.clock.now += 0.2
        self.assertFalse(self.sample("ugrds")["fresh"])
        self.bridge.publish("ugrds", dict(row, seq=2, target_valid=0))
        self.assertIn("target_latitude_deg_e7", self.sample("ugrds")["invalid_fields"])
        self.bridge.set_link("disconnected")
        self.assertFalse(self.sample("ugrds")["fresh"])
        self.bridge.set_link("connected")
        self.assertNotIn("ugrds", self.bridge.snapshot()["telemetry"])

    def test_rdss_valid_flag_does_not_allow_out_of_range_coordinates(self):
        self.bridge.publish("ugrds", {"seq": 1, "t_ms": 1, "has_cached_fix": 1,
                            "local_latitude_deg_e7": 900000001,
                            "local_longitude_deg_e7": 1140000000})
        self.assertEqual(self.sample("ugrds")["invalid_fields"]["local_latitude_deg_e7"],
                         "coordinate_out_of_range")

    def test_experiment_provider_is_live_owned_and_failure_isolated(self):
        status = {"status": "recording", "dropped_records": 0, "directory": "logs/session"}
        self.bridge.set_experiment_provider(lambda: status)
        snapshot = self.bridge.snapshot()
        snapshot["experiment"]["directory"] = "changed"
        self.assertEqual(status["directory"], "logs/session")
        status.update(status="error", last_error="disk full")
        self.assertEqual(self.bridge.snapshot()["experiment"]["last_error"], "disk full")
        for provider in (lambda: float("nan"), lambda: {"bad": float("nan")},
                         lambda: (_ for _ in ()).throw(OSError("write failure"))):
            self.bridge.set_experiment_provider(provider)
            result = self.bridge.snapshot()
            self.assertEqual(result["experiment"]["status"], "error")
            self.assertEqual(result["device_link"]["status"], "connected")
            json.dumps(result, allow_nan=False)
        self.bridge.set_experiment_provider(None)
        self.assertNotIn("experiment", self.bridge.snapshot())

    def test_experiment_provider_does_not_hold_telemetry_lock(self):
        completed = threading.Event()

        def provider():
            worker = threading.Thread(target=lambda: (
                self.bridge.publish("ugctl", {"seq": 1, "t_ms": 1}), completed.set()))
            worker.start()
            acquired = completed.wait(1.0)
            worker.join(timeout=1.0)
            return {"status": "recording", "lock_free": acquired}

        self.bridge.set_experiment_provider(provider)
        self.assertTrue(self.bridge.snapshot()["experiment"]["lock_free"])

    def test_latest_only_owned_data_and_unknown_capabilities(self):
        row = {"seq": 1, "t_ms": 0, "nested": {"value": 12}}
        self.bridge.publish("ugctl", row)
        row["nested"]["value"] = 99
        snapshot = self.bridge.snapshot()
        snapshot["telemetry"]["ugctl"]["data"]["nested"]["value"] = 88
        self.assertEqual(self.sample("ugctl")["data"]["nested"]["value"], 12)
        self.bridge.publish("ugctl", {"seq": 2, "t_ms": 20, "armed": 0})
        self.assertNotIn("nested", self.sample("ugctl")["data"])
        self.assertEqual(self.sample("ugctl")["device_tick_ms"], 20)
        capabilities = snapshot["state"]["capabilities"]
        for field in ("pid_parameters_readback", "final_pwm_feedback", "battery_soc",
                      "realtime_control", "parameter_write"):
            self.assertFalse(capabilities[field])
        self.assertNotIn("ugbat", snapshot["telemetry"])

    def test_pitch_reference_and_roll_validity_are_independent(self):
        base = {
            "seq": 1, "t_ms": 10, "stroke_percent": 25.0, "mm_from_front": 5.0,
            "pulse": 100, "fb_rpm": 5, "span_valid": 1, "data_fresh": 1,
            "ref_kind": 0, "roll_percent": 50.0, "roll_valid": 1,
        }
        self.bridge.publish("ugwrm", base)
        invalid = self.sample("ugwrm")["invalid_fields"]
        self.assertEqual(invalid["stroke_percent"], "worm_reference_unavailable")
        self.assertNotIn("roll_percent", invalid)
        for reference in (1, 2):
            with self.subTest(reference=reference):
                self.bridge.publish("ugwrm", dict(base, seq=reference + 1, ref_kind=reference))
                self.assertNotIn("stroke_percent", self.sample("ugwrm")["invalid_fields"])
        self.bridge.publish("ugwrm", dict(base, seq=4, ref_kind=2, data_fresh=0))
        invalid = self.sample("ugwrm")["invalid_fields"]
        self.assertIn("stroke_percent", invalid)
        self.assertIn("fb_rpm", invalid)
        self.assertNotIn("roll_percent", invalid)
        self.bridge.publish("ugwrm", dict(base, seq=5, ref_kind=2, roll_valid=0))
        invalid = self.sample("ugwrm")["invalid_fields"]
        self.assertNotIn("stroke_percent", invalid)
        self.assertIn("roll_percent", invalid)

    def test_missing_validity_never_means_valid(self):
        self.bridge.publish("ugwrm", {
            "seq": 1, "t_ms": 10, "ref_kind": 2, "stroke_percent": 0.0,
            "roll_percent": 0.0,
        })
        invalid = self.sample("ugwrm")["invalid_fields"]
        self.assertIn("stroke_percent", invalid)
        self.assertIn("roll_percent", invalid)
        self.bridge.publish("ugbat", {"seq": 1, "t_ms": 10, "filtered_mv": 0})
        self.assertIn("filtered_mv", self.sample("ugbat")["invalid_fields"])

    def test_battery_sample_quality_is_separate_from_telemetry_freshness(self):
        base = {"seq": 1, "t_ms": 10, "battery_mv": 26000, "filtered_mv": 25900,
                "valid": 1, "sample_age_ms": 500}
        self.bridge.publish("ugbat", base)
        self.assertNotIn("filtered_mv", self.sample("ugbat")["invalid_fields"])
        self.bridge.publish("ugbat", dict(base, seq=2, sample_age_ms=501))
        sample = self.sample("ugbat")
        self.assertTrue(sample["fresh"])
        self.assertEqual(sample["invalid_fields"]["filtered_mv"], "battery_sample_stale")
        self.bridge.publish("ugbat", dict(base, seq=3, valid=0))
        self.assertEqual(self.sample("ugbat")["invalid_fields"]["filtered_mv"], "battery_invalid")

    def test_nonfinite_values_are_json_null_with_reason(self):
        self.bridge.publish("ugekf", {
            "seq": 1, "t_ms": 1, "fl": 2, "pitch_deg": float("nan"),
            "roll_deg": float("inf"),
        })
        sample = self.sample("ugekf")
        self.assertIsNone(sample["data"]["pitch_deg"])
        self.assertIsNone(sample["data"]["roll_deg"])
        self.assertEqual(sample["invalid_fields"]["pitch_deg"], "non_finite")
        json.dumps(self.bridge.snapshot(), allow_nan=False)

    def test_unusable_frame_identifiers_are_not_published(self):
        for row in ({}, {"seq": -1, "t_ms": 10}, {"seq": True, "t_ms": 10},
                    {"seq": 1, "t_ms": None}):
            self.assertFalse(self.bridge.publish("ugctl", row))
        self.assertEqual(self.bridge.snapshot()["telemetry"], {})
        with self.assertRaises(ValueError):
            self.bridge.publish("control", {"seq": 1, "t_ms": 10})


class MonitorBridgeHttpTests(unittest.TestCase):
    def test_thread_start_failure_closes_bound_socket_and_allows_retry(self):
        bridge = MonitorBridge(port=0)
        self.addCleanup(bridge.stop)
        created = []
        server_type = monitor_bridge._SnapshotServer

        def create_server(*args):
            server = server_type(*args)
            created.append(server)
            return server

        with patch.object(monitor_bridge, "_SnapshotServer", side_effect=create_server):
            with patch.object(monitor_bridge.threading.Thread, "start",
                              side_effect=RuntimeError("模拟线程创建失败")):
                with self.assertRaises(RuntimeError):
                    bridge.start()
        self.assertEqual(len(created), 1)
        self.assertEqual(created[0].socket.fileno(), -1)
        # 失败前已 bind 的真实端口可以立即再次绑定，不能只检查对象标志。
        with socket.socket() as probe:
            probe.bind(created[0].server_address)
        self.assertGreater(bridge.start()[1], 0)

    def test_binding_conflict_closes_failed_server_without_affecting_owner(self):
        owner = MonitorBridge(port=0)
        self.addCleanup(owner.stop)
        address = owner.start()
        contender = MonitorBridge(host=address[0], port=address[1])
        self.addCleanup(contender.stop)
        closed = []
        close_server = monitor_bridge._SnapshotServer.server_close

        def record_close(server):
            closed.append(server)
            return close_server(server)

        with patch.object(monitor_bridge._SnapshotServer, "server_close", new=record_close):
            with self.assertRaises(OSError):
                contender.start()
        self.assertEqual(len(closed), 1)
        self.assertEqual(closed[0].socket.fileno(), -1)
        connection = http.client.HTTPConnection(*address, timeout=2.0)
        self.addCleanup(connection.close)
        connection.request("GET", "/snapshot")
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        response.read()
        connection.close()
        owner.stop()
        self.assertEqual(contender.start(), address)

    def test_http_get_only_and_lifecycle(self):
        bridge = MonitorBridge(port=0)
        self.addCleanup(bridge.stop)
        address = bridge.start()
        self.assertEqual(address, bridge.address)
        self.assertEqual(address, bridge.start())
        self.assertGreater(address[1], 0)
        connection = http.client.HTTPConnection(*address, timeout=2.0)
        self.addCleanup(connection.close)
        connection.request("GET", "/snapshot")
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Cache-Control"), "no-store")
        snapshot = json.loads(response.read())
        self.assertTrue(snapshot["monitor"]["read_only"])
        self.assertEqual(snapshot["device_link"]["status"], "stopped")
        for method in ("POST", "PUT", "PATCH", "DELETE", "HEAD", "OPTIONS"):
            connection.request(method, "/snapshot")
            response = connection.getresponse()
            self.assertEqual(response.status, 405)
            response.read()
        connection.request("GET", "/control")
        response = connection.getresponse()
        self.assertEqual(response.status, 404)
        response.read()
        bridge.stop()
        bridge.stop()
        self.assertEqual(bridge.snapshot()["device_link"]["status"], "stopped")

    def test_no_nonloopback_listener_and_invalid_ports(self):
        for host in ("0.0.0.0", "192.0.2.1", "::", "example.test"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                MonitorBridge(host=host, port=0)
        for port in (-1, 65536, True, "8801"):
            with self.subTest(port=port), self.assertRaises(ValueError):
                MonitorBridge(port=port)
        self.assertEqual(MonitorBridge(host="localhost", port=0).address, ("127.0.0.1", 0))


if __name__ == "__main__":
    unittest.main()
