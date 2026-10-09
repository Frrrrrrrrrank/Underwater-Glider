#!/usr/bin/env python3
"""URGC 状态模型与只读 API 路由回归测试。"""

import sys
import unittest
from pathlib import Path


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from groundstation.models import LinkStatus  # noqa: E402
from groundstation.state_store import StateStore  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value


class StateStoreTests(unittest.TestCase):
    def test_initial_snapshot_is_explicitly_read_only_and_offline(self) -> None:
        store = StateStore()
        generation, snapshot = store.read()

        self.assertEqual(generation, 0)
        self.assertEqual(snapshot["backend"]["name"], "UnderwaterRobotGroundControl")
        self.assertEqual(snapshot["device_link"]["status"], "stopped")
        self.assertFalse(snapshot["state"]["overall_fresh"])
        self.assertEqual(snapshot["state"]["command_capability"], "disabled")
        self.assertTrue(snapshot["state"]["capabilities"]["observe"])
        self.assertFalse(snapshot["state"]["capabilities"]["realtime_control"])
        self.assertFalse(snapshot["state"]["capabilities"]["autonomous_mission"])

    def test_telemetry_freshness_uses_host_monotonic_time(self) -> None:
        clock = FakeClock()
        store = StateStore(clock=clock, required_topics=("ugctl",))
        store.set_link(LinkStatus.CONNECTED, peer="192.0.2.10:8800", session_id="session-1")
        store.publish_telemetry(
            "ugctl",
            {"armed": 0},
            session_id="session-1",
            device_tick_ms=1234,
            stale_after_ms=500,
        )

        _generation, fresh = store.read()
        self.assertTrue(fresh["telemetry"]["ugctl"]["fresh"])
        self.assertTrue(fresh["state"]["overall_fresh"])

        clock.value += 0.501
        _generation, stale = store.read()
        self.assertFalse(stale["telemetry"]["ugctl"]["fresh"])
        self.assertEqual(stale["state"]["stale_reason"], "stale_telemetry")

    def test_input_mapping_is_copied_and_wait_reports_timeout(self) -> None:
        clock = FakeClock()
        store = StateStore(clock=clock)
        payload = {"nested": {"value": 1}}
        generation = store.publish_telemetry("ekf", payload)
        payload["nested"]["value"] = 99

        _generation, snapshot = store.read()
        self.assertEqual(snapshot["telemetry"]["ekf"]["data"]["nested"]["value"], 1)

        waited_generation, _waited_snapshot, changed = store.wait_for_change(generation, 0.0)
        self.assertEqual(waited_generation, generation)
        self.assertFalse(changed)


class ApiShapeTests(unittest.TestCase):
    def test_app_exposes_no_write_route(self) -> None:
        try:
            from groundstation.app import create_app
        except ModuleNotFoundError as exc:
            if exc.name in {"fastapi", "starlette"}:
                self.skipTest("尚未安装 FastAPI 依赖")
            raise

        # 路由形状测试不需要占用设备端口；链路线程由独立回环测试覆盖。
        app = create_app(StateStore(), device_link_enabled=False)
        http_methods = {
            method
            for route in app.routes
            for method in (getattr(route, "methods", None) or set())
        }
        paths = {route.path for route in app.routes}

        self.assertIn("/api/v1/health", paths)
        self.assertIn("/api/v1/state", paths)
        self.assertIn("/api/v1/ws/state", paths)
        self.assertTrue(http_methods.isdisjoint({"POST", "PUT", "PATCH", "DELETE"}))


if __name__ == "__main__":
    unittest.main()
