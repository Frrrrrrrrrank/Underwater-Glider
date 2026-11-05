#!/usr/bin/env python3
"""URGC 手柄预览的生命周期、质量、映射与只读接口回归。"""

import asyncio
import json
import math
import sys
import time
import unittest
from dataclasses import asdict, replace
from pathlib import Path
from unittest.mock import patch


STATION_DIR = Path(__file__).resolve().parents[1]
if str(STATION_DIR) not in sys.path:
    sys.path.insert(0, str(STATION_DIR))

from gamepad_control import (  # noqa: E402
    GamepadConfig, GamepadService, GamepadSnapshot, map_gamepad_to_target,
)
from groundstation.gamepad_preview import GamepadPreview  # noqa: E402
from groundstation.state_store import StateStore  # noqa: E402


class FakeClock:
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        return self.value


class FakeService:
    def __init__(self, config=None):
        self.config = config
        self.alive = False
        self.starts = 0
        self.stops = 0
        self.reads = 0
        self.samples = [GamepadSnapshot(index=index) for index in range(4)]

    def start(self):
        self.starts += 1
        self.alive = True

    def stop(self):
        self.stops += 1
        self.alive = False

    def is_alive(self):
        return self.alive

    def snapshot(self, index):
        self.reads += 1
        return self.samples[index]

    def poll_once(self):
        raise AssertionError("预览读取不得触发采样")


class GamepadPreviewTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.service = FakeService()
        self.factory_calls = 0

        def factory(config):
            self.factory_calls += 1
            self.service.config = config
            return self.service

        self.preview = GamepadPreview(service_factory=factory, clock=self.clock)

    def sample(self, index=0, **changes):
        self.service.samples[index] = GamepadSnapshot(
            index=index, connected=True, sample_time=self.clock.value, **changes,
        )

    def slot(self, index=0):
        return self.preview.snapshot()["slots"][index]

    def test_construction_and_read_are_lazy_and_start_is_idempotent(self):
        self.assertEqual(self.factory_calls, 0)
        self.assertEqual(self.preview.snapshot()["status"], "stopped")
        self.assertEqual(self.factory_calls, 0)
        self.preview.start()
        self.preview.start()
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(self.service.starts, 1)
        self.assertEqual(self.preview.snapshot()["status"], "running")
        self.assertEqual(self.service.reads, 4)
        self.preview.stop()
        self.preview.stop()
        self.assertEqual(self.service.stops, 1)
        self.assertEqual(self.preview.snapshot()["status"], "stopped")

    def test_disabled_preview_never_constructs_device_service(self):
        def forbidden_factory(_config):
            raise AssertionError("禁用后不应构造 XInput")

        preview = GamepadPreview(enabled=False, service_factory=forbidden_factory)
        preview.start()
        preview.stop()
        snapshot = preview.snapshot()
        self.assertEqual(snapshot["status"], "disabled")
        self.assertTrue(all(slot["sample"] is None for slot in snapshot["slots"]))

    def test_missing_dll_isolated_as_error(self):
        def missing_dll(_config):
            raise OSError("XInput DLL 不可用")

        preview = GamepadPreview(service_factory=missing_dll)
        preview.start()
        snapshot = preview.snapshot()
        self.assertEqual(snapshot["status"], "error")
        self.assertIn("DLL", snapshot["error"])
        self.assertTrue(all(not slot["fresh"] for slot in snapshot["slots"]))
        preview.stop()

    def test_partial_start_failure_cleans_up_service(self):
        def fail_start():
            self.service.alive = True
            raise OSError("线程启动失败")

        self.service.start = fail_start
        self.preview.start()
        self.assertEqual(self.preview.snapshot()["status"], "error")
        self.assertEqual(self.service.stops, 1)
        self.assertFalse(self.service.alive)

    def test_hung_stop_cannot_create_second_sampler(self):
        self.preview.start()
        self.service.stop = lambda: None
        self.preview.stop()
        self.assertEqual(self.preview.snapshot()["status"], "error")
        self.assertFalse(self.slot()["fresh"])
        self.preview.start()
        self.assertEqual(self.factory_calls, 1)
        self.service.alive = False
        self.preview.stop()
        self.assertEqual(self.preview.snapshot()["status"], "stopped")

    def test_all_four_slots_are_independent_and_missing_device_is_not_zero(self):
        self.preview.start()
        self.sample(index=0, forward=0.75)
        self.sample(index=2, button_a=True)
        self.service.samples[1] = GamepadSnapshot(index=1, error="手柄未连接")
        snapshot = self.preview.snapshot()
        self.assertEqual([slot["index"] for slot in snapshot["slots"]], [0, 1, 2, 3])
        self.assertEqual([slot["status"] for slot in snapshot["slots"]],
                         ["fresh", "disconnected", "fresh", "waiting"])
        self.assertIsNone(snapshot["slots"][1]["intent"])
        self.assertIsNone(snapshot["slots"][3]["neutral"])
        self.assertEqual(snapshot["mode"], "preview_only")
        self.assertEqual(snapshot["source"], "core_xinput")
        self.assertEqual(snapshot["config"], asdict(GamepadConfig()))

    def test_current_mapping_reused_without_second_mapping(self):
        self.preview.start()
        cases = [
            {}, {"forward": 1.0, "left_x": 0.4},
            {"forward": -0.8, "left_x": -0.2}, {"left_x": 1.0},
            {"button_y": True}, {"button_a": True},
            {"dpad_left": True}, {"dpad_right": True},
            {"right_trigger": 0.55}, {"left_trigger": 0.55},
            {"button_b": True, "right_trigger": 1.0},
            {"button_x": True, "forward": 1.0},
        ]
        for changes in cases:
            with self.subTest(changes=changes):
                self.sample(**changes)
                expected = asdict(map_gamepad_to_target(self.service.samples[0], self.preview.config))
                self.assertEqual(self.slot()["intent"], expected)

    def test_release_and_opposed_buttons_remove_motion_intent(self):
        self.preview.start()
        self.sample(button_y=True, dpad_left=True, right_trigger=1.0)
        intent = self.slot()["intent"]
        self.assertEqual((intent["pitch_rpm"], intent["roll_rpm"], intent["rudder_rate_deg_s"]),
                         (-100, 300, 30.0))
        self.sample()
        self.assertTrue(self.slot()["neutral"])
        self.assertTrue(all(value == 0 for value in self.slot()["intent"].values()))
        self.sample(button_y=True, button_a=True, dpad_left=True, dpad_right=True,
                    left_trigger=1.0, right_trigger=1.0)
        self.assertFalse(self.slot()["neutral"])
        self.assertTrue(all(value == 0 for value in self.slot()["intent"].values()))

    def test_preview_does_not_integrate_angle_or_fake_session_b_suppression(self):
        self.preview.start()
        self.sample(button_b=True, right_trigger=1.0, rudder_center_seq=3)
        self.assertTrue(self.slot()["intent"]["rudder_center"])
        self.sample(right_trigger=1.0, rudder_center_seq=3)
        slot = self.slot()
        self.assertEqual(slot["intent"]["rudder_rate_deg_s"], 30.0)
        self.assertEqual(slot["sample"]["rudder_center_seq"], 3)
        self.assertNotIn("rudder_angle_deg", slot["intent"])

    def test_stale_disconnected_and_dead_service_drop_sample_and_intent(self):
        self.preview.start()
        self.sample(button_y=True)
        self.clock.value += 0.201
        stale = self.slot()
        self.assertEqual(stale["status"], "stale")
        self.assertTrue(stale["connected"])
        self.assertAlmostEqual(stale["age_ms"], 201.0)
        self.assertIsNone(stale["sample"])
        self.assertIsNone(stale["intent"])
        self.assertIsNone(stale["neutral"])
        self.service.samples[0] = replace(self.service.samples[0], connected=False, error="拔出")
        self.assertEqual(self.slot()["status"], "disconnected")
        self.sample()
        self.service.alive = False
        self.assertEqual(self.preview.snapshot()["status"], "error")
        self.assertEqual(self.slot()["status"], "unavailable")
        self.assertIsNone(self.slot()["intent"])

    def test_invalid_timestamp_axis_and_event_counter_never_publish_values(self):
        self.preview.start()
        for changes in ({"sample_time": math.nan}, {"sample_time": math.inf},
                        {"sample_time": 100.001}, {"sample_time": -1},
                        {"sample_time": 10 ** 400},
                        {"left_x": math.nan}, {"forward": 1.1},
                        {"left_trigger": -0.1}, {"exit_seq": -1},
                        {"loss_seq": True}, {"index": 1}):
            with self.subTest(changes=changes):
                self.sample()
                self.service.samples[0] = replace(self.service.samples[0], **changes)
                slot = self.slot()
                self.assertEqual(slot["status"], "invalid")
                self.assertFalse(slot["fresh"])
                self.assertIsNone(slot["sample"])
                self.assertIsNone(slot["intent"])
                json.dumps(self.preview.snapshot(), allow_nan=False)

    def test_one_slot_read_failure_does_not_hide_other_slots(self):
        self.preview.start()
        self.sample(index=1)
        original = self.service.snapshot

        def fail_one(index):
            if index == 0:
                raise RuntimeError("该槽位异常")
            return original(index)

        self.service.snapshot = fail_one
        snapshot = self.preview.snapshot()
        self.assertEqual(snapshot["status"], "running")
        self.assertEqual(snapshot["slots"][0]["status"], "unavailable")
        self.assertTrue(snapshot["slots"][1]["fresh"])

    def test_unchanged_input_refreshes_on_successful_poll_not_preview_read(self):
        class ConstantBackend:
            def __init__(self):
                self.calls = 0

            def poll(self, index):
                self.calls += 1
                return GamepadSnapshot(index=index, connected=True, forward=0.5)

        class ManualService(GamepadService):
            def start(self):
                self.alive = True

            def stop(self):
                self.alive = False

            def is_alive(self):
                return self.alive

        backend = ConstantBackend()
        service = ManualService(backend=backend)
        preview = GamepadPreview(service_factory=lambda _config: service, clock=self.clock)
        preview.start()
        for now in (100.0, 100.5, 101.0):
            self.clock.value = now
            with patch("gamepad_control.time.monotonic", return_value=now):
                service.poll_once()
            before = backend.calls
            self.assertTrue(preview.snapshot()["slots"][0]["fresh"])
            self.assertEqual(backend.calls, before)
        self.clock.value += 0.201
        self.assertFalse(preview.snapshot()["slots"][0]["fresh"])
        preview.stop()


class GamepadApiTests(unittest.TestCase):
    def setUp(self):
        try:
            from fastapi import Response, WebSocketDisconnect
            from groundstation.app import create_app
        except ModuleNotFoundError as exc:
            if exc.name in {"fastapi", "starlette"}:
                self.skipTest("尚未安装 FastAPI 依赖；可用 station/.venv 补跑")
            raise
        self.Response = Response
        self.WebSocketDisconnect = WebSocketDisconnect
        self.create_app = create_app
        self.clock = FakeClock()
        self.service = FakeService()
        self.service.samples[0] = GamepadSnapshot(index=0, connected=True, sample_time=100.0)
        self.preview = GamepadPreview(service_factory=lambda _config: self.service, clock=self.clock)
        self.store = StateStore(clock=self.clock)

    @staticmethod
    def endpoint(app, path):
        return next(route.endpoint for route in app.routes if route.path == path)

    def test_app_construction_does_not_load_xinput(self):
        with patch("groundstation.gamepad_preview.GamepadService", side_effect=AssertionError("不可加载")):
            app = self.create_app(device_link_enabled=False)
            self.assertEqual(app.state.gamepad_preview.snapshot()["status"], "stopped")

    def test_environment_disables_only_gamepad(self):
        class LinkSpy:
            started = False
            stopped = False

            def start(self):
                self.started = True

            def stop(self):
                self.stopped = True

        link = LinkSpy()
        with patch.dict("os.environ", {"URGC_GAMEPAD_ENABLED": "0"}):
            app = self.create_app(link_manager=link)

        async def exercise():
            async with app.router.lifespan_context(app):
                self.assertTrue(link.started)
                self.assertEqual(app.state.gamepad_preview.snapshot()["status"], "disabled")
            self.assertTrue(link.stopped)

        asyncio.run(exercise())

    def test_rest_contract_no_store_and_device_disabled_still_reads_gamepad(self):
        app = self.create_app(self.store, device_link_enabled=False, gamepad_preview=self.preview)

        async def exercise():
            async with app.router.lifespan_context(app):
                response = self.Response()
                gamepad = await self.endpoint(app, "/api/v1/input/gamepad")(response)
                self.assertEqual(response.headers["cache-control"], "no-store")
                self.assertTrue(gamepad["slots"][0]["fresh"])
                state_response = self.Response()
                state = await self.endpoint(app, "/api/v1/state")(state_response)
                self.assertEqual(state["gamepad"], gamepad)
                self.assertEqual(state_response.headers["cache-control"], "no-store")
                self.assertEqual(self.store.read()[0], 0)
                self.assertFalse(state["state"]["capabilities"]["realtime_control"])
                self.assertIsNone(app.state.device_link)
            self.assertEqual(self.service.stops, 1)

        asyncio.run(exercise())
        methods = {method for route in app.routes for method in (getattr(route, "methods", None) or [])}
        self.assertTrue(methods.isdisjoint({"POST", "PUT", "PATCH", "DELETE"}))

    def test_dll_failure_does_not_stop_telemetry_lifespan(self):
        class LinkSpy:
            def __init__(self):
                self.calls = []

            def start(self):
                self.calls.append("start")

            def stop(self):
                self.calls.append("stop")

            def send(self, *_args, **_kwargs):
                raise AssertionError("预览不得发送板端命令")

            sendall = send
            send_control = send

        def missing_dll(_config):
            raise OSError("缺少 DLL")

        link = LinkSpy()
        preview = GamepadPreview(service_factory=missing_dll)
        app = self.create_app(self.store, link_manager=link, gamepad_preview=preview)

        async def exercise():
            async with app.router.lifespan_context(app):
                response = self.Response()
                state = await self.endpoint(app, "/api/v1/state")(response)
                self.assertEqual(state["gamepad"]["status"], "error")
                self.assertEqual(link.calls, ["start"])
                health = await self.endpoint(app, "/api/v1/health")()
                self.assertEqual(health["status"], "ok")
            self.assertEqual(link.calls, ["start", "stop"])

        asyncio.run(exercise())

    def test_link_start_failure_still_stops_input_service(self):
        class BrokenLink:
            def start(self):
                raise RuntimeError("设备链路启动失败")

            def stop(self):
                pass

        app = self.create_app(self.store, link_manager=BrokenLink(), gamepad_preview=self.preview)

        async def exercise():
            with self.assertRaisesRegex(RuntimeError, "设备链路启动失败"):
                async with app.router.lifespan_context(app):
                    self.fail("启动失败不应进入服务")
            self.assertEqual(self.service.stops, 1)

        asyncio.run(exercise())

    def test_press_release_and_exit_preview_never_call_device_sender(self):
        class ReadOnlyLink:
            def start(self):
                pass

            def stop(self):
                pass

            def forbidden_send(self, *_args, **_kwargs):
                raise AssertionError("只读手柄预览不得调用任何控制发送")

            send = forbidden_send
            sendall = forbidden_send
            send_control = forbidden_send
            send_ugcp_frame = forbidden_send

        app = self.create_app(self.store, link_manager=ReadOnlyLink(), gamepad_preview=self.preview)

        async def exercise():
            async with app.router.lifespan_context(app):
                for buttons in ({"button_y": True, "right_trigger": 1.0}, {}, {"button_x": True}):
                    self.service.samples[0] = GamepadSnapshot(
                        index=0, connected=True, sample_time=self.clock.value, **buttons)
                    snapshot = await self.endpoint(app, "/api/v1/state")(self.Response())
                    self.assertTrue(snapshot["gamepad"]["slots"][0]["fresh"])
                    self.assertEqual(snapshot["state"]["command_capability"], "disabled")

        asyncio.run(exercise())

    def test_ws_updates_input_and_age_without_telemetry_generation_change(self):
        app = self.create_app(self.store, device_link_enabled=False, gamepad_preview=self.preview)
        frames = []
        sent_at = []
        outer = self

        class FakeWebSocket:
            async def accept(self):
                pass

            async def send_json(self, frame):
                frames.append(json.loads(json.dumps(frame, allow_nan=False)))
                sent_at.append(time.monotonic())
                if len(frames) == 1:
                    outer.service.samples[0] = replace(outer.service.samples[0], button_y=True)
                elif len(frames) == 2:
                    outer.clock.value += 0.201
                else:
                    raise outer.WebSocketDisconnect()

        async def exercise():
            async with app.router.lifespan_context(app):
                await self.endpoint(app, "/api/v1/ws/state")(FakeWebSocket())

        asyncio.run(exercise())
        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[1]["type"], "heartbeat")
        self.assertEqual(frames[1]["data"]["gamepad"]["slots"][0]["intent"]["pitch_rpm"], -100)
        self.assertEqual(frames[2]["data"]["gamepad"]["slots"][0]["status"], "stale")
        self.assertTrue(all(frame["data"]["state"]["generation"] == 0 for frame in frames))
        self.assertGreaterEqual(sent_at[1] - sent_at[0], 0.09)
        self.assertGreaterEqual(sent_at[2] - sent_at[1], 0.09)

    def test_high_frequency_telemetry_cannot_bypass_ws_preview_rate_limit(self):
        app = self.create_app(self.store, device_link_enabled=False, gamepad_preview=self.preview)
        sent_at = []
        generations = []
        outer = self

        def immediate_change(_generation, _timeout):
            outer.store.publish_telemetry("ugctl", {"armed": 0})
            generation, snapshot = outer.store.read()
            return generation, snapshot, True

        class FakeWebSocket:
            async def accept(self):
                pass

            async def send_json(self, frame):
                sent_at.append(time.monotonic())
                generations.append(frame["data"]["state"]["generation"])
                if len(sent_at) == 3:
                    raise outer.WebSocketDisconnect()

        async def exercise():
            async with app.router.lifespan_context(app):
                with patch.object(self.store, "wait_for_change", side_effect=immediate_change):
                    await self.endpoint(app, "/api/v1/ws/state")(FakeWebSocket())

        asyncio.run(exercise())
        self.assertEqual(generations, [0, 1, 2])
        self.assertGreaterEqual(sent_at[1] - sent_at[0], 0.09)
        self.assertGreaterEqual(sent_at[2] - sent_at[1], 0.09)


if __name__ == "__main__":
    unittest.main()
