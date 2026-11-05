"""URGC FastAPI 只读入口。"""

from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .ethernet_link import EthernetRealtimeLink
from .gamepad_preview import GamepadPreview
from .models import SCHEMA_VERSION, utc_now_iso
from .state_store import StateStore
from .session_recorder import SessionRecorder, disabled_recording_snapshot


def _environment_flag(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def create_app(
    store: Optional[StateStore] = None,
    *,
    link_manager: Optional[EthernetRealtimeLink] = None,
    device_link_enabled: Optional[bool] = None,
    gamepad_preview: Optional[GamepadPreview] = None,
) -> FastAPI:
    """创建可注入 StateStore 的应用，便于后续测试与核心集成。"""

    station_store = store or StateStore()
    input_preview = gamepad_preview if gamepad_preview is not None else GamepadPreview(
        enabled=_environment_flag("URGC_GAMEPAD_ENABLED", True)
    )
    if device_link_enabled is None:
        device_link_enabled = _environment_flag("URGC_DEVICE_LINK_ENABLED", True)
    if link_manager is None and device_link_enabled:
        link_manager = EthernetRealtimeLink(
            station_store,
            host=os.getenv("URGC_DEVICE_HOST", "192.168.57.100"),
            port=int(os.getenv("URGC_DEVICE_PORT", "8800")),
            recorder=(
                SessionRecorder(Path(os.getenv(
                    "URGC_LOG_DIR", str(Path(__file__).resolve().parents[1] / "logs" / "sessions")
                ))) if _environment_flag("URGC_RECORDING_ENABLED", True) else None
            ),
        )

    @asynccontextmanager
    async def lifespan(_application: FastAPI):
        input_preview.start()
        try:
            if link_manager is not None:
                link_manager.start()
            yield
        finally:
            try:
                if link_manager is not None:
                    link_manager.stop()
            finally:
                input_preview.stop()

    app = FastAPI(
        title="UnderwaterRobotGroundControl",
        version="0.1.0",
        description="URGC 只读遥测核心；拥有唯一设备 socket，但不接受控制命令。",
        lifespan=lifespan,
    )
    app.state.station_store = station_store
    app.state.device_link = link_manager
    app.state.gamepad_preview = input_preview

    # 仅允许本机 Vite 开发服务器跨域访问；正式构建由同一 FastAPI 进程托管。
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
        allow_credentials=False,
        allow_methods=["GET"],
        allow_headers=["*"],
    )

    @app.get("/api/v1/health")
    async def health() -> dict:
        generation, snapshot = station_store.read()
        return {
            "status": "ok",
            "schema_version": SCHEMA_VERSION,
            "generated_at": utc_now_iso(),
            "service": "UnderwaterRobotGroundControl",
            "mode": snapshot["backend"]["mode"],
            "generation": generation,
            "device_link_status": snapshot["device_link"]["status"],
            "device_link_enabled": link_manager is not None,
        }

    @app.get("/api/v1/state")
    async def state(response: Response) -> dict:
        response.headers["Cache-Control"] = "no-store"
        _generation, snapshot = station_store.read()
        snapshot["gamepad"] = input_preview.snapshot()
        return snapshot

    @app.get("/api/v1/input/gamepad")
    async def gamepad(response: Response) -> dict:
        response.headers["Cache-Control"] = "no-store"
        return input_preview.snapshot()

    @app.get("/api/v1/recording")
    async def recording() -> dict:
        return link_manager.recording_snapshot() if link_manager else disabled_recording_snapshot()

    @app.websocket("/api/v1/ws/state")
    async def websocket_state(websocket: WebSocket) -> None:
        await websocket.accept()
        generation, snapshot = station_store.read()
        snapshot["gamepad"] = input_preview.snapshot()
        await websocket.send_json({"type": "state.snapshot", "data": snapshot})

        try:
            while True:
                cycle_started = asyncio.get_running_loop().time()
                next_generation, next_snapshot, changed = await asyncio.to_thread(
                    station_store.wait_for_change,
                    generation,
                    0.1,
                )
                # 遥测高频唤醒时也最多 10 Hz 读取预览；不增加遥测 generation。
                await asyncio.sleep(max(0.0, 0.1 - (asyncio.get_running_loop().time() - cycle_started)))
                next_generation, next_snapshot = station_store.read()
                changed = next_generation != generation
                next_snapshot["gamepad"] = input_preview.snapshot()
                if changed:
                    generation = next_generation
                    await websocket.send_json(
                        {"type": "state.snapshot", "data": next_snapshot}
                    )
                else:
                    await websocket.send_json(
                        {
                            "type": "heartbeat",
                            "generated_at": next_snapshot["generated_at"],
                            "generation": generation,
                            "data": next_snapshot,
                        }
                    )
        except WebSocketDisconnect:
            return

    frontend_dist = Path(__file__).resolve().parents[1] / "frontend" / "dist"
    if frontend_dist.is_dir():
        # API/WS 路由必须先注册，最后才把根路径交给单页应用。
        app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")
    else:

        @app.get("/", include_in_schema=False)
        async def root() -> dict:
            return {
                "service": "UnderwaterRobotGroundControl",
                "short_name": "URGC",
                "mode": "read_only_telemetry",
                "frontend": "not_built",
                "state_url": "/api/v1/state",
                "docs_url": "/docs",
            }

    return app


app = create_app()
