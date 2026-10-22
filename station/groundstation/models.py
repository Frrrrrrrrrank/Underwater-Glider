"""URGC 对外状态模型。

这里刻意不依赖 FastAPI/Pydantic，使通信线程、命令行界面和 Web 服务以后都能
共享同一份状态定义，而不会把业务核心绑死在某一种前端框架上。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple

from .telemetry_quality import evaluate_telemetry_quality


SCHEMA_VERSION = 1
DEFAULT_REQUIRED_TOPICS = ("ugekf", "ugctl", "ugwrm")


def utc_now_iso() -> str:
    """返回带 UTC 时区的 ISO-8601 时间，供 API 和日志统一使用。"""

    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class LinkStatus(str, Enum):
    """核心与滑翔机之间的链路状态，而不是浏览器与核心的连接状态。"""

    STOPPED = "stopped"
    WAITING = "waiting"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    ERROR = "error"


@dataclass(frozen=True)
class BackendInfo:
    """常驻 URGC 核心自身的信息。"""

    started_at: str = field(default_factory=utc_now_iso)
    name: str = "UnderwaterRobotGroundControl"
    version: str = "0.1.0"
    mode: str = "read_only_telemetry"

    def to_public_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "mode": self.mode,
            "started_at": self.started_at,
        }


@dataclass(frozen=True)
class DeviceLinkState:
    """单一核心拥有的设备链路状态。

    第一阶段尚未接管 contcp_t.py 的 socket，所以初始状态必须是 STOPPED，
    不能因为 Web API 正常就假装设备已经在线。
    """

    transport: str = "ethernet_tcp"
    status: LinkStatus = LinkStatus.STOPPED
    peer: Optional[str] = None
    session_id: Optional[str] = None
    connected_at: Optional[str] = None
    last_rx_at: Optional[str] = None
    last_tx_at: Optional[str] = None
    last_error: Optional[str] = None

    def to_public_dict(self) -> Dict[str, Any]:
        return {
            "transport": self.transport,
            "status": self.status.value,
            "peer": self.peer,
            "session_id": self.session_id,
            "connected_at": self.connected_at,
            "last_rx_at": self.last_rx_at,
            "last_tx_at": self.last_tx_at,
            "last_error": self.last_error,
        }


@dataclass(frozen=True)
class TelemetrySample:
    """一类遥测的最新快照及其主机侧新鲜度元数据。"""

    source: str
    received_at: str
    received_monotonic_s: float
    stale_after_ms: int
    data: Mapping[str, Any]
    sampled_at: Optional[str] = None
    device_tick_ms: Optional[int] = None
    session_id: Optional[str] = None
    topic: str = ""

    def to_public_dict(self, now_monotonic_s: float) -> Dict[str, Any]:
        age_ms = max(0, int((now_monotonic_s - self.received_monotonic_s) * 1000.0))
        data, invalid_fields, quality = evaluate_telemetry_quality(self.topic, self.data)
        return {
            "source": self.source,
            "sampled_at": self.sampled_at,
            "received_at": self.received_at,
            "device_tick_ms": self.device_tick_ms,
            "session_id": self.session_id,
            "age_ms": age_ms,
            "stale_after_ms": self.stale_after_ms,
            "fresh": age_ms <= self.stale_after_ms,
            "data": data,
            "invalid_fields": invalid_fields,
            "quality": quality,
        }


@dataclass(frozen=True)
class CapabilityState:
    """当前阶段真实具备的能力；未实现的能力必须显式为 false。"""

    observe: bool = True
    realtime_control: bool = False
    parameter_write: bool = False
    autonomous_mission: bool = False
    intermittent_command: bool = False

    def to_public_dict(self) -> Dict[str, bool]:
        return {
            "observe": self.observe,
            "realtime_control": self.realtime_control,
            "parameter_write": self.parameter_write,
            "autonomous_mission": self.autonomous_mission,
            "intermittent_command": self.intermittent_command,
        }


@dataclass(frozen=True)
class GroundStationSnapshot:
    """REST 与 WebSocket 共用的完整只读快照。"""

    backend: BackendInfo
    device_link: DeviceLinkState
    telemetry: Mapping[str, TelemetrySample]
    capabilities: CapabilityState = field(default_factory=CapabilityState)
    required_topics: Tuple[str, ...] = DEFAULT_REQUIRED_TOPICS

    def to_public_dict(self, generation: int, now_monotonic_s: float) -> Dict[str, Any]:
        telemetry = {
            name: sample.to_public_dict(now_monotonic_s)
            for name, sample in sorted(self.telemetry.items())
        }
        stale_topics = [name for name, sample in telemetry.items() if not sample["fresh"]]
        missing_topics = [name for name in self.required_topics if name not in telemetry]
        # 额外诊断主题不阻塞默认订阅；无指定主题时至少要有一条实际遥测并保持新鲜。
        freshness_topics = self.required_topics or tuple(telemetry)
        required_stale = any(name in stale_topics for name in freshness_topics)

        if self.device_link.status is not LinkStatus.CONNECTED:
            overall_fresh = False
            stale_reason = "device_link_{}".format(self.device_link.status.value)
        elif not telemetry:
            overall_fresh = False
            stale_reason = "no_telemetry"
        elif missing_topics:
            overall_fresh = False
            stale_reason = "missing_telemetry"
        elif required_stale:
            overall_fresh = False
            stale_reason = "stale_telemetry"
        else:
            overall_fresh = True
            stale_reason = None

        return {
            "schema_version": SCHEMA_VERSION,
            "generated_at": utc_now_iso(),
            "backend": self.backend.to_public_dict(),
            "device_link": self.device_link.to_public_dict(),
            "telemetry": telemetry,
            "state": {
                "generation": generation,
                "overall_fresh": overall_fresh,
                "stale_reason": stale_reason,
                "stale_topics": stale_topics,
                "required_topics": list(self.required_topics),
                "missing_topics": missing_topics,
                "command_capability": "disabled",
                "capabilities": self.capabilities.to_public_dict(),
            },
        }
