"""线程安全的 URGC 最新状态仓库。"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from threading import Condition, RLock
from time import monotonic
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Tuple

from .models import (
    BackendInfo,
    DEFAULT_REQUIRED_TOPICS,
    DeviceLinkState,
    GroundStationSnapshot,
    LinkStatus,
    TelemetrySample,
    utc_now_iso,
)


SnapshotResult = Tuple[int, Dict[str, Any]]
WaitResult = Tuple[int, Dict[str, Any], bool]


class StateStore:
    """保存最新状态，并为多个只读前端提供一致快照。

    Condition 同时承担互斥锁和变更通知：未来唯一 TCP 接收线程写入状态时，
    REST/WS 读取者不会看到“字段只更新了一半”的撕裂快照。
    """

    def __init__(
        self,
        clock: Callable[[], float] = monotonic,
        *,
        required_topics: Iterable[str] = DEFAULT_REQUIRED_TOPICS,
    ) -> None:
        # 订阅要求是会话配置；不能只凭先到的一条主题就宣称所有遥测齐全。
        if isinstance(required_topics, str):
            raise ValueError("required_topics 必须是主题名称集合")
        topics = tuple(required_topics)
        if any(not isinstance(topic, str) or not topic.strip() for topic in topics):
            raise ValueError("required_topics 不得含空主题或非字符串")
        self._required_topics = tuple(dict.fromkeys(topic.strip() for topic in topics))
        self._clock = clock
        self._condition = Condition(RLock())
        self._generation = 0
        self._backend = BackendInfo()
        self._device_link = DeviceLinkState()
        self._telemetry: Dict[str, TelemetrySample] = {}

    def _snapshot_locked(self) -> GroundStationSnapshot:
        return GroundStationSnapshot(
            backend=self._backend,
            device_link=self._device_link,
            telemetry=dict(self._telemetry),
            required_topics=self._required_topics,
        )

    def _public_snapshot_locked(self) -> Dict[str, Any]:
        return self._snapshot_locked().to_public_dict(self._generation, self._clock())

    def _publish_locked(self) -> None:
        self._generation += 1
        self._condition.notify_all()

    def read(self) -> SnapshotResult:
        """原子读取当前 generation 与完整公开快照。"""

        with self._condition:
            return self._generation, self._public_snapshot_locked()

    def wait_for_change(self, after_generation: int, timeout_s: float) -> WaitResult:
        """等待 generation 前进；超时用于让 WebSocket 发送心跳。"""

        if timeout_s < 0.0:
            raise ValueError("timeout_s 不得为负数")
        with self._condition:
            changed = self._condition.wait_for(
                lambda: self._generation > after_generation,
                timeout=timeout_s,
            )
            return self._generation, self._public_snapshot_locked(), bool(changed)

    def set_link(
        self,
        status: LinkStatus,
        *,
        peer: Optional[str] = None,
        session_id: Optional[str] = None,
        last_error: Optional[str] = None,
    ) -> int:
        """发布设备链路状态；真正的 ConnectionManager 后续只调用此接口。"""

        if not isinstance(status, LinkStatus):
            status = LinkStatus(status)

        now = utc_now_iso()
        with self._condition:
            connected_at = self._device_link.connected_at
            if status is LinkStatus.CONNECTED and self._device_link.status is not LinkStatus.CONNECTED:
                connected_at = now
            elif status is LinkStatus.STOPPED:
                connected_at = None

            self._device_link = replace(
                self._device_link,
                status=status,
                peer=peer,
                session_id=session_id,
                connected_at=connected_at,
                last_error=last_error,
            )
            self._publish_locked()
            return self._generation

    def publish_telemetry(
        self,
        topic: str,
        data: Mapping[str, Any],
        *,
        source: str = "ethernet_tcp",
        sampled_at: Optional[str] = None,
        device_tick_ms: Optional[int] = None,
        session_id: Optional[str] = None,
        stale_after_ms: int = 500,
    ) -> int:
        """原子替换某一主题的最新遥测，并记录主机接收时刻。"""

        topic = topic.strip()
        if not topic:
            raise ValueError("topic 不得为空")
        if stale_after_ms <= 0:
            raise ValueError("stale_after_ms 必须大于 0")

        received_at = utc_now_iso()
        with self._condition:
            previous = self._telemetry.get(topic)
            if (topic == "ugps" and previous is not None and
                    previous.session_id == session_id and
                    (previous.data.get("seq"), previous.data.get("t_ms")) ==
                    (data.get("seq"), data.get("t_ms"))):
                # 同一 GNSS 诊断快照重复到达不能刷新接收年龄。
                return self._generation
            self._telemetry[topic] = TelemetrySample(
                topic=topic,
                source=source,
                sampled_at=sampled_at,
                received_at=received_at,
                received_monotonic_s=self._clock(),
                stale_after_ms=stale_after_ms,
                device_tick_ms=device_tick_ms,
                session_id=session_id,
                data=deepcopy(dict(data)),
            )
            self._device_link = replace(self._device_link, last_rx_at=received_at)
            self._publish_locked()
            return self._generation

    def mark_tx(self) -> int:
        """记录最近一次 socket 写入时刻；它不等同于板端 ACK。"""

        with self._condition:
            self._device_link = replace(self._device_link, last_tx_at=utc_now_iso())
            self._publish_locked()
            return self._generation

    def clear_session_telemetry(self) -> int:
        """新连接会话开始前清空旧遥测，防止把上一会话误判为在线数据。"""

        with self._condition:
            self._telemetry.clear()
            self._publish_locked()
            return self._generation
