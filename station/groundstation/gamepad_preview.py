"""URGC 手柄只读预览；采样与意图计算不包含接管、积分或板端发送。"""

from __future__ import annotations

import math
from dataclasses import asdict
from numbers import Integral, Real
from threading import RLock
from time import monotonic
from typing import Callable, Optional

try:
    from ..gamepad_control import (
        GamepadConfig, GamepadService, GamepadSnapshot,
        is_gamepad_neutral, map_gamepad_to_target,
    )
except ImportError:
    from gamepad_control import (
        GamepadConfig, GamepadService, GamepadSnapshot,
        is_gamepad_neutral, map_gamepad_to_target,
    )


_SAMPLE_FIELDS = (
    "left_x", "forward", "left_trigger", "right_trigger", "button_x", "button_y",
    "button_a", "button_b", "dpad_left", "dpad_right", "exit_seq", "loss_seq",
    "rudder_center_seq",
)


def _finite_time(value: object) -> bool:
    try:
        return (not isinstance(value, bool) and isinstance(value, Real)
                and math.isfinite(value) and value >= 0)
    except (ValueError, OverflowError):
        return False


class GamepadPreview:
    """托管唯一采样服务；读取快照不会轮询硬件，也不会改变控制状态。"""

    def __init__(
        self,
        config: Optional[GamepadConfig] = None,
        *,
        enabled: bool = True,
        service_factory: Optional[Callable] = None,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        self.config = config if config is not None else GamepadConfig()
        if not isinstance(self.config, GamepadConfig):
            raise ValueError("config 必须是 GamepadConfig")
        self.enabled = enabled
        self._service_factory = service_factory or GamepadService
        self._clock = clock
        self._lock = RLock()
        self._service = None
        self._status = "stopped" if enabled else "disabled"
        self._error: Optional[str] = None

    def start(self) -> None:
        """仅在应用生命周期启动采样；缺少 XInput 不阻断遥测服务。"""
        with self._lock:
            if not self.enabled or self._service is not None:
                return
            try:
                # 构造 GamepadService 才会加载 XInput，不能放在模块导入阶段。
                self._service = self._service_factory(self.config)
                self._service.start()
                self._status, self._error = "running", None
            except Exception as exc:
                self._status = "error"
                self._error = str(exc) or type(exc).__name__
                if self._service is not None:
                    try:
                        self._service.stop()
                        if not self._service.is_alive():
                            self._service = None
                    except Exception as cleanup_error:
                        self._error += "; 清理失败：" + str(cleanup_error)

    def stop(self) -> None:
        """退出时停止采样；异常线程未退出时保留引用，防止重启第二个线程。"""
        with self._lock:
            self._status = "stopped" if self.enabled else "disabled"
            if self._service is None:
                return
            try:
                self._service.stop()
                if self._service.is_alive():
                    self._status, self._error = "error", "手柄采样线程尚未退出"
                else:
                    self._service = None
            except Exception as exc:
                self._status = "error"
                self._error = "停止手柄采样失败：" + (str(exc) or type(exc).__name__)

    @staticmethod
    def _unavailable_slot(index: int, reason: str) -> dict:
        return {
            "index": index, "connected": False, "fresh": False, "age_ms": None,
            "status": "unavailable", "reason": reason,
            "sample": None, "neutral": None, "intent": None,
        }

    def _slot_snapshot(self, index: int, sample: object, now: float) -> dict:
        slot = self._unavailable_slot(index, "尚无有效手柄输入")
        if not isinstance(sample, GamepadSnapshot):
            slot.update(status="invalid", reason="采样类型无效")
            return slot
        if (isinstance(sample.index, bool) or not isinstance(sample.index, Integral)
                or sample.index != index or not isinstance(sample.connected, bool)):
            slot.update(status="invalid", reason="采样槽位或连接标志无效")
            return slot
        slot["connected"] = sample.connected
        if not sample.connected:
            # 未采到首帧与明确拔出分开；旧值不当作中立输入继续展示。
            waiting = sample.sample_time == 0 and not sample.error
            slot.update(status="waiting" if waiting else "disconnected",
                        reason="等待首次采样" if waiting else (sample.error or "手柄未连接"))
            if (_finite_time(now) and _finite_time(sample.sample_time)
                    and 0 < sample.sample_time <= now):
                age_ms = (now - sample.sample_time) * 1000.0
                slot["age_ms"] = age_ms if math.isfinite(age_ms) else None
            return slot
        if (not _finite_time(now) or not _finite_time(sample.sample_time)
                or sample.sample_time > now):
            slot.update(status="invalid", reason="采样单调时间无效或位于未来")
            return slot
        age_s = now - sample.sample_time
        if not math.isfinite(age_s * 1000.0):
            slot.update(status="invalid", reason="采样年龄无效")
            return slot
        slot["age_ms"] = age_s * 1000.0
        if age_s > self.config.input_timeout_s:
            slot.update(status="stale", reason="手柄输入超过新鲜度阈值")
            return slot
        try:
            for name in ("exit_seq", "loss_seq", "rudder_center_seq"):
                value = getattr(sample, name)
                if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                    raise ValueError(name + " 必须是非负整数")
            # 直接复用 CLI 纯映射：不积分尾舵角度，不模拟 B 键会话抑制状态。
            intent = asdict(map_gamepad_to_target(sample, self.config))
            neutral = is_gamepad_neutral(sample, self.config)
        except (ValueError, TypeError, OverflowError) as exc:
            slot.update(status="invalid", reason=str(exc) or "手柄输入无效")
            return slot
        slot.update(fresh=True, status="fresh", reason=None, neutral=neutral,
                    sample={name: getattr(sample, name) for name in _SAMPLE_FIELDS},
                    intent=intent)
        return slot

    def snapshot(self) -> dict:
        """返回四个槽位的独立质量；失效输入为 null，不用假零表示松开。"""
        with self._lock:
            status, error = self._status, self._error
            service = self._service
            if status == "running":
                try:
                    if service is None or not service.is_alive():
                        status, error = "error", "手柄采样线程未运行"
                except Exception as exc:
                    status, error = "error", str(exc) or "手柄采样服务不可用"
            slots = []
            if status == "running":
                # 先复制样本再读时钟，避免采样线程更新导致正常样本被判作未来。
                samples = []
                for index in range(4):
                    try:
                        samples.append((service.snapshot(index), None))
                    except Exception as exc:
                        samples.append((None, str(exc) or "手柄快照读取失败"))
                now = self._clock()
                for index, (sample, failure) in enumerate(samples):
                    slots.append(self._unavailable_slot(index, failure) if failure
                                 else self._slot_snapshot(index, sample, now))
            else:
                reason = error or {"disabled": "手柄预览已禁用", "stopped": "手柄采样尚未启动"}.get(
                    status, "手柄采样不可用")
                slots = [self._unavailable_slot(index, reason) for index in range(4)]
            return {
                "schema_version": 1, "mode": "preview_only", "source": "core_xinput",
                "status": status, "error": error, "config": asdict(self.config), "slots": slots,
            }
