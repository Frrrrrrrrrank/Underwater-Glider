"""Xbox 手柄采集与控制意图转换；本模块不建立网络连接。"""

import ctypes
import math
import os
import threading
import time
from dataclasses import dataclass, replace
from numbers import Integral, Real


def _finite_number(value, name):
    """拒绝字符串、布尔值和非有限数字，防止错误配置变成运动指令。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("{} 必须是有限数字".format(name))
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError("{} 必须是有限数字".format(name)) from exc
    if not math.isfinite(result):
        raise ValueError("{} 必须是有限数字".format(name))
    return result


def _bounded_number(value, name, lower, upper):
    result = _finite_number(value, name)
    if not lower <= result <= upper:
        raise ValueError("{} 必须在 [{}, {}] 范围内".format(name, lower, upper))
    return result


def _validate_index(index):
    if isinstance(index, bool) or not isinstance(index, Integral) or not 0 <= index <= 3:
        raise ValueError("手柄槽位必须是 0 到 3 的整数")
    return int(index)

#数据定义
@dataclass(frozen=True)
class GamepadConfig:
    """原型参数集中定义；百分率是控制目标，不表示实测推力。"""

    deadzone: float = 0.10
    poll_period_s: float = 0.02
    send_period_s: float = 0.10
    input_timeout_s: float = 0.20
    telemetry_timeout_s: float = 1.0
    prepare_timeout_s: float = 3.0
    max_percent: float = 100
    pitch_rpm: int = 100
    motor1_is_left: bool = True
    motor1_forward_dir: int = 0
    motor2_forward_dir: int = 1
    roll_rpm: int = 300
    trigger_deadzone: float = 0.10
    rudder_rate_deg_s: float = 30.0

    def __post_init__(self):
        deadzone = _finite_number(self.deadzone, "deadzone")
        if not 0 <= deadzone < 1:
            raise ValueError("deadzone 必须在 [0, 1) 范围内")
        trigger_deadzone = _finite_number(self.trigger_deadzone, "trigger_deadzone")
        if not 0 <= trigger_deadzone < 1:
            raise ValueError("trigger_deadzone 必须在 [0, 1) 范围内")
        if _finite_number(self.rudder_rate_deg_s, "rudder_rate_deg_s") <= 0:
            raise ValueError("rudder_rate_deg_s 必须大于 0")
        for name in ("poll_period_s", "send_period_s", "input_timeout_s",
                     "telemetry_timeout_s", "prepare_timeout_s"):
            if _finite_number(getattr(self, name), name) <= 0:
                raise ValueError("{} 必须大于 0".format(name))
        _bounded_number(self.max_percent, "max_percent", 0, 100)
        for name in ("pitch_rpm", "roll_rpm"):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, Integral) or
                    not 1 <= value <= 32767):
                raise ValueError("{} 必须是 1 到 32767 的整数".format(name))
        if not isinstance(self.motor1_is_left, bool):
            raise ValueError("motor1_is_left 必须是布尔值")
        for name in ("motor1_forward_dir", "motor2_forward_dir"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or value not in (0, 1):
                raise ValueError("{} 必须是整数 0 或 1".format(name))


@dataclass(frozen=True)
class GamepadSnapshot:
    """一次输入快照；时间戳记录成功读取，事件序号跨重连保留。"""

    index: int
    connected: bool = False
    left_x: float = 0.0
    forward: float = 0.0
    button_x: bool = False
    button_y: bool = False
    button_a: bool = False
    sample_time: float = 0.0
    exit_seq: int = 0
    loss_seq: int = 0
    error: str = ""
    dpad_left: bool = False
    dpad_right: bool = False
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    button_b: bool = False
    rudder_center_seq: int = 0


@dataclass(frozen=True)
class ControlTarget:
    """与协议无关的意图；尾舵速度指目标角度变化率，不是实测转速。"""

    left_percent: float = 0
    right_percent: float = 0
    pitch_enable: int = 0
    pitch_rpm: int = 0
    roll_enable: int = 0
    roll_rpm: int = 0
    rudder_rate_deg_s: float = 0.0
    rudder_center: bool = False


def apply_axis_deadzone(value, deadzone):
    """每个轴独立去死区，并将死区外的剩余行程线性恢复到满量程。"""
    value = _bounded_number(value, "轴值", -1, 1)
    deadzone = _finite_number(deadzone, "deadzone")
    if not 0 <= deadzone < 1:
        raise ValueError("deadzone 必须在 [0, 1) 范围内")
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def apply_trigger_deadzone(value, deadzone):
    """扳机只能在 [0, 1] 范围，独立去死区后恢复剩余按压行程。"""
    value = _bounded_number(value, "扳机值", 0, 1)
    return apply_axis_deadzone(value, deadzone)


def mix_thrusters(forward, turn, max_percent=100):
    """横向拨杆指定艇首转向；前后都通过降低一侧幅值实现差动。"""
    forward = _bounded_number(forward, "forward", -1, 1)
    turn = _bounded_number(turn, "turn", -1, 1)
    maximum = _bounded_number(max_percent, "max_percent", 0, 100)
    if forward >= 0:
        left = forward * (1.0 - max(-turn, 0.0))
        right = forward * (1.0 - max(turn, 0.0))
    else:
        left = forward * (1.0 - max(turn, 0.0))
        right = forward * (1.0 - max(-turn, 0.0))
    return left * maximum, right * maximum


def _validate_connected_sample(sample, index):
    """读取失败和非法输入不能冒充摇杆回中。"""
    if not isinstance(sample, GamepadSnapshot):
        raise ValueError("采集结果必须是 GamepadSnapshot")
    if _validate_index(sample.index) != index:
        raise ValueError("采集结果的手柄槽位不匹配")
    if not isinstance(sample.connected, bool):
        raise ValueError("connected 必须是布尔值")
    if not sample.connected:
        raise ValueError(sample.error or "手柄未连接")
    _bounded_number(sample.left_x, "left_x", -1, 1)
    _bounded_number(sample.forward, "forward", -1, 1)
    _bounded_number(sample.left_trigger, "left_trigger", 0, 1)
    _bounded_number(sample.right_trigger, "right_trigger", 0, 1)
    for name in ("button_x", "button_y", "button_a", "dpad_left", "dpad_right", "button_b"):
        if not isinstance(getattr(sample, name), bool):
            raise ValueError("{} 必须是布尔值".format(name))
    if (isinstance(sample.rudder_center_seq, bool) or
            not isinstance(sample.rudder_center_seq, Integral) or sample.rudder_center_seq < 0):
        raise ValueError("rudder_center_seq 必须是非负整数")


def is_gamepad_neutral(sample, config):
    """接管前逐项检查松开状态，双扳机相互抵消不能冒充中立。"""
    if not isinstance(config, GamepadConfig):
        raise ValueError("config 必须是 GamepadConfig")
    if not isinstance(sample, GamepadSnapshot):
        raise ValueError("sample 必须是 GamepadSnapshot")
    _validate_connected_sample(sample, _validate_index(sample.index))
    return (abs(sample.left_x) <= config.deadzone and
            abs(sample.forward) <= config.deadzone and
            sample.left_trigger <= config.trigger_deadzone and
            sample.right_trigger <= config.trigger_deadzone and
            not any((sample.button_x, sample.button_y, sample.button_a,
                     sample.dpad_left, sample.dpad_right, sample.button_b)))


def map_gamepad_to_target(sample, config):
    """从有效快照生成当前意图；松开不延续上一帧的 pitch/roll 运动。"""
    if not isinstance(config, GamepadConfig):
        raise ValueError("config 必须是 GamepadConfig")
    if not isinstance(sample, GamepadSnapshot):
        raise ValueError("sample 必须是 GamepadSnapshot")
    _validate_connected_sample(sample, _validate_index(sample.index))
    if sample.button_x:
        return ControlTarget()
    forward = apply_axis_deadzone(sample.forward, config.deadzone)
    turn = apply_axis_deadzone(sample.left_x, config.deadzone)
    left, right = mix_thrusters(forward, turn, config.max_percent)
    pitch_rpm = 0
    if sample.button_y != sample.button_a:
        pitch_rpm = -config.pitch_rpm if sample.button_y else config.pitch_rpm
    # 十字键左正右负；同按或全松开时禁用 roll 并请求零转速。
    roll_rpm = 0
    if sample.dpad_left != sample.dpad_right:
        roll_rpm = config.roll_rpm if sample.dpad_left else -config.roll_rpm
    left_trigger = apply_trigger_deadzone(sample.left_trigger, config.trigger_deadzone)
    right_trigger = apply_trigger_deadzone(sample.right_trigger, config.trigger_deadzone)
    rudder_rate = 0.0
    # B 只优先回中尾舵；双扳机有效时停止改变目标，不按差值决定方向。
    if not sample.button_b:
        if right_trigger > 0 and left_trigger == 0:
            rudder_rate = right_trigger * config.rudder_rate_deg_s
        elif left_trigger > 0 and right_trigger == 0:
            rudder_rate = -left_trigger * config.rudder_rate_deg_s
    return ControlTarget(left, right, int(pitch_rpm != 0), pitch_rpm,
                         int(roll_rpm != 0), roll_rpm,
                         rudder_rate_deg_s=rudder_rate, rudder_center=sample.button_b)


class _XInputGamepad(ctypes.Structure):
    """固定宽度字段与 Windows XINPUT_GAMEPAD 二进制布局一致。"""

    _fields_ = [
        ("wButtons", ctypes.c_uint16),
        ("bLeftTrigger", ctypes.c_uint8),
        ("bRightTrigger", ctypes.c_uint8),
        ("sThumbLX", ctypes.c_int16),
        ("sThumbLY", ctypes.c_int16),
        ("sThumbRX", ctypes.c_int16),
        ("sThumbRY", ctypes.c_int16),
    ]


class _XInputState(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint32), ("Gamepad", _XInputGamepad)]


class XInputBackend:
    """只读 Windows XInput；导入模块本身不会加载 DLL 或接触设备。"""

    def __init__(self):
        if os.name != "nt":
            raise OSError("Xbox XInput 采集仅支持 Windows")
        errors = []
        for name in ("xinput1_4.dll", "xinput9_1_0.dll"):
            try:
                library = ctypes.WinDLL(name)
                get_state = library.XInputGetState
                get_state.argtypes = [ctypes.c_uint32, ctypes.POINTER(_XInputState)]
                get_state.restype = ctypes.c_uint32
                self._library = library
                self._get_state = get_state
                break
            except (OSError, AttributeError) as exc:
                errors.append("{}: {}".format(name, exc))
        else:
            raise OSError("无法加载 XInput：" + "; ".join(errors))

    @staticmethod
    def _normalize_axis(value):
        # 正负原始端点不对称，分别归一化后才能同时到达正负 100%。
        return value / (32767.0 if value >= 0 else 32768.0)

    def poll(self, index):
        """返回当前状态；成功采样时间由采集服务统一填写。"""
        index = _validate_index(index)
        state = _XInputState()
        result = self._get_state(index, ctypes.byref(state))
        if result:
            error = "手柄未连接" if result == 1167 else "XInput 读取失败：{}".format(result)
            return GamepadSnapshot(index=index, error=error)
        gamepad = state.Gamepad
        return GamepadSnapshot(
            index=index, connected=True,
            left_x=self._normalize_axis(gamepad.sThumbLX),
            forward=self._normalize_axis(gamepad.sThumbLY),
            button_x=bool(gamepad.wButtons & 0x4000),
            button_y=bool(gamepad.wButtons & 0x8000),
            button_a=bool(gamepad.wButtons & 0x1000),
            dpad_left=bool(gamepad.wButtons & 0x0004),
            dpad_right=bool(gamepad.wButtons & 0x0008),
            left_trigger=gamepad.bLeftTrigger / 255.0,
            right_trigger=gamepad.bRightTrigger / 255.0,
            button_b=bool(gamepad.wButtons & 0x2000),
        )


class GamepadService:
    """采集四个槽位，只发布快照；控制模式自行固定所选槽位。"""

    def __init__(self, config=None, backend=None):
        self.config = config if config is not None else GamepadConfig()
        if not isinstance(self.config, GamepadConfig):
            raise ValueError("config 必须是 GamepadConfig")
        self._backend = backend if backend is not None else XInputBackend()
        self._lock = threading.Lock()
        self._poll_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._samples = {index: GamepadSnapshot(index=index) for index in range(4)}
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        """重复启动不会生成多个采集线程。"""
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, name="gamepad-input", daemon=True)
            self._thread.start()

    def stop(self):
        """请求采集退出；有界等待避免退出流程被异常设备永久阻塞。"""
        with self._lifecycle_lock:
            self._stop_event.set()
            thread = self._thread
        if (thread is not None and thread is not threading.current_thread()
                and thread.ident is not None):
            thread.join(timeout=2.0)

    def is_alive(self):
        thread = self._thread
        return thread is not None and thread.is_alive()

    def snapshot(self, index):
        index = _validate_index(index)
        with self._lock:
            return self._samples[index]

    def connected_indices(self):
        with self._lock:
            return tuple(index for index in range(4) if self._samples[index].connected)

    def poll_once(self):
        """同步采集一轮，供线程及假后端测试复用；不依赖真实手柄。"""
        with self._poll_lock:
            for index in range(4):
                try:
                    sample = self._backend.poll(index)
                    _validate_connected_sample(sample, index)
                    # 输入值保持不变也属于成功采样，不能按 XInput 包号判过期。
                    sample_time = time.monotonic()
                    error = ""
                except Exception as exc:
                    sample = None
                    sample_time = None
                    error = str(exc) or type(exc).__name__
                with self._lock:
                    previous = self._samples[index]
                    if sample is None:
                        # 每段连续失败只记一次；快速重连不能抹掉失连证据。
                        loss_seq = previous.loss_seq + int(previous.connected or not previous.error)
                        updated = GamepadSnapshot(
                            index=index, sample_time=previous.sample_time,
                            exit_seq=previous.exit_seq, loss_seq=loss_seq, error=error,
                            rudder_center_seq=previous.rudder_center_seq,
                        )
                    else:
                        rising_x = sample.button_x and (not previous.connected or not previous.button_x)
                        rising_b = sample.button_b and (not previous.connected or not previous.button_b)
                        updated = replace(
                            sample, sample_time=sample_time,
                            exit_seq=previous.exit_seq + int(rising_x),
                            loss_seq=previous.loss_seq, error="",
                            rudder_center_seq=previous.rudder_center_seq + int(rising_b),
                        )
                    self._samples[index] = updated

    def _run(self):
        next_poll = time.monotonic()
        while not self._stop_event.is_set():
            self.poll_once()
            next_poll += self.config.poll_period_s
            now = time.monotonic()
            if next_poll < now:
                next_poll = now
            self._stop_event.wait(max(0.0, next_poll - now))
