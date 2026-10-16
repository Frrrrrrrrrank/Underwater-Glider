"""电池遥测校验与主动告警；只消费数据，不发送执行器命令。"""

import re
import time


BATTERY_INIT = 0
BATTERY_NORMAL = 1
BATTERY_WARN = 2
BATTERY_CRITICAL = 3
BATTERY_INVALID = 4
BATTERY_SAMPLE_MAX_AGE_MS = 500
BATTERY_TELEMETRY_TIMEOUT_S = 3.0
BATTERY_STARTUP_GRACE_S = 8.0

UGBAT_FIELDS = (
    "schema", "seq", "t_ms", "adc_raw", "battery_mv", "filtered_mv",
    "valid", "state", "sample_age_ms", "error_count",
)
UGBAT_CSV_HEADER = ("host_unix_ms",) + UGBAT_FIELDS
_FIELD_LIMITS = (1, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFF, 100000, 100000,
                 1, BATTERY_INVALID, 0xFFFFFFFF, 0xFFFFFFFF)
_FRAME_PATTERN = re.compile(r"\$(UGBAT,[0-9,]+)\*([0-9a-fA-F]{2})")


def ugbat_parse(line):
    """严格接收 schema 1 的十个无符号整数字段和 NMEA 异或校验。"""
    if not isinstance(line, str) or len(line) > 256:
        return None
    match = _FRAME_PATTERN.fullmatch(line.rstrip("\r\n"))
    if match is None:
        return None
    body, checksum_text = match.groups()
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    if checksum != int(checksum_text, 16):
        return None
    values = body.split(",")[1:]
    if len(values) != len(UGBAT_FIELDS):
        return None
    if any(not value or len(value) > 10 for value in values):
        return None
    numbers = tuple(int(value) for value in values)
    if numbers[0] != 1 or any(value > limit for value, limit in
                               zip(numbers, _FIELD_LIMITS)):
        return None
    return dict(zip(UGBAT_FIELDS, numbers))


def ugbat_csv_row(row):
    """原始整数毫伏完整落盘，显示时才换算为伏。"""
    return (time.time_ns() // 1_000_000,) + tuple(row[name] for name in UGBAT_FIELDS)


def _console_alert(message):
    # 单次换行输出并刷新，输入提示符、键盘和手柄控制均不需要退出才能看到。
    print("\n" + message, flush=True)


class BatteryAlarmMonitor:
    """由唯一接收线程调用；使用单调时钟限频，不依赖用户输入。"""

    def __init__(self, emit=None, clock=None):
        self._emit = _console_alert if emit is None else emit
        self._clock = time.monotonic if clock is None else clock
        self._started_at = self._clock()
        self._last_frame_at = None
        self._last_key = None
        self._row = None
        self._state = None
        self._last_low_state = None
        self._last_alert_at = None
        self._last_unavailable_at = None
        self._fault_reason = None
        self._disconnected = False

    def _unavailable(self, reason, now):
        self._state = None
        self._fault_reason = reason
        if (self._last_unavailable_at is not None and
                now - self._last_unavailable_at < 30.0):
            return
        previous = ""
        if self._last_low_state is not None:
            previous = "；此前低压告警尚未确认解除"
        self._emit(f"[电池监测不可用] {reason}；当前电池状态未知{previous}。")
        self._last_unavailable_at = now

    def ingest(self, row):
        """仅传入校验通过的帧；故障和过期绝不当作低压解除。"""
        if self._disconnected:
            return
        now = self._clock()
        key = (row["seq"], row["t_ms"])
        if key == self._last_key:
            # 重复帧不能刷新接收有效期，避免重复播放旧样本维持正常状态。
            self.poll()
            return
        self._last_key = key
        self._last_frame_at = now
        self._row = dict(row)
        if row["state"] == BATTERY_INIT:
            self._state = None
            self._fault_reason = "板端尚在启动或电压判定中"
            if now - self._started_at >= BATTERY_STARTUP_GRACE_S:
                self._unavailable(self._fault_reason, now)
            return
        if not row["valid"] or row["state"] == BATTERY_INVALID:
            self._unavailable("板端 ADC 测量无效", now)
            return
        if row["sample_age_ms"] > BATTERY_SAMPLE_MAX_AGE_MS:
            self._unavailable(f"板端样本过期（{row['sample_age_ms']} ms）", now)
            return

        state = row["state"]
        changed = state != self._state
        was_unavailable = self._fault_reason is not None
        self._fault_reason = None
        self._last_unavailable_at = None
        voltage = row["filtered_mv"] / 1000.0
        if state == BATTERY_NORMAL and changed:
            if self._last_low_state is not None:
                self._emit(f"[电池低压解除] 总压={voltage:.2f} V；新的有效样本已确认恢复正常。")
            elif was_unavailable:
                self._emit(f"[电池监测恢复] 总压={voltage:.2f} V，状态正常。")
            else:
                self._emit(f"[电池] 监测已就绪，总压={voltage:.2f} V，状态正常。")
            self._last_low_state = None
        elif state in (BATTERY_WARN, BATTERY_CRITICAL):
            interval = 10.0 if state == BATTERY_CRITICAL else 30.0
            if changed or self._last_alert_at is None or now - self._last_alert_at >= interval:
                self._low_alert(state, voltage, now, was_unavailable)
            self._last_low_state = state
        self._state = state

    def _low_alert(self, state, voltage, now, was_unavailable=False):
        if state == BATTERY_CRITICAL:
            message = f"[电池严重低压] 总压={voltage:.2f} V；请及时处理低电压。"
        elif self._last_low_state == BATTERY_CRITICAL:
            message = f"[电池低压预警] 总压={voltage:.2f} V；严重低压已降级，但仍处于低压预警。"
        else:
            message = f"[电池低压预警] 总压={voltage:.2f} V；请留意电量并安排充电。"
        if was_unavailable:
            message += " 监测已恢复，低压告警仍然有效。"
        self._emit(message)
        self._last_alert_at = now

    def poll(self):
        """每轮收包或超时均检查，其他遥测持续到达也不能掩盖电池失联。"""
        if self._disconnected:
            return
        now = self._clock()
        if self._last_frame_at is None:
            if now - self._started_at >= BATTERY_STARTUP_GRACE_S:
                self._unavailable("尚未收到电池遥测，请核对固件是否支持 UGBAT 及电池流订阅", now)
            return
        if now - self._last_frame_at > BATTERY_TELEMETRY_TIMEOUT_S:
            self._unavailable("超过 3 秒未收到新的电池遥测", now)
            return
        if self._fault_reason is not None:
            if (self._row["state"] != BATTERY_INIT or
                    now - self._started_at >= BATTERY_STARTUP_GRACE_S):
                self._unavailable(self._fault_reason, now)
            return
        if self._state in (BATTERY_WARN, BATTERY_CRITICAL):
            interval = 10.0 if self._state == BATTERY_CRITICAL else 30.0
            if now - self._last_alert_at >= interval:
                self._low_alert(self._state, self._row["filtered_mv"] / 1000.0, now)

    def disconnect(self):
        """关闭连接只说明监测中断，不能据此宣称电池开关已关闭。"""
        if self._disconnected:
            return
        self._disconnected = True
        previous = "；此前低压告警尚未确认解除" if self._last_low_state is not None else ""
        self._emit("[电池监测中断] 连接已结束，当前电池状态未知，"
                   f"无法区分断电和通信断开{previous}。")
