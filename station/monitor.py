#!/usr/bin/env python3
"""第二终端只读面板：只向本机快照桥发送 GET，不接触设备连接或控制命令。"""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from http.client import HTTPException
import json
import math
import os
import shutil
import sys
import time
import unicodedata
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


try:
    from .gps_telemetry import ugps_invalid_fields
except ImportError:
    from gps_telemetry import ugps_invalid_fields


DEFAULT_PORT = 8801
DEFAULT_URL = f"http://127.0.0.1:{DEFAULT_PORT}/snapshot"
REFRESH_SECONDS = 0.2
MAX_SNAPSHOT_BYTES = 256 * 1024

_LINK_NAMES = {
    "connected": "已连接", "waiting": "等待板端", "disconnected": "已断开",
    "stopped": "未启动", "error": "链路错误",
}
_PID_MODES = {0: "OFF", 1: "PITCH_HOLD", 2: "DEPTH_CASCADE"}
_PID_PHASES = {0: "下潜", 1: "上浮"}
_PID_FAULTS = {
    0: "无", 1: "未ARM", 2: "命令超时", 3: "参数无效", 4: "参数CRC不符",
    5: "EKF无效", 6: "深度过期", 7: "命令无效", 8: "算法拒绝", 9: "位移无效",
}
_CORE_FAULTS = {
    0: "无", 1: "未ARM", 2: "EKF无效", 3: "配置无效", 4: "命令无效",
    5: "周期无效", 6: "深度无效", 7: "位移无效",
}
_PITCH_STATES = {0: "未校准", 1: "趋近机头", 2: "退出机头", 3: "完成", 4: "失败"}
_ROLL_STATES = {
    0: "未校准", 1: "趋近PB3", 2: "退出PB3", 3: "趋近PB5",
    4: "退出PB5", 5: "返回50%", 6: "完成", 7: "失败",
}
_CALIB_STATES = {0: "未校准", 1: "校准中", 2: "双轴完成", 3: "失败", 4: "已取消"}
_CALIB_FAILURES = {0: "无", 1: "pitch失败", 2: "roll失败", 3: "已取消", 4: "PID接管"}
_RDSS_RESULTS = {
    0: "空闲", 1: "待发", 2: "等待模块回执", 3: "模块确认",
    4: "模块拒绝", 5: "回执超时", 6: "UART错误",
}
_INVALID_REASONS = {
    "non_finite": "非有限数", "ekf_not_valid": "EKF无效",
    "worm_feedback_stale": "反馈过期", "worm_feedback_unknown": "反馈状态未知",
    "worm_span_invalid": "跨度无效", "worm_span_unknown": "跨度状态未知",
    "worm_reference_unavailable": "无有效参考", "roll_reference_invalid": "未标定",
    "roll_reference_unknown": "参考未知", "battery_invalid": "电池数据无效",
    "battery_sample_stale": "采样过期", "battery_sample_age_unknown": "采样年龄未知",
    "local_fix_unavailable": "无缓存定位", "target_coordinate_unavailable": "目标无效",
    "coordinate_out_of_range": "坐标无效",
    "gps_origin_unavailable": "无地理原点", "gps_not_anchored": "尚未锚定",
    "gps_covariance_unavailable": "协方差无效", "gps_measurement_unavailable": "无有效测量",
    "gps_innovation_unavailable": "无有效创新", "gps_never_fused": "尚未融合",
    "gps_age_unknown": "年龄未知",
}


def _mapping(value):
    return value if isinstance(value, Mapping) else {}


def _number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _missing(reason):
    return f"--（{reason}）"


def _rudder_lines(snapshot, error):
    """显示最近成功发送的历史指令，时效与连接状态不等于执行确认。"""
    link = _mapping(_mapping(snapshot).get("device_link"))
    command = _mapping(_mapping(_mapping(snapshot).get("commands")).get("rudder"))
    angle, age = command.get("angle_deg"), command.get("age_ms")
    reason = ("接口不可用" if error else "尚无成功发送记录" if not command else
              "旧连接记录" if command.get("session_id") != link.get("session_id") else
              "数值无效" if not _number(angle) or abs(angle) > 45 else "")
    if reason:
        value, detail = _missing(reason), "来源/时间：--"
    else:
        status = ("板端断开，仅历史记录" if link.get("status") != "connected" else
                  "最近发送" if command.get("fresh") is True else "发送记录超龄")
        authority = "已发释放请求" if command.get("eth_control_flag") == 0 else "直控请求"
        value = f"{angle:+.2f}°（{status}；{authority}）"
        age_text = f"{age:.0f}ms" if _number(age) and age >= 0 else "未知"
        detail = (f"来源：{command.get('source') or '未知'}/{command.get('phase') or '--'}  "
                  f"发送：{command.get('sent_at') or '--'}  年龄：{age_text}")
    return [f"尾舵上位机指令：{value}", f"      {detail}",
            "      非板端执行确认/实测角度；commands.csv 记录发送角度和 success。"]


class _Topic:
    """按字段判断有效性，不能因为 pitch 无效就抹掉独立有效的 roll。"""

    def __init__(self, snapshot, name, error):
        self.link = _mapping(_mapping(snapshot).get("device_link"))
        self.sample = _mapping(_mapping(_mapping(snapshot).get("telemetry")).get(name))
        self.data = _mapping(self.sample.get("data"))
        self.invalid = _mapping(self.sample.get("invalid_fields"))
        if error or not isinstance(snapshot, Mapping):
            self.reason = "接口不可用"
        elif self.link.get("status") != "connected":
            self.reason = "板端未连接"
        elif not self.sample:
            self.reason = "未收到"
        elif self.sample.get("fresh") is not True:
            self.reason = "数据过期"
        elif (self.link.get("session_id") is not None and
              self.sample.get("session_id") != self.link.get("session_id")):
            self.reason = "旧连接数据"
        else:
            self.reason = ""

    def field(self, name, formatter=str, gate=""):
        reason = self.reason or gate
        if not reason and name in self.invalid:
            invalid = self.invalid[name]
            reason = _INVALID_REASONS.get(invalid, "字段无效") if isinstance(invalid, str) else "字段无效"
        value = self.data.get(name)
        if not reason:
            reason = "未上报" if value is None else ""
        if not reason and not _number(value):
            reason = "数值无效"
        if reason:
            return _missing(reason)
        return formatter(value)

    def enum(self, name, choices, gate=""):
        def label(value):
            if int(value) != value:
                return _missing("枚举无效")
            return choices.get(int(value), f"未知({int(value)})")
        return self.field(name, label, gate)

    def age(self):
        if self.reason:
            return self.reason
        age = self.sample.get("age_ms")
        quality = _mapping(self.sample.get("quality")).get("status")
        label = {"valid": "有效", "invalid": "部分无效", "unknown": "质量未知"}.get(quality, "质量未知")
        return f"{int(age)}ms/{label}" if _number(age) and age >= 0 else f"年龄未知/{label}"



_GPS_STATES = {0: "等待定位", 1: "质量观察", 2: "融合中", 3: "失锁推算",
               4: "重新捕获", 5: "等待EKF"}
_GPS_REASONS = {0: "无", 1: "输入无效", 2: "重复", 3: "时间倒退", 4: "过期",
                5: "未就绪", 6: "质量观察", 7: "跳点", 8: "创新超限",
                9: "数值异常", 10: "水平重置", 11: "IMU过期"}


def _gps_lines(snapshot, error):
    """分别显示自身观测、融合结果和参考校验，不把新诊断帧当新定位。"""
    gps = _Topic(snapshot, "ugps", error)
    if not gps.sample:
        reason = gps.reason if gps.reason != "未收到" else "未上报；旧固件可能不支持"
        return ["GPS诊断：" + _missing(reason)]
    gps.invalid = dict(gps.invalid)
    for field, reason in ugps_invalid_fields(gps.data).items():
        gps.invalid.setdefault(field, reason)

    def bit(index):
        return gps.field("flags", lambda flags: "是" if int(flags) & (1 << index) else "否")

    def pair(north, east):
        return (gps.field(north, lambda n: f"{n / 1000:+.3f}") + ", " +
                gps.field(east, lambda n: f"{n / 1000:+.3f}"))

    latitude = gps.field("lat_e7", lambda n: f"{n / 1e7:+.7f}°")
    longitude = gps.field("lon_e7", lambda n: f"{n / 1e7:+.7f}°")
    reference = gps.field("flags", lambda flags:
                          "已标记验证（非导航就绪）" if int(flags) & 8 else
                          "安装杆臂/真北未实测确认，非导航就绪")
    return [
        f"GPS融合  状态：{gps.enum('state', _GPS_STATES)}  原因：{gps.enum('reason', _GPS_REASONS)}  辅助有效：{bit(2)}",
        f"自身定位（纬,经）：{latitude}, {longitude}  样本：{gps.field('sample_seq')}",
        f"      接收龄：{gps.field('rx_age_ms', lambda n: f'{n}ms')}  融合龄：{gps.field('fuse_age_ms', lambda n: f'{n}ms')}  原点ID：{gps.field('origin_id')}",
        f"      NE测量(m)：{pair('meas_n_mm', 'meas_e_mm')}  NE估计(m)：{pair('est_n_mm', 'est_e_mm')}",
        f"      NE创新(m)：{pair('innov_n_mm', 'innov_e_mm')}  NIS：{gps.field('nis_milli', lambda n: f'{n / 1000:.3f}')}",
        f"      R配置(m²)：{gps.field('r_var_cm2', lambda n: f'{n / 10000:.4f}')}  Pnn/ne/ee(m²)："
        + "/".join(gps.field(name, lambda n: f"{n / 10000:.4f}") for name in ('pnn_cm2', 'pne_cm2', 'pee_cm2')),
        f"      接受/拒绝/重置：{gps.field('accept_count')}/{gps.field('reject_count')}/{gps.field('reset_count')}  "
        f"重复/丢样/输入拒绝：{gps.field('duplicate_count')}/{gps.field('drop_count')}/{gps.field('input_reject_count')}",
        f"      参考：{reference}",
        "      接收与融合年龄分开；R为配置方差，不是接收机报告精度。",
    ]


def display_width(text):
    """按常见终端的中文双列规则计宽；组合符号不单独占列。"""
    return sum(0 if unicodedata.combining(ch) else
               2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
               for ch in text)


def _fit_line(text, width):
    # 接口错误等文本也不得向终端注入控制字符或移动光标。
    text = "".join(ch if not unicodedata.category(ch).startswith("C") else "?" for ch in text)
    if display_width(text) <= width:
        return text
    result = []
    columns = 0
    for ch in text:
        size = display_width(ch)
        if columns + size > max(0, width - 1):
            break
        result.append(ch)
        columns += size
    return "".join(result) + "…"


def render_panel(snapshot, *, width=100, endpoint=DEFAULT_URL, error=None):
    """将 GroundStationSnapshot 字典渲染为纯文本，不联网、不读时钟、不产生 ANSI。"""
    width = max(1, int(width))
    ekf = _Topic(snapshot, "ugekf", error)
    ctl = _Topic(snapshot, "ugctl", error)
    wrm = _Topic(snapshot, "ugwrm", error)
    bat = _Topic(snapshot, "ugbat", error)
    rdss = _Topic(snapshot, "ugrds", error)
    link = _mapping(_mapping(snapshot).get("device_link"))
    link_status = _LINK_NAMES.get(link.get("status"), "未知")
    interface_status = f"不可用：{error}" if error else ("正常" if isinstance(snapshot, Mapping) else "不可用")

    ref = wrm.data.get("ref_kind")
    pitch_gate = ("反馈无效" if wrm.data.get("data_fresh") != 1 else
                  "跨度无效" if wrm.data.get("span_valid") != 1 else
                  "无有效参考" if ref not in (1, 2) else "")
    pitch_label = "观测范围/未校准" if ref == 1 else "校准行程"
    pitch = wrm.field("stroke_percent", lambda n: f"{n:.2f}%（{pitch_label}）", pitch_gate)
    old_wrm = wrm.data.get("schema") == 2
    roll_gate = "旧协议未上报" if old_wrm else (
        "未标定" if wrm.data.get("roll_valid") == 0 else
        "参考未知" if wrm.data.get("roll_valid") != 1 else "")
    roll = wrm.field("roll_percent", lambda n: f"{n:.2f}%（脉冲估算）", roll_gate)

    flags = ekf.data.get("fl")
    ekf_gate = "" if _number(flags) and int(flags) == flags and int(flags) & 2 else "EKF无效"
    angle = lambda name: ekf.field(name, lambda n: f"{n:+.2f}°", ekf_gate)
    depth = ekf.field("depth", lambda n: f"{n:.3f} m", ekf_gate)
    calib_gate = "旧协议未上报" if old_wrm else ""
    calib = wrm.enum("calib_state", _CALIB_STATES, calib_gate)
    hold = wrm.enum("calib_hold", {0: "否", 1: "校准接管" if wrm.data.get("calib_state") == 1 else "等待中立"}, calib_gate)
    calib_fail = wrm.enum("calib_fail", _CALIB_FAILURES, calib_gate)

    def limit(bit):
        def level(value):
            if int(value) != value or not 0 <= value <= 15:
                return _missing("限位位图无效")
            return "触发" if (int(value) & (1 << bit)) == 0 else "释放"
        return wrm.field("limit_bits", level)

    sample_age = bat.data.get("sample_age_ms")
    bat_gate = ("数据无效" if bat.data.get("valid") != 1 else
                "采样年龄未知" if not _number(sample_age) or sample_age < 0 else
                "采样过期" if sample_age > 500 else "")
    voltage = bat.field("filtered_mv", lambda n: f"{n / 1000:.3f} V", bat_gate)
    # 电压和有效性共用同一条件；板端 valid=1 但采样过期时不能仍显示有效。
    battery_valid = bat.field("valid", lambda n: "是" if (
        n == 1 and not bat_gate and "filtered_mv" not in bat.invalid and
        _number(bat.data.get("filtered_mv"))) else "否")
    pid_running = ctl.data.get("mode") in (1, 2) and ctl.data.get("armed") == 1
    phase_gate = "" if (pid_running and ctl.data.get("mode") == 2 and
                         ctl.data.get("output_valid") == 1) else "未运行"
    pitch_sp = ctl.field("pitch_sp_deg", lambda n: f"{n:+.2f}°",
                         "未运行" if not pid_running else
                         "" if ctl.data.get("output_valid") == 1 else "算法输出无效")
    crc = ctl.field("active_crc", lambda n: (
        f"0x{int(n):08X}" if int(n) == n and 0 <= n <= 0xFFFFFFFF else _missing("CRC无效")))

    def coordinate(prefix, flag, reason):
        gate = "" if rdss.data.get(flag) == 1 else reason

        def degrees(limit):
            return lambda n: (f"{n / 1e7:+.7f}°" if int(n) == n and -limit <= n <= limit
                              else _missing("坐标无效"))

        return (rdss.field(prefix + "_latitude_deg_e7", degrees(900_000_000), gate) + ", " +
                rdss.field(prefix + "_longitude_deg_e7", degrees(1_800_000_000), gate))

    yes_no = {0: "否", 1: "是"}

    lines = [
        "水下滑翔机 · 只读监视   刷新约5Hz",
        f"接口 {endpoint}  {interface_status}",
        f"板端：{link_status if not error else '未知'}  （本窗口不发送控制）",
        *_rudder_lines(snapshot, error),
        f"轴位置  Pitch：{pitch}",
        f"        Roll ：{roll}",
        f"校准：{calib}  保持：{hold}  原因：{calib_fail}",
        f"阶段  Pitch：{wrm.enum('home_state', _PITCH_STATES)}  Roll：{wrm.enum('roll_home_state', _ROLL_STATES, calib_gate)}",
        f"限位  PB3 roll 0%：{limit(0)}   PB4 pitch头：{limit(1)}",
        f"      PB5 roll100%：{limit(2)}   PD7 pitch尾：{limit(3)}",
        f"EKF   Pitch：{angle('pitch_deg')}   Roll：{angle('roll_deg')}",
        f"      Yaw：{angle('yaw_deg')}   Depth：{depth}（向下为正）",
        f"电池  滤波电压：{voltage}  有效：{battery_valid}",
        f"北斗  信号：{rdss.enum('signal_good', {0: '弱/无', 1: '良好'})}  就绪：{rdss.enum('ready_to_send', yes_no)}  结果：{rdss.enum('last_result', _RDSS_RESULTS)}",
        f"      待发：{rdss.enum('pending', yes_no)}  等回执：{rdss.enum('waiting_ack', yes_no)}  模块确认：{rdss.field('sent_count')}  SEND OK：{rdss.field('send_ok_count')}",
        f"      尝试：{rdss.field('attempt_count')}  拒绝：{rdss.field('reject_count')}  超时：{rdss.field('timeout_count')}  UART失败：{rdss.field('uart_fail_count')}",
        f"      卡号已知：{rdss.enum('card_id_known', yes_no)}  不匹配：{rdss.enum('card_mismatch', yes_no)}  目标接收：{rdss.field('goto_received_count')}",
        f"缓存定位（非实时）：{coordinate('local', 'has_cached_fix', '无缓存定位')}",
        f"目标坐标（纬,经）：{coordinate('target', 'target_valid', '目标无效')}",
        "北斗模块确认不代表对端送达；缓存定位年龄未上报。",
        *_gps_lines(snapshot, error),
        f"PID   模式：{ctl.enum('mode', _PID_MODES)}  ARM：{ctl.enum('armed', {0: '否', 1: '是'})}  阶段：{ctl.enum('phase', _PID_PHASES, phase_gate)}",
        f"      故障：{ctl.enum('fault', _PID_FAULTS)}  算法故障：{ctl.enum('core_fault', _CORE_FAULTS)}",
        f"PID请求 深度：{ctl.field('target_depth', lambda n: f'{n:.3f} m')}  Pitch：{ctl.field('target_pitch_deg', lambda n: f'{n:+.2f}°')}",
        f"PID算法 Pitch设定：{pitch_sp}  （PID状态不代表当前执行来源）",
        f"PID参数 有效：{ctl.enum('pid_valid', {0: '否', 1: '是'})}  active_crc：{crc}",
        "PID 整套参数：当前固件未上报（Kp/Ki/Kd/限幅等）",
        "左右实际 PWM：当前固件未上报（有方向 % / μs）",
        f"数据 EKF：{ekf.age()}  CTL：{ctl.age()}",
        f"     WRM：{wrm.age()}  RDSS：{rdss.age()}",
        "限位0=触发/1=释放。Ctrl+C退出监视。",
    ]
    experiment = _mapping(_mapping(snapshot).get("experiment"))
    if experiment:
        raw_status = experiment.get("status")
        if not isinstance(raw_status, str):
            raw_status = "未知"
        status = {
            "recording": "记录中", "active": "记录中", "idle": "未开始",
            "stopped": "已停止", "closed": "已结束", "error": "记录失败",
            "degraded": "记录不完整", "starting": "正在开始",
        }.get(raw_status, raw_status)
        dropped = experiment.get("dropped_records")
        dropped_text = str(dropped) if type(dropped) is int and dropped >= 0 else "未知"
        if error:
            status, dropped_text = "接口不可用", "未知"
        # 记录状态放在顶部，小终端也能看见；目录仅供定位本机实验文件。
        lines[3:3] = [f"实验记录：{status}  丢记录：{dropped_text}",
                      f"记录目录：{experiment.get('directory') or '--'}"]
        if experiment.get("last_error"):
            lines.insert(5, f"记录错误：{experiment['last_error']}")
    return "\n".join(_fit_line(line, width) for line in lines)


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("本机快照接口不允许重定向")


def fetch_snapshot(url=DEFAULT_URL, timeout=0.4):
    """仅 GET 本机 /snapshot；禁用代理和重定向，不可能改连设备地址。"""
    parsed = urlsplit(url)
    if (parsed.scheme != "http" or parsed.hostname != "127.0.0.1" or
            parsed.path != "/snapshot" or parsed.query or parsed.fragment or
            parsed.username is not None or parsed.password is not None):
        raise ValueError("仅允许 http://127.0.0.1:<port>/snapshot")
    if parsed.port is not None and not 1 <= parsed.port <= 65535:
        raise ValueError("端口必须为1..65535")
    opener = build_opener(ProxyHandler({}), _NoRedirect())
    request = Request(url, headers={"Accept": "application/json"}, method="GET")
    with opener.open(request, timeout=timeout) as response:
        payload = response.read(MAX_SNAPSHOT_BYTES + 1)
    if len(payload) > MAX_SNAPSHOT_BYTES:
        raise ValueError("快照过大")
    snapshot = json.loads(payload.decode("utf-8"))
    if (not isinstance(snapshot, dict) or snapshot.get("schema_version") != 1 or
            not isinstance(snapshot.get("device_link"), dict) or
            not isinstance(snapshot.get("telemetry"), dict)):
        raise ValueError("快照格式不兼容")
    return snapshot


class _TerminalPanel:
    """只重写变化行，进入时清屏一次；所有退出路径恢复光标和原终端模式。"""

    def __init__(self, stream):
        self.stream = stream
        self.previous = []
        self.console_mode = None
        self.geometry = None

    def __enter__(self):
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            kernel.GetStdHandle.argtypes = [wintypes.DWORD]
            kernel.GetStdHandle.restype = wintypes.HANDLE
            kernel.GetConsoleMode.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
            kernel.SetConsoleMode.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            handle = kernel.GetStdHandle(-11)
            mode = wintypes.DWORD()
            if not kernel.GetConsoleMode(handle, ctypes.byref(mode)):
                raise OSError("无法读取终端模式；请用 Windows Terminal 或 --once")
            if not kernel.SetConsoleMode(handle, mode.value | 0x0004):
                raise OSError("无法启用 ANSI；请用 Windows Terminal 或 --once")
            self.console_mode = kernel, handle, mode.value
        try:
            self.stream.write("\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H")
            self.stream.flush()
        except BaseException:
            # __enter__ 未成功时 Python 不会自动调用 __exit__。
            try:
                self.__exit__()
            except OSError:
                pass
            raise
        return self

    def draw(self, panel, height, width=100):
        geometry = max(1, width), max(1, height)
        width, height = geometry
        if self.geometry is not None and self.geometry != geometry:
            # 只在窗口尺寸改变时清理；不能用旧行缓存向缩小后的屏幕外写入。
            self.stream.write("\x1b[H\x1b[J")
            self.previous = []
        self.geometry = geometry
        lines = panel.splitlines()
        if len(lines) > height:
            lines = lines[:max(0, height - 1)] + [_fit_line("终端过矮，请增大窗口以显示完整面板。", width)]
        output = []
        for index in range(min(height, max(len(lines), len(self.previous)))):
            line = lines[index] if index < len(lines) else ""
            if index >= len(self.previous) or self.previous[index] != line:
                output.append(f"\x1b[{index + 1};1H{line}\x1b[K")
        self.stream.write("".join(output))
        self.stream.flush()
        self.previous = lines

    def __exit__(self, *_):
        try:
            self.stream.write("\x1b[?25h\x1b[?1049l")
            self.stream.flush()
        finally:
            if self.console_mode:
                kernel, handle, mode = self.console_mode
                kernel.SetConsoleMode(handle, mode)


def _port(text):
    value = int(text)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError("端口必须为1..65535")
    return value


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=_port, default=DEFAULT_PORT, help="本机快照端口（默认8801）")
    parser.add_argument("--once", action="store_true", help="输出一次纯文本后退出，可用于管道或排查")
    args = parser.parse_args(argv)
    endpoint = f"http://127.0.0.1:{args.port}/snapshot"

    def read_panel(width):
        try:
            snapshot = fetch_snapshot(endpoint)
            return render_panel(snapshot, width=width, endpoint=endpoint), 0
        except (OSError, URLError, HTTPException, ValueError) as exc:
            # 请求失败时不复用前一帧，清掉所有旧值；下次刷新自动重试。
            return render_panel(None, width=width, endpoint=endpoint, error=str(exc)), 1

    try:
        if args.once or not sys.stdout.isatty():
            panel, status = read_panel(100)
            print(panel)
            return status
        with _TerminalPanel(sys.stdout) as terminal:
            while True:
                started = time.monotonic()
                columns, rows = shutil.get_terminal_size((100, 30))
                panel, _ = read_panel(max(1, columns - 1))
                terminal.draw(panel, max(1, rows - 1), max(1, columns - 1))
                time.sleep(max(0.0, REFRESH_SECONDS - (time.monotonic() - started)))
    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        print(f"监视终端不可用：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

