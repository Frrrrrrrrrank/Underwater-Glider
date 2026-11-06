"""实验会话记录：控制线程只投递，后台线程负责文件与批量刷新。"""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import re
import threading
import time
import uuid

try:
    from .gps_telemetry import UGPS_FIELDS
except ImportError:
    from gps_telemetry import UGPS_FIELDS


CONTROL_FIELDS = (
    "pitchmotor_enable", "pitchmotor_speed_rpm", "rollmotor_enable",
    "rollmotor_speed_rpm", "thrustmotor_direction", "thrustmotor_direction2",
    "thrustmotor1se_speed_rpm", "thrustmotor2se_speed_rpm", "backup_for_thrust",
    "pump_direction", "pump_on_off", "ask_for_status", "eth_control_flag",
    "rudder_angle_cdeg",
)
COMMON_COLUMNS = ("session_id", "event_id", "t_ns", "host_iso")
COMMAND_COLUMNS = COMMON_COLUMNS + (
    "source", "phase", "run_id", "tx_start_ns", "tx_end_ns", "seq", "msg_type",
    "success", "error", "payload_hex",
) + CONTROL_FIELDS + ("input_json", "extra_fields_json", "nonfinite_fields_json", "rudder_angle_deg")
TELEMETRY_FIELDS = (
    "schema", "seq", "t_ms", "valid", "data_fresh", "span_valid", "pulse",
    "stroke_percent", "cmd_rpm", "fb_rpm", "moving", "alarm", "ref_kind",
    "ref_lost", "limit_bits", "roll_pulse", "roll_percent", "roll_valid",
    "calib_state", "calib_hold", "home_state", "roll_home_state",
    "status", "fl", "roll_deg", "pitch_deg", "yaw_deg", "depth", "vD",
    "displacement_mm", "mode", "armed", "output_valid", "fault",
    "battery_mv", "filtered_mv", "state", "sample_age_ms", "error_count",
)
# 新列放到原 CSV 尾部，旧主题的原有列名/顺序不变。
GPS_TELEMETRY_FIELDS = tuple(name for name in UGPS_FIELDS if name not in TELEMETRY_FIELDS)
TELEMETRY_COLUMNS = COMMON_COLUMNS + ("batch_id", "line_index", "topic") + TELEMETRY_FIELDS + (
    "row_json", "raw", "nonfinite_fields_json",
) + GPS_TELEMETRY_FIELDS


def _host_iso():
    return datetime.now().astimezone().isoformat(timespec="microseconds")


def _clean_json(value, nonfinite, path=""):
    """复制可序列化数据；坏浮点显式留痕，不让 JSON 产生 NaN/Infinity。"""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        nonfinite.append(path or "$")
        return None
    if isinstance(value, dict):
        return {
            str(key): _clean_json(item, nonfinite, path + "." + str(key))
            for key, item in value.items()
        }
    if isinstance(value, (tuple, list)):
        return [
            _clean_json(item, nonfinite, "%s[%d]" % (path, index))
            for index, item in enumerate(value)
        ]
    raise TypeError("实验事件含不可序列化类型：%s" % type(value).__name__)


def _json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


class ExperimentRecorder:
    """单会话记录器；满队列和写盘故障只损害记录完整性，不中断控制。"""

    FLUSH_PERIOD_S = 0.25
    SESSION_PERIOD_S = 1.0
    START_TIMEOUT_S = 2.0

    def __init__(self, root, metadata=None, queue_size=4096, rdss_root=None):
        if not isinstance(queue_size, int) or isinstance(queue_size, bool) or queue_size <= 0:
            raise ValueError("queue_size 必须是正整数")
        self._root = Path(root)
        self._rdss_root = Path(rdss_root) if rdss_root is not None else None
        self._rdss_log_path = None
        self._rdss_log = None
        nonfinite = []
        self._metadata = _clean_json(metadata or {}, nonfinite)
        self._metadata_nonfinite = nonfinite
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._ready = threading.Event()
        self._done = threading.Event()
        self._thread = None
        self._accepting = False
        self._status = "stopped"
        self._closed = False
        self._incomplete = False
        self._error_version = 0
        self._finalized = False
        self._stop_timed_out = False
        self._last_error = ""
        self._session_id = None
        self._directory = None
        self._peer = ""
        self._started_ns = None
        self._started_iso = None
        self._stopped_iso = None
        self._event_id = 0
        self._accepted = 0
        self._written = 0
        self._dropped = 0
        self._rejected = 0
        self._handles = []
        self._telemetry_writers = {}

    def start(self, peer=""):
        """启动时等待文件就绪；任何文件操作均在后台执行。"""
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("实验记录器只能启动一次")
            self._session_id = uuid.uuid4().hex
            timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
            self._directory = str(self._root / (timestamp + "_" + self._session_id[:8]))
            if self._rdss_root is not None:
                self._rdss_log_path = str(self._rdss_root / (
                    "rsdd_" + timestamp + "_" + self._session_id[:8] + ".log"))
            self._peer = str(peer)
            self._started_ns = time.monotonic_ns()
            self._started_iso = _host_iso()
            self._thread = threading.Thread(
                target=self._worker, name="experiment-recorder", daemon=True)
            thread = self._thread
        thread.start()
        if not self._ready.wait(self.START_TIMEOUT_S):
            with self._lock:
                self._accepting = False
                self._set_error_locked("实验记录器初始化超时")
            self._stop_requested.set()
            raise RuntimeError("实验记录器初始化超时")
        with self._lock:
            if self._status == "error":
                raise OSError(self._last_error)
            return self._directory

    def _set_error_locked(self, message):
        self._status = "error"
        self._incomplete = True
        self._last_error = str(message)
        self._error_version += 1

    def record(self, event):
        """非阻塞投递；复制防止调用方复用字典，无磁盘操作且不抛异常。"""
        try:
            with self._lock:
                if not self._accepting:
                    self._rejected += 1
                    return False
            fallback_ns = time.monotonic_ns()
            fallback_iso = _host_iso()
            if not isinstance(event, dict):
                raise TypeError("实验事件必须为字典")
            nonfinite = []
            owned = _clean_json(event, nonfinite)
            owned.setdefault("t_ns", fallback_ns)
            owned.setdefault("host_iso", fallback_iso)
            if nonfinite:
                owned["nonfinite_fields"] = nonfinite
            with self._lock:
                if not self._accepting:
                    self._rejected += 1
                    return False
                self._event_id += 1
                owned["event_id"] = self._event_id
                owned["session_id"] = self._session_id
                try:
                    self._queue.put_nowait(owned)
                except queue.Full:
                    self._dropped += 1
                    self._set_error_locked("实验记录队列已满，记录不完整")
                    return False
                self._accepted += 1
                return True
        except Exception as exc:
            with self._lock:
                self._dropped += 1
                self._set_error_locked("实验事件记录失败：%s" % exc)
            return False

    def snapshot(self):
        with self._lock:
            return {
                "status": self._status,
                "directory": self._directory,
                "session_id": self._session_id,
                "rdss_log_path": self._rdss_log_path,
                "accepting": self._accepting,
                "closed": self._closed,
                "complete": self._closed and not self._incomplete and self._accepted == self._written,
                "accepted_records": self._accepted,
                "written_records": self._written,
                "dropped_records": self._dropped,
                "rejected_records": self._rejected,
                "pending_records": self._accepted - self._written,
                "last_event_id": self._event_id,
                "last_error": self._last_error,
                "stop_timed_out": self._stop_timed_out,
            }

    def mark_incomplete(self, reason):
        """收尾前标记接收线程等外部遗漏；不阻止后台继续排空已入队数据。"""
        try:
            with self._lock:
                self._set_error_locked("实验记录不完整：%s" % reason)
        except Exception:
            # 该接口会在设备退出路径调用，异常必须被限制在记录旁路。
            return

    def stop(self, timeout_s=2.0):
        """先拒收，再让后台排空；慢磁盘不能无限阻挡退出/停止控制。"""
        try:
            timeout_s = float(timeout_s)
            if not math.isfinite(timeout_s) or timeout_s < 0:
                timeout_s = 0.0
        except (TypeError, ValueError):
            timeout_s = 0.0
        with self._lock:
            self._accepting = False
            thread = self._thread
            if thread is None:
                self._closed = True
                return
        self._stop_requested.set()
        if thread is not threading.current_thread():
            thread.join(timeout_s)
        if thread.is_alive():
            with self._lock:
                if not self._finalized:
                    self._stop_timed_out = True
                    self._set_error_locked("实验日志收尾超时，完整性未确认")

    def _open_outputs(self):
        directory = Path(self._directory)
        directory.mkdir(parents=True, exist_ok=False)
        self._events = (directory / "events.jsonl").open("x", encoding="utf-8", newline="")
        self._handles.append(self._events)
        commands = (directory / "commands.csv").open("x", encoding="utf-8-sig", newline="")
        self._handles.append(commands)
        self._commands = csv.DictWriter(commands, fieldnames=COMMAND_COLUMNS)
        self._commands.writeheader()
        if self._rdss_root is not None:
            self._rdss_root.mkdir(parents=True, exist_ok=True)
            self._rdss_log = Path(self._rdss_log_path).open("x", encoding="utf-8", newline="")
            self._handles.append(self._rdss_log)

    def _telemetry_writer(self, topic):
        topic = str(topic or "unknown")
        if topic not in self._telemetry_writers:
            safe = re.sub(r"[^a-z0-9_-]", "_", topic.lower())[:64] or "unknown"
            if safe != topic:
                safe += "_" + hashlib.sha256(topic.encode("utf-8")).hexdigest()[:8]
            path = Path(self._directory) / ("telemetry_" + safe + ".csv")
            handle = path.open("x", encoding="utf-8-sig", newline="")
            self._handles.append(handle)
            writer = csv.DictWriter(handle, fieldnames=TELEMETRY_COLUMNS)
            writer.writeheader()
            self._telemetry_writers[topic] = writer
        return self._telemetry_writers[topic]

    def _write_event(self, event):
        self._events.write(_json(event) + "\n")
        kind = event.get("kind")
        if kind == "tx":
            row = {key: event.get(key, "") for key in COMMAND_COLUMNS}
            fields = event.get("fields") or {}
            if not isinstance(fields, dict):
                fields = {}
            row.update({key: fields.get(key, "") for key in CONTROL_FIELDS})
            angle = fields.get("rudder_angle_cdeg")
            # 保留原始百分之一度列；派生度列便于直接查看，发送失败仍由 success 区分。
            row["rudder_angle_deg"] = angle / 100.0 if type(angle) is int else ""
            row["input_json"] = _json(event.get("input"))
            row["extra_fields_json"] = _json({
                key: value for key, value in fields.items() if key not in CONTROL_FIELDS})
            row["nonfinite_fields_json"] = _json(event.get("nonfinite_fields", []))
            self._commands.writerow(row)
        elif kind == "rx":
            row = {key: event.get(key, "") for key in TELEMETRY_COLUMNS}
            parsed = event.get("row")
            for key in TELEMETRY_FIELDS + GPS_TELEMETRY_FIELDS:
                row[key] = parsed.get(key, "") if isinstance(parsed, dict) else ""
            # 全部字段存 JSON 列，避免老协议首帧决定表头后丢失新版字段。
            row["row_json"] = _json(parsed)
            row["nonfinite_fields_json"] = _json(event.get("nonfinite_fields", []))
            self._telemetry_writer(event.get("topic")).writerow(row)
            if event.get("topic") == "ugrds" and self._rdss_log is not None:
                # 独立 .log 使用 JSON 行转义换行，坏校验和断连残片的原文不会被吞掉。
                rdss_event = {key: event.get(key) for key in (
                    "session_id", "event_id", "host_iso", "t_ns", "batch_id", "line_index", "raw")}
                rdss_event["complete"] = event.get("complete", True)
                rdss_event["parsed"] = isinstance(parsed, dict)
                self._rdss_log.write(_json(rdss_event) + "\n")

    def _flush(self):
        for handle in self._handles:
            handle.flush()

    def _write_session(self):
        summary = self.snapshot()
        summary.update({
            "format_version": 1,
            "clock": "time.monotonic_ns",
            "started_ns": self._started_ns,
            "started_iso": self._started_iso,
            "stopped_iso": self._stopped_iso,
            "peer": self._peer,
            "metadata": self._metadata,
            "metadata_nonfinite_fields": self._metadata_nonfinite,
        })
        path = Path(self._directory) / "session.json"
        temporary = path.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8", newline="") as handle:
            json.dump(summary, handle, ensure_ascii=False, allow_nan=False, indent=2)
            handle.write("\n")
        temporary.replace(path)

    def _worker(self):
        try:
            self._open_outputs()
            with self._lock:
                if not self._incomplete and not self._stop_requested.is_set():
                    self._status = "recording"
                    self._accepting = True
            self._flush()
            self._write_session()
            self._ready.set()
            last_flush = last_session = time.monotonic()
            while not self._stop_requested.is_set() or not self._queue.empty():
                try:
                    event = self._queue.get(timeout=0.05)
                except queue.Empty:
                    event = None
                if event is not None:
                    try:
                        self._write_event(event)
                        with self._lock:
                            self._written += 1
                    finally:
                        self._queue.task_done()
                now = time.monotonic()
                if now - last_flush >= self.FLUSH_PERIOD_S:
                    self._flush()
                    last_flush = now
                if now - last_session >= self.SESSION_PERIOD_S:
                    self._write_session()
                    last_session = now
            self._flush()
        except Exception as exc:
            with self._lock:
                self._accepting = False
                self._set_error_locked("实验日志写盘失败：%s" % exc)
                # 包含本次可能部分写出的事件，以及队列中尚未写出的事件。
                self._dropped += self._accepted - self._written
            while True:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    break
        finally:
            self._ready.set()
            for handle in self._handles:
                try:
                    handle.close()
                except Exception as exc:
                    with self._lock:
                        self._set_error_locked("实验日志关闭失败：%s" % exc)
            with self._lock:
                self._accepting = False
                self._closed = True
                self._stopped_iso = _host_iso()
                if not self._incomplete:
                    self._status = "stopped"
            try:
                if self._directory is not None:
                    # stop 超时可能与最终写状态并发，必须重写而不能留下 complete=true。
                    while True:
                        with self._lock:
                            version = self._error_version
                        self._write_session()
                        with self._lock:
                            if version == self._error_version:
                                self._finalized = True
                                break
            except Exception as exc:
                with self._lock:
                    self._set_error_locked("实验会话状态保存失败：%s" % exc)
            self._done.set()
