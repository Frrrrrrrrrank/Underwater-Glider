"""按设备连接会话记录原始文本、兼容 CSV 和连接事件。"""

from __future__ import annotations

import csv
import json
import queue
import threading
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from .legacy_telemetry import (
    DecodedTelemetry,
    EKF_CSV_HEADER,
    UGCTL_CSV_HEADER,
    UGWRM_CSV_HEADER,
    UGRDS_CSV_HEADER,
    UGPS_CSV_HEADER,
    UGRAW_CSV_HEADER,
    ekf_csv_row,
    ugctl_csv_row,
    ugwrm_csv_row,
    ugrds_csv_row,
    ugps_csv_row,
    ugraw_csv_row,
)
from .models import utc_now_iso


CSV_FORMATS = {
    "ugekf": ("ekf.csv", EKF_CSV_HEADER, ekf_csv_row),
    "ugctl": ("ctl.csv", UGCTL_CSV_HEADER, ugctl_csv_row),
    "ugwrm": ("worm.csv", UGWRM_CSV_HEADER, ugwrm_csv_row),
    "ugrds": ("rdss.csv", UGRDS_CSV_HEADER, ugrds_csv_row),
    "ugps": ("gps.csv", UGPS_CSV_HEADER, ugps_csv_row),
    "ugraw": ("uart7_raw.csv", UGRAW_CSV_HEADER, ugraw_csv_row),
}


def disabled_recording_snapshot() -> Dict[str, Any]:
    """未启用记录时仍返回同一形状，避免前端猜测字段。"""
    return {
        "enabled": False, "status": "idle", "session_id": None,
        "started_at": None, "directory": None, "files": {}, "rows": {},
        "raw_lines": 0, "dropped_records": 0, "invalid_ugraw_count": 0,
        "last_error": None,
    }


class SessionRecorder:
    """磁盘写入由单独线程负责，接收线程只向有界队列提交。"""

    def __init__(self, root: Path, queue_size: int = 2048) -> None:
        if queue_size <= 0:
            raise ValueError("记录队列长度必须大于零")
        self._root = Path(root)
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._state = disabled_recording_snapshot()
        self._state["enabled"] = True
        self._handles: Dict[str, Any] = {}
        self._writers: Dict[str, Any] = {}
        self._directory: Optional[Path] = None
        self._csv_stamp = ""
        self._failed = False
        self._stopping = threading.Event()
        self._accepting = False
        # 会话故障对象随begin/end入队；后台尚未处理begin时的丢样也属于新会话。
        self._submission_faults = {"dropped_records": 0, "last_error": None}
        self._active_faults = None

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return deepcopy(self._state)

    def _error(self, message: str, dropped: bool = False, faults=None) -> None:
        with self._lock:
            target = self._active_faults if faults is None else faults
            if target is not None:
                target["last_error"] = message
                if dropped:
                    target["dropped_records"] += 1
            if faults is None or target is self._active_faults:
                self._state["status"] = "error"
                self._state["last_error"] = message
                if dropped:
                    self._state["dropped_records"] += 1

    def begin_session(self, session_id: str, peer: str) -> None:
        self._submission_faults = {"dropped_records": 0, "last_error": None}
        if self._thread is None or not self._thread.is_alive():
            self._stopping.clear()
            self._thread = threading.Thread(
                target=self._run, name="urgc-session-recorder", daemon=True,
            )
            self._thread.start()
        # 会话边界可以短暂等待；常规收包提交使用非阻塞操作。
        self._accepting = self._submit(
            ("begin", session_id, peer, utc_now_iso(), self._submission_faults), boundary=True)
        if not self._accepting:
            self._error("新会话未能进入记录队列，无法落盘")

    def record(self, line: str, decoded: Optional[DecodedTelemetry], *,
               received_ns: Optional[int] = None, received_at: Optional[str] = None) -> None:
        if self._accepting:
            self._submit(("line", line, decoded, received_at or utc_now_iso(),
                          time.monotonic_ns() if received_ns is None else received_ns))

    def end_session(self, reason: str) -> None:
        self._submit(("end", reason, utc_now_iso(), self._submission_faults), boundary=True)
        self._accepting = False

    def _submit(self, item: tuple, boundary: bool = False) -> bool:
        try:
            if boundary:
                self._queue.put(item, timeout=0.5)
            else:
                self._queue.put_nowait(item)
            return True
        except queue.Full:
            self._error("记录队列已满，存在未落盘数据", dropped=True, faults=self._submission_faults)
            return False

    def stop(self, timeout_s: float = 3.0) -> None:
        if self._thread is None:
            return
        self._accepting = False
        self._stopping.set()
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            self._error("记录线程未能在关闭期限内完成落盘")
        else:
            self._thread = None

    def _event(self, event_type: str, **values: Any) -> None:
        handle = self._handles.get("events")
        if handle is not None:
            handle.write(json.dumps({"type": event_type, **values}, ensure_ascii=False, allow_nan=False) + "\n")
            handle.flush()

    def _begin(self, session_id: str, peer: str, received_at: str, faults=None) -> None:
        self._close()
        self._failed = False
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ")
        self._csv_stamp = stamp[:15]
        # session_id 来源于本地 UUID；目录名仍只保留安全字符。
        safe_id = "".join(ch for ch in session_id if ch.isalnum())[:32] or "session"
        self._directory = self._root / (stamp + "_" + safe_id)
        with self._lock:
            self._active_faults = faults if faults is not None else self._submission_faults
            self._state.update(
                status="error" if self._active_faults["last_error"] else "starting",
                session_id=session_id, started_at=received_at,
                directory=str(self._directory), files={}, rows={}, raw_lines=0,
                dropped_records=self._active_faults["dropped_records"], invalid_ugraw_count=0,
                last_error=self._active_faults["last_error"],
            )
        self._directory.mkdir(parents=True, exist_ok=False)
        for key, filename in (("raw", "raw.log"), ("events", "events.jsonl")):
            path = self._directory / filename
            self._handles[key] = path.open("x", encoding="utf-8", newline="")
            with self._lock:
                self._state["files"][key] = str(path)
        self._event("session.started", received_at=received_at, session_id=session_id, peer=peer)
        with self._lock:
            if self._state["status"] != "error":
                self._state["status"] = "recording"

    def _line(self, line: str, decoded: Optional[DecodedTelemetry], received_at: str,
              received_ns: Optional[int] = None) -> None:
        if self._failed or "raw" not in self._handles:
            return
        # 原有主题可能留下额外CR用于判坏帧；不能strip后悄悄修复证据。
        self._handles["raw"].write(received_at + "\t" + line + ("" if line.endswith("\n") else "\n"))
        self._handles["raw"].flush()
        with self._lock:
            self._state["raw_lines"] += 1
        if line.startswith("$UGRAW"):
            # 坏外包与完整外包一并留证；内层载荷不做二次NMEA过滤。
            if decoded is None:
                with self._lock:
                    self._state["invalid_ugraw_count"] += 1
            self._event("rx", kind="rx", topic="ugraw", received_at=received_at,
                        t_ns=received_ns, raw=line, complete=line.endswith("\n"),
                        row=decoded.data if decoded is not None else None)
        if decoded is None or decoded.topic not in CSV_FORMATS:
            return
        topic = decoded.topic
        filename, header, make_row = CSV_FORMATS[topic]
        filename = filename.replace(".csv", "_" + self._csv_stamp + ".csv")
        if topic not in self._writers:
            path = self._directory / filename
            handle = path.open("x", encoding="utf-8", newline="")
            self._handles[topic] = handle
            self._writers[topic] = csv.writer(handle)
            self._writers[topic].writerow(header)
            with self._lock:
                self._state["files"][topic] = str(path)
                self._state["rows"][topic] = 0
        row = make_row(decoded.data)
        # CSV 列顺序保持兼容，但时间取实际收包时刻，不取后台落盘时刻。
        row[0] = received_at
        if topic == "ugraw":
            row[1] = received_ns
        self._writers[topic].writerow(row)
        self._handles[topic].flush()
        with self._lock:
            self._state["rows"][topic] += 1

    def _close(self) -> None:
        for handle in self._handles.values():
            try:
                handle.close()
            except OSError as exc:
                self._error(str(exc))
        self._handles.clear()
        self._writers.clear()

    def _run(self) -> None:
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.1)
                except queue.Empty:
                    if self._stopping.is_set():
                        break
                    continue
                try:
                    if item[0] == "begin":
                        self._begin(*item[1:])
                    elif item[0] == "line":
                        self._line(*item[1:])
                    elif item[0] == "end":
                        if len(item) > 3 and item[3] is not self._active_faults:
                            # begin本身入队失败时，不把旧会话的文件当成新会话结束记录。
                            continue
                        snapshot = self.snapshot()
                        # 结束事件排在本会话记录之后；离线工具可辨认队列丢样与写盘故障。
                        self._event(
                            "session.ended", reason=item[1], received_at=item[2],
                            status="error" if snapshot["status"] == "error" else "idle",
                            dropped_records=snapshot["dropped_records"],
                            invalid_ugraw_count=snapshot["invalid_ugraw_count"],
                            last_error=snapshot["last_error"], pending_records=0,
                            complete=(not self._failed and snapshot["status"] != "error"
                                      and snapshot["dropped_records"] == 0),
                        )
                        self._close()
                        with self._lock:
                            if self._state["status"] != "error":
                                self._state["status"] = "idle"
                except (OSError, ValueError, TypeError, KeyError) as exc:
                    self._failed = True
                    self._error(str(exc))
                    self._close()
                finally:
                    self._queue.task_done()
        finally:
            self._close()
            with self._lock:
                if self._state["status"] != "error":
                    self._state["status"] = "stopped"
