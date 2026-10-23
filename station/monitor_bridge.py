"""CLI 的本机只读快照桥；不接触设备 socket，也不发送板端命令。"""

from __future__ import annotations

import ipaddress
import json
import socket
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Mapping, Optional, Tuple

try:
    from .groundstation.models import LinkStatus
    from .groundstation.state_store import StateStore
except ImportError:
    from groundstation.models import LinkStatus
    from groundstation.state_store import StateStore


TOPIC_STALE_MS = {
    "ugekf": 500,
    "ugctl": 500,
    "ugwrm": 500,
    "ugbat": 3000,
    "ugrds": 2000,
}
CLIENT_TIMEOUT_S = 2.0
MAX_HTTP_CLIENTS = 4


class _SnapshotHandler(BaseHTTPRequestHandler):
    """每次请求仅序列化一份最新快照，慢读者不会持有状态锁。"""

    protocol_version = "HTTP/1.0"

    def log_message(self, format: str, *args: Any) -> None:
        # 面板周期轮询不占用 CLI 输入行。
        pass

    def do_GET(self) -> None:
        if self.path != "/snapshot":
            self.send_error(404, "Not Found")
            return
        snapshot = self.server.bridge.snapshot()
        payload = json.dumps(
            snapshot, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(payload)
        except (OSError, TimeoutError):
            # 读者退出或读得过慢只影响本次 HTTP，不影响接收线程和其他面板。
            pass

    def _reject_method(self) -> None:
        self.send_response(405, "Method Not Allowed")
        self.send_header("Allow", "GET")
        self.send_header("Content-Length", "0")
        self.send_header("Connection", "close")
        self.end_headers()

    do_POST = do_PUT = do_PATCH = do_DELETE = do_HEAD = do_OPTIONS = _reject_method


class _SnapshotServer(ThreadingHTTPServer):
    """有限并发及有限连接超时；关停不等待滞留读者。"""

    daemon_threads = True
    block_on_close = False
    request_queue_size = MAX_HTTP_CLIENTS
    # Windows 的 SO_REUSEADDR 允许多个监听者抢占同一端口，必须使用独占绑定。
    allow_reuse_address = not hasattr(socket, "SO_EXCLUSIVEADDRUSE")
    allow_reuse_port = False

    def __init__(self, address: Tuple[str, int], bridge: "MonitorBridge") -> None:
        self.address_family = (
            socket.AF_INET6 if ipaddress.ip_address(address[0]).version == 6 else socket.AF_INET
        )
        self.bridge = bridge
        self._client_slots = threading.BoundedSemaphore(MAX_HTTP_CLIENTS)
        super().__init__(address, _SnapshotHandler)

    def server_bind(self) -> None:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(CLIENT_TIMEOUT_S)
        return connection, address

    def verify_request(self, request, client_address) -> bool:
        return ipaddress.ip_address(client_address[0]).is_loopback

    def process_request(self, request, client_address) -> None:
        if not self._client_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._client_slots.release()
            raise

    def process_request_thread(self, request, client_address) -> None:
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._client_slots.release()


class MonitorBridge:
    """供唯一 CLI 接收线程发布、独立只读面板轮询的最新状态缓存。"""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8801,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        # 不通过 DNS 扩展监听范围；localhost 固定解释为 IPv4 回环。
        if host == "localhost":
            host = "127.0.0.1"
        try:
            local_ip = ipaddress.ip_address(host)
        except ValueError as exc:
            raise ValueError("monitor 只接受本机回环 IP 或 localhost") from exc
        if not local_ip.is_loopback:
            raise ValueError("monitor 只允许绑定本机回环地址")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise ValueError("monitor 端口必须为 0..65535 的整数")
        self._configured_address = (str(local_ip), port)
        self._address = self._configured_address
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._store = StateStore(clock=clock, required_topics=tuple(TOPIC_STALE_MS))
        self._status = LinkStatus.STOPPED
        self._session_id: Optional[str] = None
        self._last_keys = {}
        self._server: Optional[_SnapshotServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def address(self) -> Tuple[str, int]:
        """返回实际监听地址；尚未启动时为配置地址，port=0 支持临时端口。"""
        return self._address

    def start(self) -> Tuple[str, int]:
        """启动本机 HTTP 线程并返回地址；重复调用不会创建第二个监听者。"""
        with self._lifecycle_lock:
            if self._server is not None:
                return self._address
            server = _SnapshotServer(self._configured_address, self)
            thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.1},
                name="station-monitor-http",
                daemon=True,
            )
            try:
                thread.start()
            except Exception:
                server.server_close()
                raise
            self._address = (server.server_address[0], server.server_address[1])
            self._server, self._thread = server, thread
            return self._address

    def stop(self) -> None:
        """停止本机接口；不关闭 CLI 的设备连接，不发送任何停止/退出命令。"""
        with self._lifecycle_lock:
            self.set_link("stopped")
            server, thread = self._server, self._thread
            self._server = self._thread = None
            if server is not None:
                server.shutdown()
                server.server_close()
            if thread is not None:
                thread.join(timeout=CLIENT_TIMEOUT_S + 0.5)

    def set_link(
        self, status: str, peer: Optional[str] = None, last_error: Optional[str] = None
    ) -> None:
        """CLI 发布设备连接状态；每次 connected 都开启新会话并清空旧数据。"""
        state = LinkStatus(status)
        with self._lock:
            if state is LinkStatus.CONNECTED:
                self._session_id = uuid.uuid4().hex
                self._last_keys.clear()
                self._store.clear_session_telemetry()
            self._status = state
            self._store.set_link(
                state, peer=peer, session_id=self._session_id, last_error=last_error
            )

    def publish(self, topic: str, row: Mapping[str, Any]) -> bool:
        """发布已通过原解析器校验的行；重复帧不能刷新本机接收新鲜度。"""
        if topic not in TOPIC_STALE_MS:
            raise ValueError("monitor 不支持该遥测主题: {}".format(topic))
        key = (row.get("seq"), row.get("t_ms"))
        if any(type(value) is not int or not 0 <= value <= 0xFFFFFFFF for value in key):
            return False
        with self._lock:
            if self._status is not LinkStatus.CONNECTED or self._last_keys.get(topic) == key:
                return False
            self._store.publish_telemetry(
                topic, row, source="cli_ethernet_tcp", device_tick_ms=key[1],
                session_id=self._session_id, stale_after_ms=TOPIC_STALE_MS[topic],
            )
            self._last_keys[topic] = key
            return True

    @staticmethod
    def _apply_monitor_quality(topic: str, sample: dict) -> None:
        """补充面板需要的有效性，不修改网页共享规则或原始有限诊断值。"""
        data, invalid = sample["data"], sample["invalid_fields"]

        def mark(fields, reason):
            for field in fields:
                if field in data:
                    invalid.setdefault(field, reason)

        if topic == "ugwrm":
            if data.get("ref_kind") not in (1, 2):
                mark(("stroke_percent", "mm_from_front"), "worm_reference_unavailable")
            if data.get("span_valid") != 1:
                mark(("stroke_percent", "mm_from_front"), "worm_span_unknown")
            if data.get("data_fresh") != 1:
                mark(("pulse", "stroke_percent", "mm_from_front", "fb_rpm"),
                     "worm_feedback_unknown")
            if data.get("roll_valid") != 1:
                mark(("roll_percent",), "roll_reference_unknown")
        elif topic == "ugbat":
            if data.get("valid") != 1:
                mark(("battery_mv", "filtered_mv"), "battery_invalid")
            sample_age = data.get("sample_age_ms")
            if type(sample_age) is not int or sample_age < 0:
                mark(("battery_mv", "filtered_mv"), "battery_sample_age_unknown")
            elif sample_age > 500:
                mark(("battery_mv", "filtered_mv"), "battery_sample_stale")
        elif topic == "ugrds":
            if data.get("has_cached_fix") != 1:
                mark(("local_latitude_deg_e7", "local_longitude_deg_e7"),
                     "local_fix_unavailable")
            if data.get("target_valid") != 1:
                mark(("target_latitude_deg_e7", "target_longitude_deg_e7"),
                     "target_coordinate_unavailable")

        reasons = list(dict.fromkeys(sample["quality"]["reasons"] + list(invalid.values())))
        sample["quality"] = {
            "status": "invalid" if invalid else sample["quality"]["status"],
            "reasons": reasons,
        }

    def snapshot(self) -> dict:
        """取得独立的公开字典；断线后保留诊断值但禁止把它们显示为当前值。"""
        with self._lock:
            _generation, result = self._store.read()
        connected = result["device_link"]["status"] == LinkStatus.CONNECTED.value
        for topic, sample in result["telemetry"].items():
            self._apply_monitor_quality(topic, sample)
            if not connected:
                sample["fresh"] = False
        result["state"]["stale_topics"] = [
            topic for topic, sample in result["telemetry"].items() if not sample["fresh"]
        ]
        result["state"]["capabilities"].update({
            "pid_parameters_readback": False,
            "final_pwm_feedback": False,
            "battery_soc": False,
        })
        result["monitor"] = {"schema": 1, "read_only": True, "transport": "loopback_http"}
        return result
