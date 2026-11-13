"""URGC 的板端 Ethernet/TCP 唯一接收者。"""

from __future__ import annotations

import logging
import socket
import threading
import time
import uuid
from typing import Optional, Tuple

from .legacy_telemetry import decode_telemetry_line
from .models import LinkStatus, utc_now_iso
from .state_store import StateStore
from .session_recorder import SessionRecorder, disabled_recording_snapshot

try:
    from ..raw_telemetry import UGRAW_RENEW_S
    from ..ugcp_protocol import (
        MSG_STREAM_CONTROL,
        STREAM_CTL,
        STREAM_EKF,
        STREAM_WRM,
        STREAM_RDSS,
        STREAM_RAW,
        SequenceCounter,
        build_frame,
        pack_stream_control_payload,
    )
except ImportError:
    from raw_telemetry import UGRAW_RENEW_S
    from ugcp_protocol import (
        MSG_STREAM_CONTROL,
        STREAM_CTL,
        STREAM_EKF,
        STREAM_WRM,
        STREAM_RDSS,
        STREAM_RAW,
        SequenceCounter,
        build_frame,
        pack_stream_control_payload,
    )


LOGGER = logging.getLogger(__name__)
DEFAULT_STREAM_MASK = STREAM_EKF | STREAM_CTL | STREAM_WRM | STREAM_RDSS
MAX_LINE_BUFFER = 64 * 1024


class EthernetRealtimeLink:
    """监听板端 TCP 客户端并把有效遥测发布到 StateStore。

    本类是 socket 的唯一所有者。连接后只发送遥测流订阅；请求 RDSS 时先发
    基础主题，再尝试扩展主题，兼容拒绝未知订阅位的旧固件。不暴露执行器、
    PID、参数或回零命令。
    """

    def __init__(
        self,
        store: StateStore,
        *,
        host: str = "192.168.57.100",
        port: int = 8800,
        stream_mask: int = DEFAULT_STREAM_MASK,
        retry_delay_s: float = 2.0,
        recorder: Optional[SessionRecorder] = None,
        raw_capture_enabled: bool = False,
    ) -> None:
        self._store = store
        self._recorder = recorder
        self._host = host
        self._port = int(port)
        self._stream_mask = int(stream_mask) | (STREAM_RAW if raw_capture_enabled else 0)
        self._raw_last_renew_s = 0.0
        if self._stream_mask & STREAM_RAW and recorder is None:
            LOGGER.warning("UART7原始采集已启用，但记录器关闭：仅接收转发数据，不落盘")
        self._retry_delay_s = float(retry_delay_s)
        self._sequence = SequenceCounter()
        self._stop_event = threading.Event()
        self._listening_event = threading.Event()
        self._socket_lock = threading.Lock()
        self._listener: Optional[socket.socket] = None
        self._connection: Optional[socket.socket] = None
        self._bound_endpoint: Optional[Tuple[str, int]] = None
        self._thread: Optional[threading.Thread] = None

        # 构造时即校验掩码，避免线程启动后才暴露配置错误。
        pack_stream_control_payload(self._stream_mask)
        if not 0 <= self._port <= 65535:
            raise ValueError("port 必须在 0..65535")
        if self._retry_delay_s <= 0.0:
            raise ValueError("retry_delay_s 必须大于 0")

    @property
    def configured_endpoint(self) -> Tuple[str, int]:
        return self._host, self._port

    def recording_snapshot(self) -> dict:
        return self._recorder.snapshot() if self._recorder else disabled_recording_snapshot()

    @property
    def bound_endpoint(self) -> Optional[Tuple[str, int]]:
        with self._socket_lock:
            return self._bound_endpoint

    def wait_until_listening(self, timeout_s: float) -> bool:
        return self._listening_event.wait(timeout_s)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._listening_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="urgc-ethernet-link",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 3.0) -> None:
        self._stop_event.set()
        with self._socket_lock:
            listener = self._listener
            connection = self._connection
        deadline = time.monotonic() + max(0.0, timeout_s)
        # 接收者在最多0.5秒的recv超时后自行发全关，避免第二个线程竞争发送。
        if self._stream_mask & STREAM_RAW and self._thread is not None:
            self._thread.join(timeout=min(0.6, max(0.0, timeout_s)))
        self._close_socket(connection)
        self._close_socket(listener)
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        self._thread = None
        if self._recorder is not None:
            self._recorder.stop()

    @staticmethod
    def _close_socket(sock: Optional[socket.socket]) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _set_listener(self, listener: Optional[socket.socket]) -> None:
        with self._socket_lock:
            self._listener = listener
            if listener is None:
                self._bound_endpoint = None
            else:
                host, port = listener.getsockname()[:2]
                self._bound_endpoint = str(host), int(port)

    def _set_connection(self, connection: Optional[socket.socket]) -> None:
        with self._socket_lock:
            self._connection = connection

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                listener: Optional[socket.socket] = None
                try:
                    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    listener.bind((self._host, self._port))
                    listener.listen(1)
                    listener.settimeout(0.5)
                    self._set_listener(listener)
                    self._listening_event.set()
                    self._store.set_link(LinkStatus.WAITING)
                    LOGGER.info("URGC device link listening on %s:%d", *listener.getsockname()[:2])

                    while not self._stop_event.is_set():
                        try:
                            connection, address = listener.accept()
                        except socket.timeout:
                            continue
                        except OSError:
                            if self._stop_event.is_set():
                                break
                            raise
                        self._handle_connection(connection, address)
                except OSError as exc:
                    if not self._stop_event.is_set():
                        LOGGER.warning("URGC device link error: %s", exc)
                        self._store.set_link(LinkStatus.ERROR, last_error=str(exc))
                        self._stop_event.wait(self._retry_delay_s)
                finally:
                    self._set_listener(None)
                    self._close_socket(listener)
        finally:
            self._set_connection(None)
            self._store.set_link(LinkStatus.STOPPED)

    def _handle_connection(
        self,
        connection: socket.socket,
        address: Tuple[str, int],
    ) -> None:
        session_id = uuid.uuid4().hex
        peer = "{}:{}".format(address[0], address[1])
        reason = "peer_closed"
        if self._recorder is not None:
            self._recorder.begin_session(session_id, peer)
        self._set_connection(connection)
        connection.settimeout(0.5)
        self._store.clear_session_telemetry()
        self._store.set_link(
            LinkStatus.CONNECTED,
            peer=peer,
            session_id=session_id,
        )

        try:
            self._send_stream_subscription(connection)
            reason = self._receive_lines(connection, session_id)
        except OSError as exc:
            reason = str(exc)
        finally:
            if self._stream_mask & STREAM_RAW:
                try:
                    self._send_stream_mask(connection, 0)
                except OSError:
                    pass
            self._set_connection(None)
            self._close_socket(connection)
            if self._recorder is not None:
                self._recorder.end_session("station_stopped" if self._stop_event.is_set() else reason)

        if not self._stop_event.is_set():
            self._store.set_link(LinkStatus.DISCONNECTED, last_error=reason)

    def _send_stream_subscription(self, connection: socket.socket, *, renew: bool = False) -> None:
        masks = [self._stream_mask]
        if not renew:
            base = self._stream_mask & ~STREAM_RAW
            if self._stream_mask & STREAM_RAW:
                masks.insert(0, base)
            if base & STREAM_RDSS:
                # 旧固件会整包拒绝未知位；初次分级订阅，续租不能插入关闭RAW的包。
                masks.insert(0, base & ~STREAM_RDSS)
        for mask in dict.fromkeys(masks):
            self._send_stream_mask(connection, mask)
        if self._stream_mask & STREAM_RAW:
            self._raw_last_renew_s = time.monotonic()

    def _send_stream_mask(self, connection: socket.socket, mask: int) -> None:
        frame = build_frame(MSG_STREAM_CONTROL, pack_stream_control_payload(mask), self._sequence.next())
        connection.sendall(frame)
        self._store.mark_tx()

    def _renew_raw_subscription(self, connection: socket.socket, now: Optional[float] = None) -> None:
        if (self._stream_mask & STREAM_RAW and
                (time.monotonic() if now is None else now) - self._raw_last_renew_s >= UGRAW_RENEW_S):
            self._send_stream_subscription(connection, renew=True)

    def _receive_lines(self, connection: socket.socket, session_id: str) -> str:
        line_buffer = ""
        received_ns = None
        received_at = None

        def finish(reason: str) -> str:
            # 断线留下的不完整外包也入日志，不能悄悄当成没有收到。
            if line_buffer.startswith("$UGRAW") and self._recorder is not None:
                self._recorder.record(line_buffer, None, received_ns=received_ns, received_at=received_at)
            return reason

        while not self._stop_event.is_set():
            try:
                self._renew_raw_subscription(connection)
                data = connection.recv(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:
                return finish("connection_reset")
            except OSError:
                finish("socket_error")
                raise

            if not data:
                return finish("peer_closed")

            received_ns, received_at = time.monotonic_ns(), utc_now_iso()

            line_buffer += data.decode("ascii", errors="replace")
            if len(line_buffer) > MAX_LINE_BUFFER:
                finish("line_too_long")
                line_buffer = ""
                LOGGER.warning("URGC cleared an oversized device line buffer")
                continue

            while True:
                newline_index = line_buffer.find("\n")
                if newline_index < 0:
                    break
                raw_line = line_buffer[:newline_index + 1]
                line = line_buffer[:newline_index]
                # 只去掉正常 CRLF 的一个 CR，额外 CR 留给解析器判坏帧。
                if line.endswith("\r"):
                    line = line[:-1]
                line_buffer = line_buffer[newline_index + 1:]
                is_raw = line.startswith("$UGRAW")
                decoded = decode_telemetry_line(raw_line if is_raw else line)
                if self._recorder is not None:
                    if is_raw:
                        self._recorder.record(raw_line, decoded, received_ns=received_ns, received_at=received_at)
                    else:
                        self._recorder.record(line, decoded)
                if decoded is None or is_raw:
                    continue
                self._store.publish_telemetry(
                    decoded.topic,
                    decoded.data,
                    source="ethernet_tcp",
                    device_tick_ms=decoded.device_tick_ms,
                    session_id=session_id,
                    stale_after_ms=decoded.stale_after_ms,
                )

        return finish("station_stopped")
