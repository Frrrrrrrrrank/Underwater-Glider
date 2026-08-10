"""TCP ground-station console for Ethernet and RDSS forwarding debug."""

import argparse
import socket
import threading
from datetime import datetime


DEFAULT_HOST = "192.168.57.100"
DEFAULT_PORT = 8800


def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def format_ascii(data: bytes) -> str:
    return data.decode("ascii", errors="replace").replace("\r", "<CR>").replace(
        "\n", "<LF>"
    )


class DebugSession:
    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self.send_lock = threading.Lock()
        self.print_lock = threading.Lock()
        self.stopped = threading.Event()

    def print(self, message: str) -> None:
        with self.print_lock:
            print(message, flush=True)

    def send(self, data: bytes, label: str) -> bool:
        try:
            with self.send_lock:
                self.connection.sendall(data)
        except OSError as error:
            self.print(f"[{timestamp()}] TX failed: {error}")
            self.stopped.set()
            return False

        self.print(f"[{timestamp()}] TX {label}")
        self.print(f"  ASCII: {format_ascii(data)}")
        return True

    def receive_loop(self) -> None:
        while not self.stopped.is_set():
            try:
                data = self.connection.recv(4096)
            except OSError as error:
                if not self.stopped.is_set():
                    self.print(f"[{timestamp()}] RX failed: {error}")
                break

            if not data:
                self.print(f"[{timestamp()}] peer disconnected")
                break

            self.print(f"[{timestamp()}] RX {len(data)} bytes")
            self.print(f"  ASCII: {format_ascii(data)}")

        self.stopped.set()

    def close(self) -> None:
        self.stopped.set()
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.connection.close()


def print_commands() -> None:
    print("Commands:")
    print("  rdsson          enable RDSS UART forwarding")
    print("  rdssoff         disable RDSS UART forwarding")
    print("  status          query forwarding state")
    print("  send <text>     send ASCII text followed by CRLF")
    print("  hex <bytes>     send raw hex bytes, e.g. hex 52 44 53 53")
    print("  help            show this list")
    print("  quit            close the connection and exit")


def interactive_loop(session: DebugSession) -> None:
    commands = {
        "rdsson": b"RDSS_FORWARD=1\r\n",
        "rdssoff": b"RDSS_FORWARD=0\r\n",
        "status": b"RDSS_FORWARD?\r\n",
    }

    print_commands()
    while not session.stopped.is_set():
        try:
            command = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not command:
            continue
        lowered = command.lower()
        if lowered in ("quit", "exit"):
            break
        if lowered == "help":
            print_commands()
            continue
        if lowered in commands:
            session.send(commands[lowered], lowered)
            continue
        if lowered.startswith("send "):
            text = command[5:]
            session.send(text.encode("utf-8") + b"\r\n", "text")
            continue
        if lowered.startswith("hex "):
            try:
                data = bytes.fromhex(command[4:])
            except ValueError as error:
                print(f"Invalid hex input: {error}")
                continue
            session.send(data, "raw hex")
            continue

        print("Unknown command. Type 'help' for supported commands.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive Ethernet data and debug RDSS UART forwarding"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="local IP to bind")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="local TCP port")
    parser.add_argument(
        "--no-auto-forward",
        action="store_true",
        help="do not enable RDSS forwarding automatically after connection",
    )
    args = parser.parse_args()

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)
            print(f"Listening on {args.host}:{args.port}")
            print("Close NetAssist or any other program using this port first.")

            connection, address = server.accept()
            print(f"Connected by {address[0]}:{address[1]}")

            session = DebugSession(connection)
            receiver = threading.Thread(target=session.receive_loop, daemon=True)
            receiver.start()

            if not args.no_auto_forward:
                session.send(b"RDSS_FORWARD=1\r\n", "auto rdsson")

            try:
                interactive_loop(session)
            finally:
                session.close()
                receiver.join(timeout=1.0)
    except OSError as error:
        print(f"Network error: {error}")
        print(f"Check that this PC owns {args.host} and TCP port {args.port} is free.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
