#!/usr/bin/env python3
"""
decode_shortmsg.py
------------------
将北斗短报文的十六进制载荷解码为可读字符串。

用法（命令行）：
    python decode_shortmsg.py
    python decode_shortmsg.py 68656C6C6F20686B7573740D0A
    python decode_shortmsg.py "68 65 6C 6C 6F" "244343524D4F"

支持格式：
    - 连续十六进制串：68656C6C6F
    - 空格 / 冒号 / 连字符分隔：68 65 6C 6C 6F / 68:65 / 68-65
    - 0x 前缀：0x68 0x65
    - 多条消息（每个命令行参数视为一条）
"""

import sys
import re

from show_location import open_location, parse_coordinates


CONTROL_CHARS = {
    0x00: "<NUL>",
    0x01: "<SOH>",
    0x02: "<STX>",
    0x03: "<ETX>",
    0x04: "<EOT>",
    0x05: "<ENQ>",
    0x06: "<ACK>",
    0x07: "<BEL>",
    0x08: "<BS>",
    0x09: "<TAB>",
    0x0A: "<LF>",
    0x0B: "<VT>",
    0x0C: "<FF>",
    0x0D: "<CR>",
    0x0E: "<SO>",
    0x0F: "<SI>",
    0x1B: "<ESC>",
    0x7F: "<DEL>",
}


def strip_hex_input(raw: str) -> str:
    # 去掉 0x / 0X 前缀、分隔符，只保留十六进制字符
    cleaned = re.sub(r"0[xX]", "", raw)
    cleaned = re.sub(r"[\s\-:,]+", "", cleaned)
    return cleaned.upper()


def decode_hex_payload(hex_str: str) -> tuple[bytes, str]:
    cleaned = strip_hex_input(hex_str)
    if len(cleaned) % 2 != 0:
        raise ValueError(f"十六进制长度为奇数（{len(cleaned)} 位），无法成对解码")
    raw_bytes = bytes.fromhex(cleaned)

    parts = []
    for b in raw_bytes:
        if 0x20 <= b <= 0x7E:
            parts.append(chr(b))
        elif b in CONTROL_CHARS:
            parts.append(CONTROL_CHARS[b])
        else:
            parts.append(f"<0x{b:02X}>")

    return raw_bytes, "".join(parts)


def format_hex_dump(raw_bytes: bytes) -> str:
    hex_part = " ".join(f"{b:02X}" for b in raw_bytes)
    ascii_part = "".join(chr(b) if 0x20 <= b <= 0x7E else "." for b in raw_bytes)
    return f"  HEX : {hex_part}\n  ASCII: {ascii_part}"


def process_one(label: str, hex_str: str) -> None:
    print(f"\n{'='*60}")
    print(f"  输入 : {hex_str.strip()}")
    if label:
        print(f"  标签 : {label}")
    try:
        raw_bytes, decoded = decode_hex_payload(hex_str)
        print(f"  长度 : {len(raw_bytes)} 字节")
        print(format_hex_dump(raw_bytes))
        print(f"  解码 : {decoded}")

        ascii_text = raw_bytes.decode("ascii", errors="ignore").strip()
        if re.search(r"\bLAT\s*=", ascii_text, re.IGNORECASE) and re.search(
            r"\bLON\s*=", ascii_text, re.IGNORECASE
        ):
            try:
                latitude, longitude = parse_coordinates(ascii_text)
                print("  坐标 : 有效，正在打开百度地图")
                if not open_location(latitude, longitude):
                    print("  地图 : 浏览器未能自动打开，请使用上面输出的网址")
            except ValueError as error:
                print(f"  坐标 : 无效（{error}）")

        # 尝试 UTF-8 解码（适用于含中文的载荷）
        try:
            utf8_decoded = raw_bytes.decode("utf-8")
            if utf8_decoded != decoded.replace("<CR>", "\r").replace("<LF>", "\n"):
                printable = repr(utf8_decoded)
                print(f"  UTF-8: {printable}")
        except UnicodeDecodeError:
            pass
    except ValueError as e:
        print(f"  错误 : {e}")


def interactive_mode() -> None:
    print("北斗短报文十六进制解码工具")
    print("输入十六进制内容后回车解码，输入 q 或 quit 退出")
    print("-" * 60)
    while True:
        try:
            line = input("\n请输入十六进制载荷: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            print("退出")
            break
        process_one("", line)


def main() -> None:
    args = sys.argv[1:]
    if not args:
        interactive_mode()
        return

    for i, arg in enumerate(args):
        process_one(f"参数 {i + 1}", arg)

    print(f"\n{'='*60}")


if __name__ == "__main__":
    main()
