"""UART7 原始行外包；只验证传输合同，不筛选内层 NMEA 或修补坏字节。"""

import re


UGRAW_MAX_BYTES = 384
UGRAW_MAX_LINE = 1024
UGRAW_RENEW_S = 5.0
UGRAW_WIRE_FIELDS = (
    "schema", "capture_id", "seq", "rx_us_hi", "rx_us_lo", "tx_us_hi", "tx_us_lo",
    "queue_drop", "uart_drop", "line_drop", "flags", "src_len", "kept_len",
)
UGRAW_FIELDS = UGRAW_WIRE_FIELDS + ("rx_us", "tx_us", "payload_hex")
UGRAW_CSV_HEADER = ["received_at", "t_ns"] + list(UGRAW_FIELDS) + ["raw"]
_FRAME = re.compile(r"\$(UGRAW,[0-9,]+,[0-9a-fA-F]*)\*([0-9a-fA-F]{2})(?:\r?\n)?")
_INTEGER = re.compile(r"[0-9]{1,10}")


def ugraw_parse(line):
    """返回保留原文的字典；坏外包返回 None，空/非ASCII/NUL载荷仍可记录。"""
    if not isinstance(line, str) or len(line) > UGRAW_MAX_LINE:
        return None
    match = _FRAME.fullmatch(line)
    if match is None:
        return None
    body, check = match.groups()
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    if checksum != int(check, 16):
        return None
    fields = body.split(",")[1:]
    if len(fields) != 14 or any(_INTEGER.fullmatch(value) is None for value in fields[:-1]):
        return None
    values = list(map(int, fields[:-1]))
    if any(value > 0xFFFFFFFF for value in values):
        return None
    row = dict(zip(UGRAW_WIRE_FIELDS, values))
    if row["schema"] != 1 or row["flags"] not in (0, 1):
        return None
    if (row["kept_len"] != min(row["src_len"], UGRAW_MAX_BYTES) or
            row["flags"] != int(row["src_len"] > row["kept_len"]) or
            len(fields[-1]) != 2 * row["kept_len"]):
        return None
    row["payload_hex"] = fields[-1].lower()
    row["rx_us"] = (row["rx_us_hi"] << 32) | row["rx_us_lo"]
    row["tx_us"] = (row["tx_us_hi"] << 32) | row["tx_us_lo"]
    row["raw"] = line
    return row


def ugraw_csv_row(row):
    """主机时间由接收入口填写，后台写盘时间不冒充收包时间。"""
    return ["", ""] + [row.get(name, "") for name in UGRAW_FIELDS] + [row.get("raw", "")]
