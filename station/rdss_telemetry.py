"""北斗短报文状态遥测：只解析已烧录固件的 UGRDS schema 2。"""

from datetime import datetime
import re


UGRDS_FIELDS = [
    "schema", "seq", "t_ms", "signal_good", "ready_to_send", "pending",
    "waiting_ack", "last_result", "attempt_count", "sent_count",
    "reject_count", "timeout_count", "uart_fail_count", "last_hal_status",
    "last_uart_error", "tx_mode", "card_id_known", "card_mismatch",
    "has_cached_fix", "send_ok_count", "local_latitude_deg_e7",
    "local_longitude_deg_e7", "target_valid", "goto_received_count",
    "target_latitude_deg_e7", "target_longitude_deg_e7",
]
UGRDS_INT_FIELDS = set(UGRDS_FIELDS)
UGRDS_CSV_HEADER = ["host_iso"] + UGRDS_FIELDS
UGRDS_RESULT_NAMES = {
    0: "IDLE", 1: "PENDING", 2: "WAITING_ACK", 3: "SUCCESS",
    4: "REJECTED", 5: "TIMEOUT", 6: "UART_ERROR",
}
_FLAGS = {
    "signal_good", "ready_to_send", "pending", "waiting_ack",
    "card_id_known", "card_mismatch", "has_cached_fix", "target_valid",
}
_COORDINATES = {
    "local_latitude_deg_e7": ("has_cached_fix", 900_000_000),
    "local_longitude_deg_e7": ("has_cached_fix", 1_800_000_000),
    "target_latitude_deg_e7": ("target_valid", 900_000_000),
    "target_longitude_deg_e7": ("target_valid", 1_800_000_000),
}
_FRAME_PATTERN = re.compile(r"\$(UGRDS,[-0-9,]+)\*([0-9a-fA-F]{2})(?:\r?\n)?")
_INTEGER_PATTERN = re.compile(r"-?[0-9]{1,10}")


def ugrds_parse(line):
    """严格检查整行 ASCII、校验、版本、位宽及有效坐标，不修补损坏帧。"""
    if not isinstance(line, str) or len(line) > 384:
        return None
    match = _FRAME_PATTERN.fullmatch(line)
    if match is None:
        return None
    body, checksum_text = match.groups()
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    if checksum != int(checksum_text, 16):
        return None
    values = body.split(",")[1:]
    if len(values) != len(UGRDS_FIELDS) or any(
            _INTEGER_PATTERN.fullmatch(value) is None for value in values):
        return None
    row = dict(zip(UGRDS_FIELDS, map(int, values)))
    if row["schema"] != 2:
        return None
    for field, value in row.items():
        if field in _COORDINATES:
            flag, limit = _COORDINATES[field]
            if not -(1 << 31) <= value < (1 << 31):
                return None
            # 无效定位可保留旧的原始值，但不得以有效定位显示或分析。
            if row[flag] == 1 and not -limit <= value <= limit:
                return None
        elif not 0 <= value <= 0xFFFFFFFF:
            return None
        if field in _FLAGS and value not in (0, 1):
            return None
    if (row["last_result"] not in UGRDS_RESULT_NAMES or
            row["tx_mode"] not in (0, 1, 2) or
            row["last_hal_status"] not in (0, 1, 2, 3)):
        return None
    return row


def ugrds_csv_row(row):
    """保留板端原始整数；host_iso 是主机记录墙钟，不是板端采样时间。"""
    return [datetime.now().isoformat(timespec="milliseconds")] + [
        row[name] for name in UGRDS_FIELDS]
