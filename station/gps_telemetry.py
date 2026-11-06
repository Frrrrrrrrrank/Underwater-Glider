"""自身 GNSS 与 EKF 融合诊断；UGPS schema 1，不复用 RDSS 缓存定位。"""

from datetime import datetime
import re


UGPS_FIELDS = [
    "schema", "seq", "t_ms", "state", "reason", "flags", "sample_seq",
    "rx_age_ms", "fuse_age_ms", "utc_date", "utc_ms", "lat_e7", "lon_e7",
    "origin_id", "origin_lat_e7", "origin_lon_e7", "meas_n_mm", "meas_e_mm",
    "est_n_mm", "est_e_mm", "innov_n_mm", "innov_e_mm", "nis_milli",
    "r_var_cm2", "pnn_cm2", "pne_cm2", "pee_cm2", "accept_count",
    "reject_count", "reset_count", "duplicate_count", "drop_count",
    "input_reject_count",
]
UGPS_INT_FIELDS = set(UGPS_FIELDS)
UGPS_CSV_HEADER = ["host_iso"] + UGPS_FIELDS
UGPS_STALE_MS = 3000
UGPS_UNKNOWN_AGE = 0xFFFFFFFF
UGPS_STATE_NAMES = {
    0: "WAIT_FIX", 1: "ACQUIRING", 2: "AIDING", 3: "COASTING",
    4: "REACQUIRING", 5: "WAIT_EKF",
}
UGPS_REASON_NAMES = {
    0: "NONE", 1: "BAD_INPUT", 2: "DUPLICATE", 3: "OUT_OF_ORDER",
    4: "STALE", 5: "NOT_READY", 6: "ACQUIRING", 7: "JUMP", 8: "NIS",
    9: "NUMERIC", 10: "RESET", 11: "IMU_STALE",
}
_SIGNED_FIELDS = {
    "lat_e7", "lon_e7", "origin_lat_e7", "origin_lon_e7", "meas_n_mm",
    "meas_e_mm", "est_n_mm", "est_e_mm", "innov_n_mm", "innov_e_mm", "pne_cm2",
}
_COORDINATES = {
    "lat_e7": (5, 900_000_000), "lon_e7": (5, 1_800_000_000),
    "origin_lat_e7": (0, 900_000_000), "origin_lon_e7": (0, 1_800_000_000),
}
_FRAME_PATTERN = re.compile(r"\$(UGPS,[-0-9,]+)\*([0-9a-fA-F]{2})(?:\r?\n)?")
_INTEGER_PATTERN = re.compile(r"-?[0-9]{1,10}")


def ugps_parse(line):
    """整行严格校验；无效位下保留有限原始整数，显示层不把占位零当测量。"""
    if not isinstance(line, str) or len(line) > 512:
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
    if len(values) != len(UGPS_FIELDS) or any(
            _INTEGER_PATTERN.fullmatch(value) is None for value in values):
        return None
    row = dict(zip(UGPS_FIELDS, map(int, values)))
    if (row["schema"] != 1 or row["state"] not in UGPS_STATE_NAMES or
            row["reason"] not in UGPS_REASON_NAMES or not 0 <= row["flags"] <= 255):
        return None
    for field, value in row.items():
        if field in _SIGNED_FIELDS:
            if not -(1 << 31) <= value < (1 << 31):
                return None
        elif not 0 <= value <= 0xFFFFFFFF:
            return None
        if field in _COORDINATES:
            bit, limit = _COORDINATES[field]
            if row["flags"] & (1 << bit) and not -limit <= value <= limit:
                return None
    return row


def ugps_invalid_fields(data):
    """CLI/Web 共用有效位规则；诊断帧新鲜不代表自身定位或融合刚发生。"""
    invalid = {}
    flags = data.get("flags")
    flags = flags if type(flags) is int and 0 <= flags <= 255 else 0

    def mark(names, reason):
        for name in names:
            if name in data:
                invalid.setdefault(name, reason)

    if not flags & 1:
        mark(("origin_lat_e7", "origin_lon_e7", "origin_id", "meas_n_mm", "meas_e_mm"),
             "gps_origin_unavailable")
    if not flags & 2:
        mark(("est_n_mm", "est_e_mm"), "gps_not_anchored")
    if not flags & 16:
        mark(("pnn_cm2", "pne_cm2", "pee_cm2"), "gps_covariance_unavailable")
    if not flags & 32:
        mark(("lat_e7", "lon_e7", "meas_n_mm", "meas_e_mm"), "gps_measurement_unavailable")
    if not flags & 64:
        mark(("innov_n_mm", "innov_e_mm", "nis_milli"), "gps_innovation_unavailable")
    if not flags & 128:
        mark(("fuse_age_ms",), "gps_never_fused")
    for field in ("rx_age_ms", "fuse_age_ms"):
        if data.get(field) == UGPS_UNKNOWN_AGE:
            mark((field,), "gps_age_unknown")
    for field, (_, limit) in _COORDINATES.items():
        value = data.get(field)
        if field in data and (type(value) is not int or not -limit <= value <= limit):
            mark((field,), "coordinate_out_of_range")
    return invalid


def ugps_csv_row(row):
    """保留板端定点整数及有效位；host_iso 只是 PC 记录时间。"""
    return [datetime.now().isoformat(timespec="milliseconds")] + [
        row[name] for name in UGPS_FIELDS]
