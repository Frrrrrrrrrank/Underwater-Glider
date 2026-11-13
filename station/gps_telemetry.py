"""自身 GNSS 与 EKF 融合诊断；兼容 UGPS schema 1/2，不复用 RDSS 缓存定位。"""

from datetime import datetime
import re


UGPS_SCHEMA1_FIELDS = [
    "schema", "seq", "t_ms", "state", "reason", "flags", "sample_seq",
    "rx_age_ms", "fuse_age_ms", "utc_date", "utc_ms", "lat_e7", "lon_e7",
    "origin_id", "origin_lat_e7", "origin_lon_e7", "meas_n_mm", "meas_e_mm",
    "est_n_mm", "est_e_mm", "innov_n_mm", "innov_e_mm", "nis_milli",
    "r_var_cm2", "pnn_cm2", "pne_cm2", "pee_cm2", "accept_count",
    "reject_count", "reset_count", "duplicate_count", "drop_count",
    "input_reject_count",
]
UGPS_SCHEMA2_EXTRA_FIELDS = [
    "quality_source", "velocity_source", "vel_fuse_age_ms", "no_fix_count",
    "meas_vn_mmps", "meas_ve_mmps", "est_vn_mmps", "est_ve_mmps",
    "vel_nis_milli", "vel_r_var_cm2ps2", "pvn_cm2ps2", "pvne_cm2ps2",
    "pve_cm2ps2", "vel_accept_count", "vel_reject_count", "recovery_reason",
    "reset_before_n_mm", "reset_before_e_mm", "reset_before_vn_mmps",
    "reset_before_ve_mmps", "reset_after_n_mm", "reset_after_e_mm",
    "reset_after_vn_mmps", "reset_after_ve_mmps", "reset_age_ms", "reset_valid_mask",
]
UGPS_FIELDS = UGPS_SCHEMA1_FIELDS + UGPS_SCHEMA2_EXTRA_FIELDS
UGPS_FIELDS_BY_SCHEMA = {1: UGPS_SCHEMA1_FIELDS, 2: UGPS_FIELDS}
UGPS_INT_FIELDS = set(UGPS_FIELDS)
UGPS_CSV_HEADER = ["host_iso"] + UGPS_FIELDS
UGPS_STALE_MS = 3000
UGPS_UNKNOWN_AGE = 0xFFFFFFFF
UGPS_MAX_LINE = 1024
UGPS_SCHEMA1_STATE_NAMES = {
    0: "WAIT_FIX", 1: "ACQUIRING", 2: "AIDING", 3: "COASTING",
    4: "REACQUIRING", 5: "WAIT_EKF",
}
UGPS_STATE_NAMES = {
    0: "WAIT_FIX", 1: "POSITION_ONLY", 2: "TRACKING", 3: "DR_UNTRUSTED",
    4: "REACQUIRING", 5: "WAIT_EKF",
}
UGPS_REASON_NAMES = {
    0: "NONE", 1: "BAD_INPUT", 2: "DUPLICATE", 3: "OUT_OF_ORDER",
    4: "STALE", 5: "NOT_READY", 6: "ACQUIRING", 7: "JUMP", 8: "NIS",
    9: "NUMERIC", 10: "RESET", 11: "IMU_STALE", 12: "NO_FIX",
    13: "VELOCITY_NIS", 14: "QUALITY",
}
UGPS_RECOVERY_NAMES = {0: "NONE", 1: "FIRST_FIX", 2: "AID_LOSS",
                       3: "FILTER_RESET", 4: "NUMERIC", 5: "REJECT_RECOVERY"}
UGPS_RESET_FIELDS = [
    "reset_before_n_mm", "reset_before_e_mm", "reset_before_vn_mmps",
    "reset_before_ve_mmps", "reset_after_n_mm", "reset_after_e_mm",
    "reset_after_vn_mmps", "reset_after_ve_mmps",
]
_SIGNED_FIELDS = {
    "lat_e7", "lon_e7", "origin_lat_e7", "origin_lon_e7", "meas_n_mm",
    "meas_e_mm", "est_n_mm", "est_e_mm", "innov_n_mm", "innov_e_mm", "pne_cm2",
    "meas_vn_mmps", "meas_ve_mmps", "est_vn_mmps", "est_ve_mmps", "pvne_cm2ps2",
} | set(UGPS_RESET_FIELDS)
_COORDINATES = {
    "lat_e7": (5, 900_000_000), "lon_e7": (5, 1_800_000_000),
    "origin_lat_e7": (0, 900_000_000), "origin_lon_e7": (0, 1_800_000_000),
}
_FRAME_PATTERN = re.compile(r"\$(UGPS,[-0-9,]+)\*([0-9a-fA-F]{2})(?:\r?\n)?")
_INTEGER_PATTERN = re.compile(r"-?[0-9]{1,10}")


def ugps_parse(line):
    """按 schema 严格校验；旧帧保留原字段，新字段缺失不伪造成零值。"""
    if not isinstance(line, str) or len(line) > UGPS_MAX_LINE:
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
    if not values or any(
            _INTEGER_PATTERN.fullmatch(value) is None for value in values):
        return None
    schema = int(values[0])
    fields = UGPS_FIELDS_BY_SCHEMA.get(schema)
    if fields is None or len(values) != len(fields) or (schema == 1 and len(line) > 512):
        return None
    row = dict(zip(fields, map(int, values)))
    if (row["state"] not in UGPS_STATE_NAMES or
            not 0 <= row["reason"] <= (11 if schema == 1 else 14) or
            not 0 <= row["flags"] <= (255 if schema == 1 else 8191)):
        return None
    if schema == 2 and (row["quality_source"] not in (0, 1) or
                        row["velocity_source"] not in (0, 1, 2) or
                        row["recovery_reason"] not in UGPS_RECOVERY_NAMES or
                        not 0 <= row["reset_valid_mask"] <= 255):
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
    schema2 = data.get("schema") == 2
    flags = flags if type(flags) is int and 0 <= flags <= (8191 if schema2 else 255) else 0

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
    if schema2:
        if not flags & 256:
            mark(("est_vn_mmps", "est_ve_mmps"), "gps_velocity_untrusted")
        if not flags & 512:
            mark(("meas_vn_mmps", "meas_ve_mmps"), "gps_velocity_measurement_unavailable")
        if not flags & 2048:
            mark(("pvn_cm2ps2", "pvne_cm2ps2", "pve_cm2ps2"), "gps_velocity_covariance_unavailable")
        if not flags & 4096:
            mark(("vel_nis_milli",), "gps_velocity_innovation_unavailable")
        if not flags & 1024:
            mark(tuple(UGPS_RESET_FIELDS) + ("reset_age_ms", "recovery_reason"),
                 "gps_recovery_unavailable")
        else:
            mask = data.get("reset_valid_mask")
            mask = mask if type(mask) is int and 0 <= mask <= 255 else 0
            for index, name in enumerate(UGPS_RESET_FIELDS):
                if not mask & (1 << index):
                    mark((name,), "gps_recovery_value_unavailable")
    for field in ("rx_age_ms", "fuse_age_ms", "vel_fuse_age_ms", "reset_age_ms"):
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
        row.get(name, "") for name in UGPS_FIELDS]
