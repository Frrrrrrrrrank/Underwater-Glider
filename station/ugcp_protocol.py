"""UGCP v1 上位机公共构帧工具。

帧格式：``UGCP | version | type | seq:u16LE | len:u16LE | payload | crc32:u32LE``。
CRC 覆盖 version 到 payload 末尾，与板端及 ``zlib.crc32`` 一致。
"""

import math
import struct
import threading
import zlib


UGCP_MAGIC = b"UGCP"
UGCP_VERSION = 1
UGCP_MAX_PAYLOAD = 64

MSG_CONTROL19_STREAM = 0x01
MSG_CONTROL19 = MSG_CONTROL19_STREAM
MSG_STREAM_CONTROL = 0x02
MSG_TELEM_CONTROL = MSG_STREAM_CONTROL
MSG_SNAPSHOT_REQUEST = 0x03
MSG_CONTROL19_LATCHED = 0x04
MSG_PARAM_BLOCK = 0x10
MSG_PARAM_COMMIT = 0x11

STREAM_EKF = 0x01
STREAM_MAG = 0x02
STREAM_ALL = STREAM_EKF | STREAM_MAG

PARAM_GROUP_MAG_CAL = 1
PARAM_BLOCK_COUNT = 4
PARAM_BLOCK_DATA_SIZE = 16
MAG_CAL_TABLE_SIZE = 64

_CORE_HEADER = struct.Struct("<BBHH")
_CRC32 = struct.Struct("<I")
_CONTROL19 = struct.Struct("<BiBiBBBBBBBBB")
_PARAM_BLOCK = struct.Struct("<BBB16s")
_PARAM_COMMIT = struct.Struct("<BBBI")
_MAG_CAL_TABLE = struct.Struct("<16f")


class UgcpProtocolError(ValueError):
    """字段或帧不符合 UGCP v1。"""


class SequenceCounter:
    """线程安全的 16 位发送序号；到 65535 后自然回绕。"""

    def __init__(self, initial=0):
        initial = int(initial)
        if not 0 <= initial <= 0xFFFF:
            raise UgcpProtocolError("initial 必须在 0..65535")
        self._next = initial
        self._lock = threading.Lock()

    def next(self):
        with self._lock:
            value = self._next
            self._next = (self._next + 1) & 0xFFFF
            return value


def crc32_iso_hdlc(data):
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF


def build_frame(msg_type, payload=b"", seq=0):
    payload = bytes(payload)
    msg_type = int(msg_type)
    seq = int(seq)
    if not 0 <= msg_type <= 0xFF:
        raise UgcpProtocolError("msg_type 必须在 0..255")
    if not 0 <= seq <= 0xFFFF:
        raise UgcpProtocolError("seq 必须在 0..65535")
    if len(payload) > UGCP_MAX_PAYLOAD:
        raise UgcpProtocolError("payload 超过 %d 字节" % UGCP_MAX_PAYLOAD)

    core = _CORE_HEADER.pack(UGCP_VERSION, msg_type, seq, len(payload)) + payload
    return UGCP_MAGIC + core + _CRC32.pack(crc32_iso_hdlc(core))


def parse_frame(frame):
    """严格解析一整帧，返回 (msg_type, seq, payload)。"""
    frame = bytes(frame)
    minimum_size = len(UGCP_MAGIC) + _CORE_HEADER.size + _CRC32.size
    if len(frame) < minimum_size:
        raise UgcpProtocolError("UGCP 帧被截断")
    if frame[:len(UGCP_MAGIC)] != UGCP_MAGIC:
        raise UgcpProtocolError("UGCP magic 不匹配")

    version, msg_type, seq, payload_length = _CORE_HEADER.unpack_from(
        frame, len(UGCP_MAGIC))
    if version != UGCP_VERSION:
        raise UgcpProtocolError("不支持的 UGCP version=%d" % version)
    if payload_length > UGCP_MAX_PAYLOAD:
        raise UgcpProtocolError("payload 长度超过上限")

    payload_start = len(UGCP_MAGIC) + _CORE_HEADER.size
    payload_end = payload_start + payload_length
    expected_size = payload_end + _CRC32.size
    if len(frame) != expected_size:
        raise UgcpProtocolError(
            "UGCP 总长不匹配: %d != %d" % (len(frame), expected_size))

    expected_crc = _CRC32.unpack_from(frame, payload_end)[0]
    actual_crc = crc32_iso_hdlc(frame[len(UGCP_MAGIC):payload_end])
    if actual_crc != expected_crc:
        raise UgcpProtocolError(
            "UGCP CRC32 不匹配: 0x%08X != 0x%08X"
            % (actual_crc, expected_crc))
    return msg_type, seq, frame[payload_start:payload_end]


def _require_u8(value, name):
    value = int(value)
    if not 0 <= value <= 0xFF:
        raise UgcpProtocolError("%s 必须在 0..255" % name)
    return value


def _require_bit(value, name):
    value = int(value)
    if value not in (0, 1):
        raise UgcpProtocolError("%s 只能是 0/1" % name)
    return value


def _require_i32(value, name):
    value = int(value)
    if not -0x80000000 <= value <= 0x7FFFFFFF:
        raise UgcpProtocolError("%s 必须在 int32 范围" % name)
    return value


def _require_u32(value, name):
    value = int(value)
    if not 0 <= value <= 0xFFFFFFFF:
        raise UgcpProtocolError("%s 必须在 uint32 范围" % name)
    return value


def _require_speed_percent(value, name):
    value = int(value)
    if not 0 <= value <= 100:
        raise UgcpProtocolError("%s 必须在 0..100" % name)
    return value


def pack_control_payload(pitchmotor_enable, pitchmotor_speed_rpm,
                         rollmotor_enable, rollmotor_speed_rpm,
                         thrustmotor_direction, thrustmotor_direction2,
                         thrustmotor1se_speed_rpm, thrustmotor2se_speed_rpm,
                         backup_for_thrust, pump_direction, pump_on_off,
                         ask_for_status, eth_control_flag):
    """编码板端 CONTROL19；两个 RPM 为有符号 int32 little-endian。"""
    return _CONTROL19.pack(
        _require_bit(pitchmotor_enable, "pitchmotor_enable"),
        _require_i32(pitchmotor_speed_rpm, "pitchmotor_speed_rpm"),
        _require_bit(rollmotor_enable, "rollmotor_enable"),
        _require_i32(rollmotor_speed_rpm, "rollmotor_speed_rpm"),
        _require_bit(thrustmotor_direction, "thrustmotor_direction"),
        _require_bit(thrustmotor_direction2, "thrustmotor_direction2"),
        _require_speed_percent(thrustmotor1se_speed_rpm, "thrustmotor1se_speed_rpm"),
        _require_speed_percent(thrustmotor2se_speed_rpm, "thrustmotor2se_speed_rpm"),
        _require_bit(backup_for_thrust, "backup_for_thrust"),
        _require_bit(pump_direction, "pump_direction"),
        _require_bit(pump_on_off, "pump_on_off"),
        _require_bit(ask_for_status, "ask_for_status"),
        _require_bit(eth_control_flag, "eth_control_flag"),
    )


def _float_vector(values, expected_length, name):
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise UgcpProtocolError("%s 必须是数值序列" % name) from exc
    if len(result) != expected_length:
        raise UgcpProtocolError(
            "%s 长度必须是 %d" % (name, expected_length))
    if not all(math.isfinite(value) for value in result):
        raise UgcpProtocolError("%s 不允许 NaN/Inf" % name)
    return result


def pack_mag_cal_table(hard_iron_ut, soft_iron, reserved=None):
    """编码磁标定表；校验规则与板端 decodeMagCalibration 保持一致。"""
    hard = _float_vector(hard_iron_ut, 3, "hard_iron_ut")
    if any(abs(value) > 800.0 for value in hard):
        raise UgcpProtocolError("hard_iron_ut 绝对值不能超过 800uT")

    try:
        rows = list(soft_iron)
    except TypeError as exc:
        raise UgcpProtocolError("soft_iron 必须是 3x3 数值矩阵") from exc
    if len(rows) != 3:
        raise UgcpProtocolError("soft_iron 必须恰好有 3 行")
    matrix = [
        _float_vector(row, 3, "soft_iron[%d]" % index)
        for index, row in enumerate(rows)
    ]
    if any(abs(value) > 5.0 for row in matrix for value in row):
        raise UgcpProtocolError("soft_iron 元素绝对值不能超过 5")

    for row in range(3):
        radius = 0.0
        for column in range(3):
            if row == column:
                continue
            if abs(matrix[row][column] - matrix[column][row]) > 1.0e-3:
                raise UgcpProtocolError("soft_iron 必须近似对称")
            radius += abs(matrix[row][column])
        if (matrix[row][row] - radius < 5.0e-2 or
                matrix[row][row] + radius > 5.0):
            raise UgcpProtocolError("soft_iron 增益范围或条件性不安全")

    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                        matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                          matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                          matrix[1][1] * matrix[2][0])
    )
    if not math.isfinite(determinant) or determinant < 1.0e-4:
        raise UgcpProtocolError("soft_iron 行列式过小或非正")

    reserved_values = (
        [0.0, 0.0, 0.0, 0.0]
        if reserved is None
        else _float_vector(reserved, 4, "reserved")
    )
    for value in reserved_values:
        if struct.pack("<f", value) != bytes(4):
            raise UgcpProtocolError("reserved 必须是 +0.0 的全零位模式")

    flat_matrix = [value for row in matrix for value in row]
    table = _MAG_CAL_TABLE.pack(*(hard + flat_matrix + reserved_values))
    assert len(table) == MAG_CAL_TABLE_SIZE
    return table


def unpack_mag_cal_table(table):
    table = bytes(table)
    if len(table) != MAG_CAL_TABLE_SIZE:
        raise UgcpProtocolError("磁标定表必须是 64 字节")
    values = _MAG_CAL_TABLE.unpack(table)
    hard = values[:3]
    matrix = tuple(
        tuple(values[3 + row * 3:3 + (row + 1) * 3])
        for row in range(3)
    )
    return hard, matrix, values[12:16]


def split_param_table(table):
    table = bytes(table)
    if len(table) != MAG_CAL_TABLE_SIZE:
        raise UgcpProtocolError("当前参数表必须是 64 字节")
    return tuple(
        table[offset:offset + PARAM_BLOCK_DATA_SIZE]
        for offset in range(0, len(table), PARAM_BLOCK_DATA_SIZE)
    )


def pack_param_block_payload(group, transaction, block_index, data16):
    data16 = bytes(data16)
    if len(data16) != PARAM_BLOCK_DATA_SIZE:
        raise UgcpProtocolError("参数块数据必须是 16 字节")
    return _PARAM_BLOCK.pack(
        _require_u8(group, "group"),
        _require_u8(transaction, "transaction"),
        _require_u8(block_index, "block_index"),
        data16,
    )


def pack_param_commit_payload(group, transaction, block_count, table_crc):
    return _PARAM_COMMIT.pack(
        _require_u8(group, "group"),
        _require_u8(transaction, "transaction"),
        _require_u8(block_count, "block_count"),
        _require_u32(table_crc, "table_crc"),
    )


def pack_stream_control_payload(mask):
    """编码完整遥测状态：bit0=$UGEKF，bit1=$UGMAG。"""
    mask = _require_u8(mask, "stream_mask")
    if mask & ~STREAM_ALL:
        raise UgcpProtocolError("stream_mask 只能使用 EKF/MAG 两个 bit（0..3）")
    return bytes((mask,))


def pack_telem_control_payload(enabled):
    """旧布尔接口：关闭全部，或只开启 $UGEKF（不会顺带开启 $UGMAG）。"""
    enabled = _require_bit(enabled, "enabled")
    return pack_stream_control_payload(STREAM_EKF if enabled else 0)


def pack_snapshot_request_payload():
    return b""


assert _CONTROL19.size == 19
assert _PARAM_BLOCK.size == 19
assert _PARAM_COMMIT.size == 7
assert _MAG_CAL_TABLE.size == MAG_CAL_TABLE_SIZE
