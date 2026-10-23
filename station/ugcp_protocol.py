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
MSG_CASCADE_COMMAND = 0x05
# 蜗杆回零：payload 1 字节，1=开始，0=取消
MSG_WORM_CALIB = 0x06
# 尾舵连续控制：旧 CONTROL19 末尾追加有符号百分之一度。
MSG_CONTROL21_STREAM = 0x07
MSG_PARAM_BLOCK = 0x10
MSG_PARAM_COMMIT = 0x11

STREAM_EKF = 0x01
STREAM_MAG = 0x02
STREAM_CTL = 0x04
# bit3：蜗杆位置遥测 $UGWRM（装机后 Ozone 不可用，位置量只能靠这条流看）
STREAM_WRM = 0x08
# bit4：电池电压及低压状态遥测 $UGBAT。
STREAM_BAT = 0x10
STREAM_RDSS = 0x20
STREAM_ALL = STREAM_EKF | STREAM_MAG | STREAM_CTL | STREAM_WRM | STREAM_BAT | STREAM_RDSS

PARAM_GROUP_MAG_CAL = 1
PARAM_GROUP_PID = 2
MAG_CAL_BLOCK_COUNT = 4
PID_PARAM_BLOCK_COUNT = 5
# 兼容磁标定工具的旧名称；PID 发送必须使用 PID_PARAM_BLOCK_COUNT。
PARAM_BLOCK_COUNT = MAG_CAL_BLOCK_COUNT
PARAM_BLOCK_DATA_SIZE = 16
PARAM_TABLE_SIZE = PARAM_BLOCK_COUNT * PARAM_BLOCK_DATA_SIZE
MAG_CAL_TABLE_SIZE = 64
PID_PARAM_TABLE_SIZE = 80

CASCADE_SCHEMA_V1 = 1
CASCADE_MODE_OFF = 0
CASCADE_MODE_PITCH_HOLD = 1
CASCADE_MODE_DEPTH = 2
CASCADE_FLAG_ARM = 0x01

PID_PARAM_FIELDS = (
    "buoyancy_kp_mm_per_m",
    "buoyancy_ki_mm_per_m_s",
    "buoyancy_kd_mm_s_per_m",
    "buoyancy_integral_limit_m_s",
    "buoyancy_derivative_tau_s",
    "pitch_kp_rpm_per_deg",
    "pitch_ki_rpm_per_deg_s",
    "pitch_kd_rpm_s_per_deg",
    "pitch_integral_limit_deg_s",
    "pitch_derivative_tau_s",
    "depth_to_pitch_alpha_deg_per_m",
    "max_pitch_deg",
    "max_worm_rpm",
    "max_thrust_percent",
    "surface_depth_m",
    "depth_switch_threshold_m",
    "pitch_actuator_sign",
    "displacement_neutral_mm",
)

_CORE_HEADER = struct.Struct("<BBHH")
_CRC32 = struct.Struct("<I")
_CONTROL19 = struct.Struct("<BiBiBBBBBBBBB")
_RUDDER_CDEG = struct.Struct("<h")
_CASCADE_COMMAND = struct.Struct("<BBBBHHfffI")
_PARAM_BLOCK = struct.Struct("<BBB16s")
_PARAM_COMMIT = struct.Struct("<BBBI")
_MAG_CAL_TABLE = struct.Struct("<16f")
_PID_PARAM_TABLE = struct.Struct("<20f")


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


def _require_u16(value, name):
    value = int(value)
    if not 0 <= value <= 0xFFFF:
        raise UgcpProtocolError("%s 必须在 0..65535" % name)
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


def pack_control21_payload(pitchmotor_enable, pitchmotor_speed_rpm,
                           rollmotor_enable, rollmotor_speed_rpm,
                           thrustmotor_direction, thrustmotor_direction2,
                           thrustmotor1se_speed_rpm, thrustmotor2se_speed_rpm,
                           backup_for_thrust, pump_direction, pump_on_off,
                           ask_for_status, eth_control_flag, rudder_angle_cdeg):
    """编码尾舵绝对目标；前 19 字节保留旧布局，末尾为 int16 小端百分之一度。"""
    try:
        angle = int(rudder_angle_cdeg)
    except (TypeError, ValueError, OverflowError):
        raise UgcpProtocolError("rudder_angle_cdeg 必须为有限整数") from None
    if angle != rudder_angle_cdeg or not -4500 <= angle <= 4500:
        raise UgcpProtocolError("rudder_angle_cdeg 必须为 -4500..4500 的整数")
    control = pack_control_payload(
        pitchmotor_enable, pitchmotor_speed_rpm,
        rollmotor_enable, rollmotor_speed_rpm,
        thrustmotor_direction, thrustmotor_direction2,
        thrustmotor1se_speed_rpm, thrustmotor2se_speed_rpm,
        backup_for_thrust, pump_direction, pump_on_off,
        ask_for_status, eth_control_flag,
    )
    return control + _RUDDER_CDEG.pack(angle)


def pack_cascade_command_payload(
        mode, armed, command_id, target_depth_m, target_pitch_deg,
        signed_thrust_percent, required_pid_crc):
    """编码双环控制命令；角度在线路上用度，板端服务层再转换为弧度。"""
    mode = _require_u8(mode, "mode")
    if mode not in (
            CASCADE_MODE_OFF, CASCADE_MODE_PITCH_HOLD, CASCADE_MODE_DEPTH):
        raise UgcpProtocolError("mode 只能是 OFF/PITCH_HOLD/DEPTH")

    values = _float_vector(
        (target_depth_m, target_pitch_deg, signed_thrust_percent),
        3,
        "cascade_command",
    )
    if not 0.0 <= values[0] <= 1000.0:
        raise UgcpProtocolError("target_depth_m 必须在 0..1000")
    if abs(values[1]) > 90.0:
        raise UgcpProtocolError("target_pitch_deg 绝对值不能超过 90")
    if abs(values[2]) > 100.0:
        raise UgcpProtocolError("signed_thrust_percent 绝对值不能超过 100")

    flags = CASCADE_FLAG_ARM if _require_bit(armed, "armed") else 0
    return _CASCADE_COMMAND.pack(
        CASCADE_SCHEMA_V1,
        mode,
        flags,
        0,
        _require_u16(command_id, "command_id"),
        0,
        values[0],
        values[1],
        values[2],
        _require_u32(required_pid_crc, "required_pid_crc"),
    )


def unpack_cascade_command_payload(payload):
    """严格解码 24 字节双环控制命令，主要供界面显示和回归测试。"""
    payload = bytes(payload)
    if len(payload) != _CASCADE_COMMAND.size:
        raise UgcpProtocolError("双环控制命令必须是 24 字节")

    (schema, mode, flags, reserved8, command_id, reserved16,
     target_depth_m, target_pitch_deg, signed_thrust_percent,
     required_pid_crc) = _CASCADE_COMMAND.unpack(payload)
    if schema != CASCADE_SCHEMA_V1:
        raise UgcpProtocolError("不支持的双环控制命令 schema=%d" % schema)
    if reserved8 != 0 or reserved16 != 0 or flags & ~CASCADE_FLAG_ARM:
        raise UgcpProtocolError("双环控制命令保留位必须为 0")

    # 复用编码器的全部有限值和安全范围校验。
    pack_cascade_command_payload(
        mode,
        bool(flags & CASCADE_FLAG_ARM),
        command_id,
        target_depth_m,
        target_pitch_deg,
        signed_thrust_percent,
        required_pid_crc,
    )
    return {
        "mode": mode,
        "armed": bool(flags & CASCADE_FLAG_ARM),
        "command_id": command_id,
        "target_depth_m": target_depth_m,
        "target_pitch_deg": target_pitch_deg,
        "signed_thrust_percent": signed_thrust_percent,
        "required_pid_crc": required_pid_crc,
    }


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


def pack_pid_param_table(parameters):
    """编码 PID 参数组 2；输入可为按 ``PID_PARAM_FIELDS`` 排列的序列或字典。"""
    if hasattr(parameters, "keys"):
        missing = [name for name in PID_PARAM_FIELDS if name not in parameters]
        extra = [name for name in parameters if name not in PID_PARAM_FIELDS]
        if missing or extra:
            raise UgcpProtocolError(
                "PID 参数字段不匹配，缺少=%s，多余=%s" % (missing, extra))
        source = [parameters[name] for name in PID_PARAM_FIELDS]
    else:
        source = parameters

    values = _float_vector(source, len(PID_PARAM_FIELDS), "pid_parameters")
    if any(value < 0.0 for value in values[:10]):
        raise UgcpProtocolError("PID 增益、积分限幅和微分滤波时间不得为负")
    if not 0.0 <= values[10] <= 90.0:
        raise UgcpProtocolError("depth_to_pitch_alpha_deg_per_m 必须在 [0, 90]")
    if not 0.0 < values[11] <= 90.0:
        raise UgcpProtocolError("max_pitch_deg 必须在 (0, 90]")
    if not 0.0 < values[12] <= 600.0:
        raise UgcpProtocolError("max_worm_rpm 必须在 (0, 600]")
    if not 0.0 <= values[13] <= 100.0:
        raise UgcpProtocolError("max_thrust_percent 必须在 [0, 100]")
    if values[14] < -0.2 or values[15] < 0.0:#values[14]是surface_depth_m
        raise UgcpProtocolError("深度阈值不得为负")
    if values[16] not in (-1.0, 1.0):
        raise UgcpProtocolError("pitch_actuator_sign 只能是 -1 或 +1")

    try:
        # neutral 的物理范围故意留给板端硬宏校验，以便 push 后返回专用原因码。
        table = _PID_PARAM_TABLE.pack(*(values + [0.0, 0.0]))
    except (OverflowError, struct.error) as exc:
        raise UgcpProtocolError("PID 参数不能表示为 float32") from exc
    if not all(math.isfinite(value) for value in _PID_PARAM_TABLE.unpack(table)):
        raise UgcpProtocolError("PID 参数转换为 float32 后出现 NaN/Inf")
    return table


def unpack_pid_param_table(table):
    table = bytes(table)
    if len(table) != PID_PARAM_TABLE_SIZE:
        raise UgcpProtocolError("PID 参数表必须是 80 字节")
    values = _PID_PARAM_TABLE.unpack(table)
    if values[18:] != (0.0, 0.0):
        raise UgcpProtocolError("PID 参数表保留字段必须为 0")
    # 通过同一入口验证板端会接受该表，再返回带单位的字段名。
    pack_pid_param_table(values[:len(PID_PARAM_FIELDS)])
    return dict(zip(PID_PARAM_FIELDS, values[:len(PID_PARAM_FIELDS)]))


def split_param_table(table):
    table = bytes(table)
    if len(table) not in (MAG_CAL_TABLE_SIZE, PID_PARAM_TABLE_SIZE):
        raise UgcpProtocolError("参数表必须是 64 或 80 字节")
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
    """编码完整遥测状态：bit0..4 依次为 EKF、MAG、CTL、WRM、BAT。"""
    mask = _require_u8(mask, "stream_mask")
    if mask & ~STREAM_ALL:
        raise UgcpProtocolError("stream_mask 只能使用 EKF/MAG/CTL/WRM/BAT 五个 bit（0..31）")
    return bytes((mask,))


def pack_worm_calib_payload(start):
    """蜗杆回零命令：True=开始回零，False=取消。板端只置标志，动作在 ServoTask 里。"""
    return bytes((1 if start else 0,))


def pack_telem_control_payload(enabled):
    """旧布尔接口：关闭全部，或只开启 $UGEKF（不会顺带开启 $UGMAG）。"""
    enabled = _require_bit(enabled, "enabled")
    return pack_stream_control_payload(STREAM_EKF if enabled else 0)


def pack_snapshot_request_payload():
    return b""


assert _CONTROL19.size == 19
assert _CONTROL19.size + _RUDDER_CDEG.size == 21
assert _CASCADE_COMMAND.size == 24
assert _PARAM_BLOCK.size == 19
assert _PARAM_COMMIT.size == 7
assert _MAG_CAL_TABLE.size == MAG_CAL_TABLE_SIZE
assert _PID_PARAM_TABLE.size == PID_PARAM_TABLE_SIZE
