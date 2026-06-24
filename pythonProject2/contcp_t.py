import socket
import struct
import threading
import time
import sys
import ctypes

# ==================== 全局标志 ====================
running = True          # 控制线程退出

UMOTOR_NODE_COUNT = 4
UMOTOR_MAX_READ_REG_COUNT = 16
UMOTOR_NODE_DEBUG_RX_MAX = 40


class DisplacementSensorData(ctypes.Structure):
    _fields_ = [
        ("displacement_mm", ctypes.c_float),
        ("sensor_id", ctypes.c_uint8),
        ("valid", ctypes.c_uint32),
    ]


class DisplacementSensorDiag(ctypes.Structure):
    _fields_ = [
        ("last_error", ctypes.c_uint32),
        ("comm_ok", ctypes.c_uint32),
        ("tx_count", ctypes.c_uint32),
        ("rx_count", ctypes.c_uint32),
        ("timeout_count", ctypes.c_uint32),
        ("crc_error_count", ctypes.c_uint32),
        ("last_rx_len", ctypes.c_uint16),
        ("last_rx_frame", ctypes.c_uint8 * 40),
        ("last_tx_len", ctypes.c_uint16),
        ("last_tx_frame", ctypes.c_uint8 * 8),
        ("uart_rx_state", ctypes.c_uint32),
        ("uart_error_code", ctypes.c_uint32),
        ("last_seen_tick", ctypes.c_uint32),
    ]


class UMotorNodeRegisterMap(ctypes.Structure):
    _fields_ = [
        ("reg_cmd_speed_rpm", ctypes.c_uint16),
        ("reg_cmd_start", ctypes.c_uint16),
        ("cmd_start_speed_mode_value", ctypes.c_uint16),
        ("verify_read_start_reg", ctypes.c_uint16),
        ("verify_read_reg_count", ctypes.c_uint16),
        ("status_read_start_reg", ctypes.c_uint16),
        ("status_read_reg_count", ctypes.c_uint16),
        ("fault_read_start_reg", ctypes.c_uint16),
        ("fault_read_reg_count", ctypes.c_uint16),
    ]


class UMotorNodeDebugInfo(ctypes.Structure):
    _fields_ = [
        ("poll_ok_count", ctypes.c_uint32),
        ("poll_fail_count", ctypes.c_uint32),
        ("poll_last_stage", ctypes.c_uint8),
        ("poll_last_req_start_reg", ctypes.c_uint16),
        ("poll_last_req_reg_count", ctypes.c_uint16),
        ("poll_last_rsp_reg_count", ctypes.c_uint16),
        ("poll_last_rsp_len", ctypes.c_uint16),
        ("reg_0014_node_id_cfg", ctypes.c_uint16),
        ("reg_0015_baud_cfg", ctypes.c_uint16),
        ("baud_rate_bps", ctypes.c_uint32),
        ("reg_0016_uart_fmt_cfg", ctypes.c_uint16),
        ("reg_0017_save_cmd", ctypes.c_uint16),
        ("reg_0000_driver_version", ctypes.c_uint16),
        ("reg_0001_driver_model", ctypes.c_uint16),
        ("reg_0002_node_id_runtime", ctypes.c_uint16),
        ("reg_0003_work_mode", ctypes.c_uint16),
        ("reg_0004_status", ctypes.c_uint16),
        ("reg_0005_target_speed", ctypes.c_uint16),
        ("reg_0006_actual_speed", ctypes.c_uint16),
        ("reg_0007_error_code", ctypes.c_uint16),
        ("reg_0008_error_subcode", ctypes.c_uint16),
        ("reg_000B_pos_low", ctypes.c_uint16),
        ("reg_000C_pos_high", ctypes.c_uint16),
        ("pos_signed_32", ctypes.c_int32),
        ("reg_0194_latest_fault", ctypes.c_uint16),
        ("reg_0195_latest_fault_sub", ctypes.c_uint16),
        ("reg_0196_second_fault", ctypes.c_uint16),
        ("reg_0197_second_fault_sub", ctypes.c_uint16),
        ("reg_0198_third_fault", ctypes.c_uint16),
        ("reg_0199_third_fault_sub", ctypes.c_uint16),
        ("reg_019A_comm_fault_bits", ctypes.c_uint16),
        ("poll_last_tx_frame", ctypes.c_uint8 * 8),
        ("poll_last_rx_frame", ctypes.c_uint8 * UMOTOR_NODE_DEBUG_RX_MAX),
    ]


class UMotorNodeState(ctypes.Structure):
    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("node_id", ctypes.c_uint8),
        ("enable_speed_ctrl", ctypes.c_uint8),
        ("enable_poll", ctypes.c_uint8),
        ("motor_enable", ctypes.c_uint8),
        ("valid", ctypes.c_uint32),
        ("comm_ok", ctypes.c_uint32),
        ("status_word", ctypes.c_uint16),
        ("feedback_speed_rpm", ctypes.c_int32),
        ("target_speed_rpm", ctypes.c_int32),
        ("trigger_pos_cmd", ctypes.c_uint8),
        ("pos_cmd_is_absolute", ctypes.c_uint8),
        ("pos_cmd_speed_rpm", ctypes.c_uint16),
        ("pos_cmd_pulses", ctypes.c_int32),
        ("trigger_stop_cmd", ctypes.c_uint8),
        ("stop_mode", ctypes.c_uint8),
        ("tx_count", ctypes.c_uint32),
        ("rx_count", ctypes.c_uint32),
        ("timeout_count", ctypes.c_uint32),
        ("crc_error_count", ctypes.c_uint32),
        ("protocol_error_count", ctypes.c_uint32),
        ("last_exception_func", ctypes.c_uint32),
        ("last_exception_code", ctypes.c_uint32),
        ("reg_map", UMotorNodeRegisterMap),
        ("verify_regs", ctypes.c_uint16 * UMOTOR_MAX_READ_REG_COUNT),
        ("status_regs", ctypes.c_uint16 * UMOTOR_MAX_READ_REG_COUNT),
        ("fault_regs", ctypes.c_uint16 * UMOTOR_MAX_READ_REG_COUNT),
        ("debug", UMotorNodeDebugInfo),
    ]


class UMotorBusDiag(ctypes.Structure):
    _fields_ = [
        ("initialized", ctypes.c_uint32),
        ("loop_count", ctypes.c_uint32),
        ("active_node_count", ctypes.c_uint32),
        ("total_tx_count", ctypes.c_uint32),
        ("total_rx_count", ctypes.c_uint32),
        ("total_timeout_count", ctypes.c_uint32),
        ("total_crc_error_count", ctypes.c_uint32),
        ("total_protocol_error_count", ctypes.c_uint32),
        ("last_tx_frame", ctypes.c_uint8 * 16),
        ("last_tx_len", ctypes.c_uint16),
        ("last_rx_frame", ctypes.c_uint8 * 32),
        ("last_rx_len", ctypes.c_uint16),
        ("last_error", ctypes.c_uint32),
        ("last_exception_func", ctypes.c_uint32),
        ("last_exception_code", ctypes.c_uint32),
    ]


class UMotor(ctypes.Structure):
    _fields_ = [
        ("nodes", UMotorNodeState * UMOTOR_NODE_COUNT),
        ("bus_diag", UMotorBusDiag),
        ("enable_write_cmd", ctypes.c_uint8),
    ]


class PressureSensorDiag(ctypes.Structure):
    _fields_ = [
        ("initialized", ctypes.c_uint32),
        ("slave_address", ctypes.c_uint32),
        ("last_error", ctypes.c_uint32),
        ("uart_error_code", ctypes.c_uint32),
        ("tx_in_flight", ctypes.c_uint32),
        ("response_waiting", ctypes.c_uint32),
        ("rx_irq_count", ctypes.c_uint32),
        ("rx_overflow_count", ctypes.c_uint32),
        ("request_count", ctypes.c_uint32),
        ("response_ok_count", ctypes.c_uint32),
        ("timeout_count", ctypes.c_uint32),
        ("crc_error_count", ctypes.c_uint32),
        ("exception_count", ctypes.c_uint32),
        ("last_exception_code", ctypes.c_uint32),
        ("last_request_tick", ctypes.c_uint32),
        ("last_response_tick", ctypes.c_uint32),
    ]


class PressureSensorReadout(ctypes.Structure):
    _fields_ = [
        ("last_irq_byte", ctypes.c_uint32),
        ("ring_available_bytes", ctypes.c_uint32),
        ("pressure_valid", ctypes.c_uint32),
        ("pressure_bar", ctypes.c_float),
        ("temperature_valid", ctypes.c_uint32),
        ("temperature_c", ctypes.c_float),
        ("last_update_tick", ctypes.c_uint32),
        ("last_tx_data", ctypes.c_uint8 * 16),
        ("last_response_len", ctypes.c_uint32),
        ("last_response_data", ctypes.c_uint8 * 16),
    ]


class PressureSensorState(ctypes.Structure):
    _fields_ = [
        ("diag", PressureSensorDiag),
        ("readout", PressureSensorReadout),
    ]


class IoControlPumpDebugInfo(ctypes.Structure):
    _fields_ = [
        ("right_outer_switch", ctypes.c_uint16),
        ("left_mid2_switch", ctypes.c_uint16),
        ("speed_triggered", ctypes.c_uint8),
        ("direction_reversed", ctypes.c_uint8),
    ]


class PumpMotorUart3Diag(ctypes.Structure):
    _fields_ = [
        ("initialized", ctypes.c_uint8),
        ("node_id", ctypes.c_uint8),
        ("valid", ctypes.c_uint8),
        ("tx_count", ctypes.c_uint32),
        ("rx_count", ctypes.c_uint32),
        ("timeout_count", ctypes.c_uint32),
        ("last_rx_frame", ctypes.c_uint8 * 64),
        ("last_tx_frame", ctypes.c_uint8 * 16),
        ("last_error", ctypes.c_uint8),
        ("status_code", ctypes.c_uint16),
        ("alarm_code", ctypes.c_uint16),
        ("feedback_speed_rpm", ctypes.c_int32),
    ]


class PumpDisplacementStatus(ctypes.Structure):
    _fields_ = [
        ("online", ctypes.c_uint8),
        ("valid", ctypes.c_uint8),
        ("sensor_id", ctypes.c_uint8),
        ("displacement_mm", ctypes.c_float),
        ("tx_count", ctypes.c_uint32),
        ("rx_count", ctypes.c_uint32),
        ("timeout_count", ctypes.c_uint32),
        ("crc_error_count", ctypes.c_uint32),
        ("last_error", ctypes.c_uint32),
        ("last_seen_tick", ctypes.c_uint32),
    ]


class PumpMotorStatus(ctypes.Structure):
    _fields_ = [
        ("online", ctypes.c_uint8),
        ("initialized", ctypes.c_uint8),
        ("node_id", ctypes.c_uint8),
        ("valid", ctypes.c_uint8),
        ("feedback_speed_rpm", ctypes.c_int32),
        ("tx_count", ctypes.c_uint32),
        ("rx_count", ctypes.c_uint32),
        ("status_code", ctypes.c_uint16),
        ("alarm_code", ctypes.c_uint16),
        ("timeout_count", ctypes.c_uint32),
        ("last_error", ctypes.c_uint32),
        ("last_seen_tick", ctypes.c_uint32),
    ]


class PumpDriverStatus(ctypes.Structure):
    _fields_ = [
        ("displacement", PumpDisplacementStatus),
        ("motor", PumpMotorStatus),
    ]


class ServoDebugView(ctypes.Structure):
    _fields_ = [
        ("pwm_started", ctypes.c_uint32),
        ("mode", ctypes.c_uint32),
        ("cmd_angle_deg", ctypes.c_float),
        ("current_angle_deg", ctypes.c_float),
        ("min_angle_deg", ctypes.c_float),
        ("max_angle_deg", ctypes.c_float),
        ("current_pulse_us", ctypes.c_uint16),
        ("min_pulse_us", ctypes.c_uint16),
        ("center_pulse_us", ctypes.c_uint16),
        ("max_pulse_us", ctypes.c_uint16),
        ("sweep_speed_deg_s", ctypes.c_float),
        ("sweep_step_us", ctypes.c_uint16),
        ("update_period_ms", ctypes.c_uint16),
        ("sweep_direction", ctypes.c_int32),
        ("move_count", ctypes.c_uint32),
        ("step_count", ctypes.c_uint32),
    ]


class IMUDebugView(ctypes.Structure):
    _fields_ = [
        ("frame_count", ctypes.c_uint32),
        ("crc_error_count", ctypes.c_uint32),
        ("rx_overflow_count", ctypes.c_uint32),
        ("rx_irq_count", ctypes.c_uint32),
        ("rx_byte_count", ctypes.c_uint32),
        ("rx_poll_byte_count", ctypes.c_uint32),
        ("rx_rearm_ok_count", ctypes.c_uint32),
        ("rx_rearm_fail_count", ctypes.c_uint32),
        ("rx_init_status", ctypes.c_uint32),
        ("uart4_irq_count", ctypes.c_uint32),
        ("uart4_error_count", ctypes.c_uint32),
        ("uart4_last_error", ctypes.c_uint32),
        ("uart_rxne_flag", ctypes.c_uint32),
        ("uart_error_flags", ctypes.c_uint32),
        ("recent_rx_len", ctypes.c_uint32),
        ("recent_rx_bytes", ctypes.c_uint8 * 16),
        ("task_loop_count", ctypes.c_uint32),
        ("last_function_code", ctypes.c_uint32),
        ("last_rx_function", ctypes.c_uint32),
        ("last_rx_state", ctypes.c_int32),
        ("ax", ctypes.c_float), ("ay", ctypes.c_float), ("az", ctypes.c_float),
        ("gx", ctypes.c_float), ("gy", ctypes.c_float), ("gz", ctypes.c_float),
        ("mx", ctypes.c_float), ("my", ctypes.c_float), ("mz", ctypes.c_float),
        ("raw_updated", ctypes.c_uint32),
        ("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float),
        ("euler_updated", ctypes.c_uint32),
        ("q0", ctypes.c_float), ("q1", ctypes.c_float), ("q2", ctypes.c_float), ("q3", ctypes.c_float),
        ("quat_updated", ctypes.c_uint32),
        ("height", ctypes.c_float), ("temperature", ctypes.c_float),
        ("pressure", ctypes.c_float), ("pressure_contrast", ctypes.c_float),
        ("baro_updated", ctypes.c_uint32),
        ("version_high", ctypes.c_uint8),
        ("version_mid", ctypes.c_uint8),
        ("version_low", ctypes.c_uint8),
    ]


class GliderDebugQuickView(ctypes.Structure):
    _fields_ = [
        ("update_count", ctypes.c_uint32),
        ("frame_type", ctypes.c_uint32),
        ("checksum_valid", ctypes.c_uint32),
        ("is_bdpwi", ctypes.c_uint32),
        ("is_bdepi", ctypes.c_uint32),
        ("is_gnrmc", ctypes.c_uint32),
        ("sentence", ctypes.c_char * 24),
        ("utc", ctypes.c_char * 16),
        ("mode_code", ctypes.c_uint32),
        ("status_code", ctypes.c_uint32),
        ("longitude_deg", ctypes.c_float),
        ("latitude_deg", ctypes.c_float),
        ("position_valid", ctypes.c_uint32),
        ("signal_good", ctypes.c_uint32),
        ("tx_mode", ctypes.c_uint32),
        ("card_id_known", ctypes.c_uint32),
        ("card_mismatch", ctypes.c_uint32),
        ("has_cached_fix", ctypes.c_uint32),
        ("shortmsg_pending", ctypes.c_uint32),
        ("ready_to_send", ctypes.c_uint32),
        ("local_card_id", ctypes.c_char * 24),
        ("target_card_id", ctypes.c_char * 24),
        ("last_tx_text", ctypes.c_char * 32),
        ("last_text", ctypes.c_char * 96),
    ]


class GliderDebugView(ctypes.Structure):
    _fields_ = [
        ("boot_stage", ctypes.c_uint32), ("alive_counter", ctypes.c_uint32), ("rtos_tick", ctypes.c_uint32),
        ("rx_line_count", ctypes.c_uint32), ("rx_overflow_count", ctypes.c_uint32),
        ("tx_attempt_count", ctypes.c_uint32), ("tx_ok_count", ctypes.c_uint32), ("tx_fail_count", ctypes.c_uint32),
        ("tx_last_status", ctypes.c_uint32), ("tx_last_uart_error", ctypes.c_uint32),
        ("last_tx_text", ctypes.c_char * 32), ("card_id", ctypes.c_char * 24),
        ("card_id_known", ctypes.c_uint32), ("shortmsg_sent_count", ctypes.c_uint32),
        ("last_shortmsg_payload", ctypes.c_char * 96), ("last_rx_update_count", ctypes.c_uint32),
        ("last_rx_text_len", ctypes.c_uint32), ("last_rx_text", ctypes.c_char * 512),
        ("last_rx_bin_len", ctypes.c_uint32), ("last_rx_bin", ctypes.c_uint8 * 64),
        ("is_rd_mode", ctypes.c_uint32), ("is_bdpwi", ctypes.c_uint32), ("bdpwi_utc", ctypes.c_char * 16),
        ("bdpwi_field1", ctypes.c_uint32), ("bdpwi_field2", ctypes.c_uint32),
        ("bdpwi_field1_valid", ctypes.c_uint32), ("bdpwi_field2_valid", ctypes.c_uint32),
        ("bdpwi_latitude_raw", ctypes.c_char * 16), ("bdpwi_longitude_raw", ctypes.c_char * 16),
        ("bdpwi_heading_raw", ctypes.c_char * 16), ("bdpwi_latitude_x100", ctypes.c_int32),
        ("bdpwi_longitude_x100", ctypes.c_int32), ("bdpwi_heading_x100", ctypes.c_int32),
        ("bdpwi_latitude_valid", ctypes.c_uint32), ("bdpwi_longitude_valid", ctypes.c_uint32),
        ("bdpwi_heading_valid", ctypes.c_uint32), ("bdpwi_latitude_dm", ctypes.c_char * 20),
        ("bdpwi_latitude_ns", ctypes.c_char * 4), ("bdpwi_longitude_dm", ctypes.c_char * 20),
        ("bdpwi_longitude_ew", ctypes.c_char * 4), ("bdpwi_latitude_deg_e7", ctypes.c_int32),
        ("bdpwi_longitude_deg_e7", ctypes.c_int32), ("bdpwi_latitude_deg", ctypes.c_float),
        ("bdpwi_longitude_deg", ctypes.c_float), ("bdpwi_latitude_deg_valid", ctypes.c_uint32),
        ("bdpwi_longitude_deg_valid", ctypes.c_uint32), ("bdpwi_old_beam_count", ctypes.c_uint32),
        ("bdpwi_new_beam_count", ctypes.c_uint32), ("bdpwi_best_d_cnr", ctypes.c_uint32),
        ("bdpwi_best_p_cnr", ctypes.c_uint32), ("bdpwi_good_beam_count", ctypes.c_uint32),
        ("rd_signal_good", ctypes.c_uint32), ("tx_mode", ctypes.c_uint32), ("is_gnrmc", ctypes.c_uint32),
        ("gnrmc_utc", ctypes.c_char * 16), ("gnrmc_status", ctypes.c_char * 4), ("gnrmc_date", ctypes.c_char * 16),
        ("gnrmc_latitude_dm", ctypes.c_char * 20), ("gnrmc_latitude_ns", ctypes.c_char * 4),
        ("gnrmc_longitude_dm", ctypes.c_char * 20), ("gnrmc_longitude_ew", ctypes.c_char * 4),
        ("gnrmc_latitude_deg_e7", ctypes.c_int32), ("gnrmc_longitude_deg_e7", ctypes.c_int32),
        ("gnrmc_latitude_deg", ctypes.c_float), ("gnrmc_longitude_deg", ctypes.c_float),
        ("gnrmc_status_valid", ctypes.c_uint32), ("gnrmc_latitude_valid", ctypes.c_uint32),
        ("gnrmc_longitude_valid", ctypes.c_uint32), ("is_bdepi", ctypes.c_uint32),
        ("bdepi_sender_card", ctypes.c_char * 24), ("bdepi_receiver_card", ctypes.c_char * 24),
        ("bdepi_utc", ctypes.c_char * 16), ("bdepi_freq_code", ctypes.c_uint32),
        ("bdepi_emergency_code", ctypes.c_uint32), ("bdepi_report_mode", ctypes.c_uint32),
        ("bdepi_freq_valid", ctypes.c_uint32), ("bdepi_emergency_valid", ctypes.c_uint32),
        ("bdepi_report_mode_valid", ctypes.c_uint32), ("bdepi_longitude_dm", ctypes.c_char * 20),
        ("bdepi_longitude_ew", ctypes.c_char * 4), ("bdepi_latitude_dm", ctypes.c_char * 20),
        ("bdepi_latitude_ns", ctypes.c_char * 4), ("bdepi_longitude_deg_e7", ctypes.c_int32),
        ("bdepi_latitude_deg_e7", ctypes.c_int32), ("bdepi_longitude_deg", ctypes.c_float),
        ("bdepi_latitude_deg", ctypes.c_float), ("bdepi_longitude_valid", ctypes.c_uint32),
        ("bdepi_latitude_valid", ctypes.c_uint32), ("bdepi_altitude_raw", ctypes.c_char * 16),
        ("bdepi_altitude_x100", ctypes.c_int32), ("bdepi_altitude_valid", ctypes.c_uint32),
        ("bdepi_altitude_unit", ctypes.c_char * 4), ("bdepi_altitude_abnormal", ctypes.c_char * 8),
        ("bdepi_altitude_abn_unit", ctypes.c_char * 4), ("bdepi_precision_indicator", ctypes.c_char * 8),
        ("bdepi_multi_solution", ctypes.c_char * 8), ("bdepi_rescue_subtype", ctypes.c_char * 8),
        ("bdepi_rescue_center_type", ctypes.c_char * 8), ("bdepi_data_len", ctypes.c_uint32),
        ("bdepi_data_len_valid", ctypes.c_uint32), ("bdepi_data_payload", ctypes.c_char * 48),
    ]


class EthernetUartSystemSnapshot(ctypes.Structure):
    _fields_ = [
        ("displacement_sensor", DisplacementSensorData),
        ("displacement_diag", DisplacementSensorDiag),
        ("dbg_view", GliderDebugView),
        ("dbg_quick", GliderDebugQuickView),
        ("imu_debug", IMUDebugView),
        ("limit_switch_states", ctypes.c_uint8 * 4),
        ("umotor", UMotor),
        ("pressure_sensor_state", PressureSensorState),
        ("io_pump_debug", IoControlPumpDebugInfo),
        ("pump_motor_uart3_diag", PumpMotorUart3Diag),
        ("pump_driver_status", PumpDriverStatus),
        ("servo_debug", ServoDebugView),
    ]


# 与 C 侧结构体布局保持一致时，该值应接近 3652
ETHERNET_SNAPSHOT_SIZE = ctypes.sizeof(EthernetUartSystemSnapshot)


def decode_snapshot_hex_to_struct(hex_text):
    """把十六进制文本还原成 EthernetUartSystemSnapshot 结构体。

    参数:
        hex_text: 仅包含十六进制字符的字符串，可夹杂空白。
    返回:
        EthernetUartSystemSnapshot 对象；长度不匹配时抛出 ValueError。
    """
    compact = ''.join(hex_text.split())
    if len(compact) % 2 != 0:
        raise ValueError("十六进制长度必须为偶数")

    raw = bytes.fromhex(compact)
    if len(raw) != ETHERNET_SNAPSHOT_SIZE:
        raise ValueError(f"快照长度不匹配: {len(raw)} != {ETHERNET_SNAPSHOT_SIZE}")

    snap = EthernetUartSystemSnapshot()
    ctypes.memmove(ctypes.addressof(snap), raw, ETHERNET_SNAPSHOT_SIZE)
    return snap


def is_hex_line(text):
    s = text.strip()
    if not s or len(s) % 2 != 0:
        return False
    hex_chars = set("0123456789abcdefABCDEF")
    return all(ch in hex_chars for ch in s)


class SnapshotAssembler:
    """按行拼接 MCU 发来的十六进制快照数据。"""

    def __init__(self, expected_size):
        self.expected_size = expected_size
        self._hex_parts = []
        self._hex_chars = 0

    def feed_line(self, line):
        """喂入一行文本；若拼满一个快照则返回结构体，否则返回 None。"""
        if not is_hex_line(line):
            return None

        s = line.strip()
        self._hex_parts.append(s)
        self._hex_chars += len(s)

        expected_hex_chars = self.expected_size * 2
        if self._hex_chars < expected_hex_chars:
            return None

        joined = ''.join(self._hex_parts)
        use_hex = joined[:expected_hex_chars]
        remain_hex = joined[expected_hex_chars:]

        snap = decode_snapshot_hex_to_struct(use_hex)

        self._hex_parts = [remain_hex] if remain_hex else []
        self._hex_chars = len(remain_hex)
        return snap

# ==================== 控制包发送函数 ====================
def send_control_packet(conn, pitchmotor_direction, pitchmotor_speed_rpm,
                        rollmotor_direction, rollmotor_speed_rpm,
                        thrustmotor_direction, thrustmotor_speed_rpm,
                        pump_direction, pump_on_off,
                        ask_for_status, clear_rx_ring):
    """打包并发送控制包"""
    pack_format = '<B I B I B I B B B B'   # 小端紧凑打包
    try:
        packet = struct.pack(pack_format,
                             pitchmotor_direction & 0xFF,
                             pitchmotor_speed_rpm & 0xFFFFFFFF,
                             rollmotor_direction & 0xFF,
                             rollmotor_speed_rpm & 0xFFFFFFFF,
                             thrustmotor_direction & 0xFF,
                             thrustmotor_speed_rpm & 0xFFFFFFFF,
                             pump_direction & 0xFF,
                             pump_on_off & 0xFF,
                             ask_for_status & 0xFF,
                             clear_rx_ring & 0xFF)
        conn.sendall(packet)
        print(f"[控制包] 已发送，长度 {len(packet)} 字节")
        return True
    except Exception as e:
        print(f"[发送错误] {e}")
        return False

# ==================== 控制包输入交互 ====================
def input_control_packet(conn):
    """引导用户输入控制包各字段并发送"""
    print("\n========== 输入控制包字段 ==========")
    fields = [
        ('pitchmotor_direction', 'uint8 (0-255)'),
        ('pitchmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('rollmotor_direction', 'uint8 (0-255)'),
        ('rollmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('thrustmotor_direction', 'uint8 (0-255)'),
        ('thrustmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('pump_direction', 'uint8 (0-255)'),
        ('pump_on_off', 'uint8 (0-255)'),
        ('ask_for_status', 'uint8 (0-255)'),
        ('clear_rx_ring', 'uint8 (0-255)')
    ]
    values = []
    for name, hint in fields:
        while True:
            try:
                val_str = input(f"{name} ({hint}): ").strip()
                val = int(val_str)
                if 'uint8' in hint and (val < 0 or val > 255):
                    print("  输入超出 0-255 范围，请重新输入。")
                    continue
                values.append(val)
                break
            except ValueError:
                print("  请输入有效的整数。")
    send_control_packet(conn, *values)

# ==================== 接收线程函数 ====================
def receive_thread(conn):
    """负责不断接收并打印来自客户端的数据"""
    global running
    assembler = SnapshotAssembler(ETHERNET_SNAPSHOT_SIZE)
    while running:
        try:
            # 设置超时，以便能定期检查 running 标志
            conn.settimeout(1.0)
            data = conn.recv(4096)
            if not data:
                print("[信息] 客户端已断开连接。")
                break
            # 尝试解码为文本，如果是二进制则显示十六进制
            try:
                text = data.decode('utf-8', errors='replace')
                lines = text.replace('\r', '\n').split('\n')
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue

                    snap = assembler.feed_line(line)
                    if snap is not None:
                        print(f"\n[客户端] 已还原 EthernetUartSystemSnapshot，大小={ctypes.sizeof(snap)} 字节")
                        continue

                    print(f"\n[客户端] {line}")
            except UnicodeDecodeError:
                print(f"\n[客户端] (二进制) {data.hex()}")
        except socket.timeout:
            # 超时后继续循环，以便检查 running 标志
            continue
        except ConnectionResetError:
            print("[信息] 客户端异常断开。")
            break
        except Exception as e:
            print(f"[接收错误] {e}")
            break
    # 线程结束前清理
    conn.close()
    print("[接收线程] 已退出。")

# ==================== 主程序 ====================
def main():
    global running

    # 创建服务端 socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('192.168.57.100', 8800))
    server_socket.listen(1)
    print("服务端已绑定 192.168.57.100:8800，等待客户端连接...")

    conn, addr = server_socket.accept()
    print(f"接收到客户端连接：{addr}")

    # 启动接收线程
    recv_thread = threading.Thread(target=receive_thread, args=(conn,), daemon=True)
    recv_thread.start()

    # ========== 主线程负责输入和发送 ==========
    print("\n========== 交互控制台 ==========")
    print("输入文本直接发送（文本格式）")
    print("输入 'pack' 发送控制包")
    print("输入 'quit' 或 'exit' 退出程序")
    print("==================================\n")

    try:
        while True:
            user_input = input(">>> ").strip()
            if not user_input:
                continue

            if user_input.lower() in ('quit', 'exit'):
                print("退出程序。")
                break
            elif user_input.lower() == 'pack':
                input_control_packet(conn)
            else:
                # 以文本方式发送
                try:
                    conn.send(user_input.encode('utf-8'))
                    print("[文本] 已发送")
                except Exception as e:
                    print(f"[发送错误] {e}")
                    break
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，退出。")
    finally:
        # 通知接收线程退出
        running = False
        # 等待线程结束（最多2秒）
        recv_thread.join(timeout=2)
        # 关闭连接
        conn.close()
        server_socket.close()
        print("服务端已关闭。")

if __name__ == "__main__":
    main()