import socket
import threading
import time
import sys
import ctypes
import json
import os
from datetime import datetime

try:
    from .ugcp_protocol import (
        MSG_CONTROL19_LATCHED,
        MSG_CONTROL19_STREAM,
        MSG_SNAPSHOT_REQUEST,
        MSG_STREAM_CONTROL,
        STREAM_EKF,
        SequenceCounter,
        build_frame,
        pack_control_payload,
        pack_snapshot_request_payload,
        pack_stream_control_payload,
        pack_telem_control_payload,
    )
except ImportError:
    from ugcp_protocol import (
        MSG_CONTROL19_LATCHED,
        MSG_CONTROL19_STREAM,
        MSG_SNAPSHOT_REQUEST,
        MSG_STREAM_CONTROL,
        STREAM_EKF,
        SequenceCounter,
        build_frame,
        pack_control_payload,
        pack_snapshot_request_payload,
        pack_stream_control_payload,
        pack_telem_control_payload,
    )

# ==================== 全局标志 ====================
running = True          # 控制线程退出
UGCP_TX_SEQUENCE = SequenceCounter()
UGCP_TX_LOCK = threading.Lock()

AUTO_SEND_PERIOD_S = 1.0
AUTO_KEY_POLL_S = 0.02
THRUST_SPEED_MIN = 0.0
THRUST_SPEED_MAX = 30.0
THRUST_SPEED_INIT = 10.0
THRUST_SPEED_REARM_AFTER_RESET = 10.0
THRUST_DIFF_RATE_PER_S = 20.0

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
        ("alive_counter", ctypes.c_uint32),
        ("gnrmc_latitude_e6", ctypes.c_int32),
        ("gnrmc_longitude_e6", ctypes.c_int32),
        ("alarm_bits", ctypes.c_uint32),
        ("displacement_mm_x10", ctypes.c_int16),
        ("imu_roll_x10", ctypes.c_int16),
        ("imu_pitch_x10", ctypes.c_int16),
        ("imu_yaw_x10", ctypes.c_int16),
        ("imu_height_dm", ctypes.c_int16),
        ("imu_temp_c_x10", ctypes.c_int16),
        ("pressure_bar_x100", ctypes.c_uint16),
        ("pressure_temp_c_x10", ctypes.c_int16),
        ("pump_feedback_rpm", ctypes.c_int16),
        ("umotor0_feedback_rpm", ctypes.c_int16),
        ("umotor1_feedback_rpm", ctypes.c_int16),
        ("servo_pulse_us", ctypes.c_uint16),
        ("limit_switch_bits", ctypes.c_uint8),
        ("displacement_valid", ctypes.c_uint8),
        ("gnrmc_valid", ctypes.c_uint8),
        ("pressure_valid", ctypes.c_uint8),
        ("servo_mode", ctypes.c_uint8),
        ("pump_speed_triggered", ctypes.c_uint8),
        ("pump_direction_reversed", ctypes.c_uint8),
        # EKF 状态估计（与板端 EthernetUartSystemSnapshot 严格同序）
        ("ekf_pN_cm", ctypes.c_int32),     # x：北向位置 (cm)
        ("ekf_pE_cm", ctypes.c_int32),     # y：东向位置 (cm)
        ("ekf_depth_mm", ctypes.c_int32),  # 深度 (mm，正=水下)
        ("ekf_roll_x10", ctypes.c_int16),  # 横滚 (deg×10)
        ("ekf_pitch_x10", ctypes.c_int16), # 俯仰 (deg×10)
        ("ekf_yaw_x10", ctypes.c_int16),   # 航向 (deg×10)
        ("ekf_status", ctypes.c_uint8),    # UgEkfStatus 0..5
        ("ekf_valid", ctypes.c_uint8),     # 1 = valid_for_control
    ]


# 与 C 侧 EthernetUartSystemSnapshot 精简结构保持一致
ETHERNET_SNAPSHOT_SIZE = ctypes.sizeof(EthernetUartSystemSnapshot)


ALARM_BIT_DEFINITIONS = [
    (0, "displacement_last_error", "位移传感器 last_error != 0"),
    (1, "displacement_timeout", "位移传感器 timeout_count != 0"),
    (2, "displacement_crc_error", "位移传感器 crc_error_count != 0"),
    (3, "imu_crc_error", "IMU crc_error_count != 0"),
    (4, "imu_rx_overflow", "IMU rx_overflow_count != 0"),
    (5, "imu_uart_error", "IMU uart4_error_count != 0"),
    (6, "pressure_last_error", "压力传感器 last_error != 0"),
    (7, "pressure_timeout", "压力传感器 timeout_count != 0"),
    (8, "pressure_crc_error", "压力传感器 crc_error_count != 0"),
    (9, "pressure_exception", "压力传感器 exception_count != 0"),
    (10, "pump_uart_alarm", "泵 UART3 last_error/alarm_code 异常"),
    (11, "pump_driver_alarm", "泵驱动 motor alarm/last_error 异常"),
    (12, "umotor_bus_error", "UMotor 总线 last_error != 0"),
    (13, "umotor_node_alarm", "UMotor 节点0/1 有 error_code 或 latest_fault"),
    (14, "gnrmc_invalid", "GNRMC 状态或经纬度无效"),
    (15, "limit_switch_triggered", "限位开关任意一路触发"),
]


def decode_alarm_bits(alarm_bits):
    """把 alarm_bits 解码为可读报警列表。"""
    alarms = []
    for bit, code, desc in ALARM_BIT_DEFINITIONS:
        if alarm_bits & (1 << bit):
            alarms.append({
                "bit": bit,
                "code": code,
                "desc": desc,
            })
    return alarms


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


def _char_array_to_str(arr):
    raw = bytes(arr)
    nul_idx = raw.find(b'\x00')
    if nul_idx >= 0:
        raw = raw[:nul_idx]
    return raw.decode('utf-8', errors='replace')


def _ctypes_to_py(value, max_depth=6, array_limit=16, _depth=0):
    if _depth >= max_depth:
        return '...'

    if isinstance(value, ctypes.Structure):
        out = {}
        for field_name, _ in value._fields_:
            out[field_name] = _ctypes_to_py(
                getattr(value, field_name),
                max_depth=max_depth,
                array_limit=array_limit,
                _depth=_depth + 1,
            )
        return out

    if isinstance(value, ctypes.Array):
        if getattr(value, '_type_', None) is ctypes.c_char:
            return _char_array_to_str(value)

        length = len(value)
        show_len = min(length, array_limit)
        out = [_ctypes_to_py(value[i], max_depth=max_depth, array_limit=array_limit, _depth=_depth + 1)
               for i in range(show_len)]
        if length > show_len:
            out.append(f'...(+{length - show_len} items)')
        return out

    if isinstance(value, (bytes, bytearray)):
        return value.hex().upper()

    return value


def print_snapshot_struct(snap):
    """把 EthernetUartSystemSnapshot 以可读结构化形式输出到命令行。"""
    snap_dict = _ctypes_to_py(snap)
    alarms = decode_alarm_bits(snap.alarm_bits)
    if alarms:
        print("[报警] 触发报警如下:")
        for alarm in alarms:
            print(f"  - bit{alarm['bit']:02d} {alarm['code']}: {alarm['desc']}")
    else:
        print("[报警] 无报警")

    print(json.dumps(snap_dict, ensure_ascii=False, indent=2))

# 控制包发送函数
def send_ugcp_frame(conn, msg_type, payload, label='UGCP'):
    """串行化一次完整 UGCP 发送，保证多线程下 seq 顺序与线上顺序一致。"""
    try:
        with UGCP_TX_LOCK:
            seq = UGCP_TX_SEQUENCE.next()
            frame = build_frame(msg_type, payload, seq)
            conn.sendall(frame)
        print(f'[{label}] 已发送 seq={seq}，payload={len(payload)}B，总长={len(frame)}B')
        return True
    except Exception as e:
        print(f'[发送错误] {e}')
        return False


def send_control_packet(conn, pitchmotor_enable, pitchmotor_speed_rpm,
                        rollmotor_enable, rollmotor_speed_rpm,
                        thrustmotor_direction, thrustmotor_direction2,
                        thrustmotor1se_speed_rpm, thrustmotor2se_speed_rpm,
                        backup_for_thrust,
                        pump_direction, pump_on_off,
                        ask_for_status, eth_control_flag, latched=False):
    """编码同一份 CONTROL19；latched=False 是连续租约，True 是键盘锁存。"""
    try:
        payload = pack_control_payload(
            pitchmotor_enable, pitchmotor_speed_rpm,
            rollmotor_enable, rollmotor_speed_rpm,
            thrustmotor_direction, thrustmotor_direction2,
            thrustmotor1se_speed_rpm, thrustmotor2se_speed_rpm,
            backup_for_thrust, pump_direction, pump_on_off,
            ask_for_status, eth_control_flag,
        )
    except Exception as e:
        print(f'[控制字段错误] {e}')
        return False
    msg_type = MSG_CONTROL19_LATCHED if latched else MSG_CONTROL19_STREAM
    label = '锁存控制包' if latched else '连续控制包'
    return send_ugcp_frame(conn, msg_type, payload, label)


def send_stream_control(conn, mask):
    """设置完整遥测状态：bit0=$UGEKF，bit1=$UGMAG。"""
    return send_ugcp_frame(
        conn,
        MSG_STREAM_CONTROL,
        pack_stream_control_payload(mask),
        '遥测流控制',
    )


def send_telem_control(conn, enabled):
    """兼容交互命令：off=全关，on=只开 EKF。"""
    return send_ugcp_frame(
        conn,
        MSG_STREAM_CONTROL,
        pack_telem_control_payload(enabled),
        'EKF 遥测控制',
    )


def send_neutral_and_release(conn):
    """正常退出：锁存全中立，再释放 Ethernet 控制权。"""
    neutral_ok = send_control_packet(
        conn,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0, 1,
        latched=True,
    )
    if neutral_ok:
        time.sleep(0.05)
    release_ok = send_control_packet(
        conn,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0, 0,
        latched=True,
    )
    return neutral_ok and release_ok


def send_snapshot_request(conn):
    return send_ugcp_frame(
        conn, MSG_SNAPSHOT_REQUEST, pack_snapshot_request_payload(), '快照请求')

# ==================== 控制包输入交互 ====================
def input_control_packet(conn):
    """用户输入控制包各字段并发送"""
    print("\n========== 输入控制包字段 ==========")
    fields = [
        ('pitchmotor_enable', 'bit (0/1)'),
        ('pitchmotor_speed_rpm', 'int32'),
        ('rollmotor_enable', 'bit (0/1)'),
        ('rollmotor_speed_rpm', 'int32'),
        ('thrustmotor_direction', 'bit (0/1)'),
        ('thrustmotor_direction2', 'bit (0/1)'),
        ('thrustmotor1se_speed_rpm', 'speed (0-100)'),
        ('thrustmotor2se_speed_rpm', 'speed (0-100)'),
        ('backup_for_thrust', 'bit (0/1)'),
        ('pump_direction', 'bit (0/1)'),
        ('pump_on_off', 'bit (0/1)'),
        ('ask_for_status', 'bit (0/1)'),
        ('eth_control_flag', 'bit (0/1)')
    ]
    values = []
    for name, hint in fields:
        while True:
            try:
                val_str = input(f"{name} ({hint}): ").strip()
                val = int(val_str)
                if hint.startswith('bit') and val not in (0, 1):
                    print('  该字段只能输入 0 或 1。')
                    continue
                if hint.startswith('speed') and not 0 <= val <= 100:
                    print('  推进器速度必须在 0..100。')
                    continue
                if hint == 'int32' and not -0x80000000 <= val <= 0x7FFFFFFF:
                    print('  转速必须在 int32 范围。')
                    continue
                values.append(val)
                break
            except ValueError:
                print("  请输入有效的整数。")
    sent = send_control_packet(conn, *values, latched=True)
    if sent and values[-1] == 1:
        print("[锁存警告] 此命令没有 2.5 秒超时，会一直保持到下一条控制命令、"
              "显式释放或板子复位；断网不会自动停机。")


def _clamp(value, low, high):
    return max(low, min(high, value))


def _build_default_auto_packet():
    return {
        "pitchmotor_enable": 0,
        "pitchmotor_speed_rpm": 0,
        "rollmotor_enable": 0,
        "rollmotor_speed_rpm": 0,
        "thrustmotor_direction": 0,
        "thrustmotor_direction2": 0,
        "thrustmotor1se_speed_rpm": 0,
        "thrustmotor2se_speed_rpm": 0,
        "backup_for_thrust": 0,
        "pump_direction": 0,
        "pump_on_off": 0,
        "ask_for_status": 0,
        "eth_control_flag": 1,
    }


def _apply_thrust_direction(packet, direction_key, now_s, dir_state, init_speed=None):
    if init_speed is None:
        init_speed = THRUST_SPEED_INIT

    packet["thrustmotor1se_speed_rpm"] = int(init_speed)
    packet["thrustmotor2se_speed_rpm"] = int(init_speed)

    if direction_key == 'I':
        packet["thrustmotor_direction"] = 0
        packet["thrustmotor_direction2"] = 1
    else:
        packet["thrustmotor_direction"] = 1
        packet["thrustmotor_direction2"] = 0

    dir_state["mode"] = direction_key
    dir_state["hold_start"] = now_s
    dir_state["base_s1"] = float(packet["thrustmotor1se_speed_rpm"])
    dir_state["base_s2"] = float(packet["thrustmotor2se_speed_rpm"])


def _apply_thrust_direction_keep_speed(packet, direction_key, now_s, dir_state):
    if direction_key == 'I':
        packet["thrustmotor_direction"] = 0
        packet["thrustmotor_direction2"] = 1
    else:
        packet["thrustmotor_direction"] = 1
        packet["thrustmotor_direction2"] = 0

    dir_state["mode"] = direction_key
    dir_state["hold_start"] = now_s
    dir_state["base_s1"] = float(packet["thrustmotor1se_speed_rpm"])
    dir_state["base_s2"] = float(packet["thrustmotor2se_speed_rpm"])


def _print_auto_help(packet):
    print("[AUTO] 按键说明:")
    print("  W: pitch_enable=1, pitch_speed=-200")
    print("  S: pitch_enable=1, pitch_speed=200")
    print("  X: pitch_enable=0, pitch_speed=0")
    print("  E: pump_direction=0, pump_on_off=1")
    print("  D: pump_direction=1, pump_on_off=1")
    print("  C: pump_direction=0, pump_on_off=0")
    print("  I: thrust_dir=(0,1)，首次/切换置双推进速度为10，长按同比例放大")
    print("  K: thrust_dir=(1,0)，首次/切换置双推进速度为10，长按同比例放大")
    print("  J: thrust1下降 thrust2上升（1秒变化20）")
    print("  L: thrust1上升 thrust2下降（1秒变化20）")
    print("  U: 双推进按比例衰减，长按1秒变为按下瞬间的0.5")
    print("  N: thrustmotor1se_speed_rpm 和 thrustmotor2se_speed_rpm 同时置0")
    print("  Z: 除 eth_control_flag 外全部清零，eth_control_flag=1")
    print("  N/Z 后首次 I/K: 双推进速度置10")
    print("  H: 显示帮助与当前包状态")
    print("  Q/ESC: 退出 auto")
    print(
        "[AUTO] 当前包状态: "
        f"pitch=({packet['pitchmotor_enable']},{packet['pitchmotor_speed_rpm']}) "
        f"roll=({packet['rollmotor_enable']},{packet['rollmotor_speed_rpm']}) "
        f"thrust_dir=({packet['thrustmotor_direction']},{packet['thrustmotor_direction2']}) "
        f"thrust_spd=({packet['thrustmotor1se_speed_rpm']},{packet['thrustmotor2se_speed_rpm']}) "
        f"pump=({packet['pump_direction']},{packet['pump_on_off']}) "
        f"backup={packet['backup_for_thrust']} ask={packet['ask_for_status']} eth_flag={packet['eth_control_flag']}"
    )


def run_auto_mode(conn):
    """自动模式：1 秒发送一次控制包，按键仅修改对应字段。"""
    if os.name != 'nt':
        print("[AUTO] 仅支持 Windows 控制台键盘读取。")
        return

    user32 = ctypes.windll.user32
    vk_map = {
        'W': ord('W'),
        'S': ord('S'),
        'X': ord('X'),
        'E': ord('E'),
        'D': ord('D'),
        'C': ord('C'),
        'I': ord('I'),
        'K': ord('K'),
        'J': ord('J'),
        'L': ord('L'),
        'U': ord('U'),
        'N': ord('N'),
        'Z': ord('Z'),
        'H': ord('H'),
        'Q': ord('Q'),
        'ESC': 0x1B,
    }

    packet = _build_default_auto_packet()
    prev_pressed = {k: False for k in vk_map}
    dir_state = {
        "mode": None,
        "hold_start": time.monotonic(),
        "base_s1": float(packet["thrustmotor1se_speed_rpm"]),
        "base_s2": float(packet["thrustmotor2se_speed_rpm"]),
        "rearm_init_pending": False,
        "u_hold_start": time.monotonic(),
        "u_base_s1": float(packet["thrustmotor1se_speed_rpm"]),
        "u_base_s2": float(packet["thrustmotor2se_speed_rpm"]),
    }

    last_loop_s = time.monotonic()
    next_send_s = last_loop_s

    print("[AUTO] 已进入自动模式：每 1 秒发送一次控制包。")
    print("[AUTO] 按键: W/S/X 俯仰, E/D/C 泵, I/K 推进方向, J/L 双推进差动, U 双推进衰减, N 双推进清零, Z 清零, H 帮助, Q 或 ESC 退出。")
    _print_auto_help(packet)

    while running:
        now_s = time.monotonic()
        dt = now_s - last_loop_s
        last_loop_s = now_s

        pressed = {
            k: (user32.GetAsyncKeyState(vk) & 0x8000) != 0
            for k, vk in vk_map.items()
        }
        rising = {k: pressed[k] and (not prev_pressed[k]) for k in vk_map}
        prev_pressed = pressed

        if rising['Q'] or rising['ESC']:
            packet = _build_default_auto_packet()
            dir_state["mode"] = None
            dir_state["hold_start"] = now_s
            dir_state["base_s1"] = 0.0
            dir_state["base_s2"] = 0.0
            dir_state["rearm_init_pending"] = True
            send_neutral_and_release(conn)
            print("[AUTO] 已发送 Z 清零包 + 全零停控包，退出自动模式。")
            break

        if rising['H']:
            _print_auto_help(packet)

        if rising['U']:
            dir_state["u_hold_start"] = now_s
            dir_state["u_base_s1"] = float(packet["thrustmotor1se_speed_rpm"])
            dir_state["u_base_s2"] = float(packet["thrustmotor2se_speed_rpm"])

        if rising['N']:
            packet["thrustmotor1se_speed_rpm"] = 0
            packet["thrustmotor2se_speed_rpm"] = 0
            dir_state["mode"] = None
            dir_state["base_s1"] = 0.0
            dir_state["base_s2"] = 0.0
            dir_state["hold_start"] = now_s
            dir_state["rearm_init_pending"] = True
            print("[AUTO] 已执行 N: 双推进速度置 0，推进方向状态已重置。")

        if rising['Z']:
            packet = _build_default_auto_packet()
            dir_state["mode"] = None
            dir_state["hold_start"] = now_s
            dir_state["base_s1"] = 0.0
            dir_state["base_s2"] = 0.0
            dir_state["rearm_init_pending"] = True
            print("[AUTO] 已执行清零: 除 eth_control_flag 外全部字段置 0。")
            _print_auto_help(packet)

        if rising['W']:
            packet["pitchmotor_enable"] = 1
            packet["pitchmotor_speed_rpm"] = -200
        if rising['S']:
            packet["pitchmotor_enable"] = 1
            packet["pitchmotor_speed_rpm"] = 200
        if rising['X']:
            packet["pitchmotor_enable"] = 0
            packet["pitchmotor_speed_rpm"] = 0

        if rising['E']:
            packet["pump_direction"] = 0
            packet["pump_on_off"] = 1
        if rising['D']:
            packet["pump_direction"] = 1
            packet["pump_on_off"] = 1
        if rising['C']:
            packet["pump_direction"] = 0
            packet["pump_on_off"] = 0

        if rising['I']:
            if dir_state["mode"] != 'I':
                init_speed = THRUST_SPEED_REARM_AFTER_RESET if dir_state["rearm_init_pending"] else THRUST_SPEED_INIT
                _apply_thrust_direction(packet, 'I', now_s, dir_state, init_speed=init_speed)
                dir_state["rearm_init_pending"] = False
            else:
                _apply_thrust_direction_keep_speed(packet, 'I', now_s, dir_state)
        if rising['K']:
            if dir_state["mode"] != 'K':
                init_speed = THRUST_SPEED_REARM_AFTER_RESET if dir_state["rearm_init_pending"] else THRUST_SPEED_INIT
                _apply_thrust_direction(packet, 'K', now_s, dir_state, init_speed=init_speed)
                dir_state["rearm_init_pending"] = False
            else:
                _apply_thrust_direction_keep_speed(packet, 'K', now_s, dir_state)

        s1 = float(packet["thrustmotor1se_speed_rpm"])
        s2 = float(packet["thrustmotor2se_speed_rpm"])

        if pressed['J'] and (not pressed['L']):
            if dir_state["mode"] == 'K':
                s1 += THRUST_DIFF_RATE_PER_S * dt
                s2 -= THRUST_DIFF_RATE_PER_S * dt
            else:
                s1 -= THRUST_DIFF_RATE_PER_S * dt
                s2 += THRUST_DIFF_RATE_PER_S * dt
        elif pressed['L'] and (not pressed['J']):
            if dir_state["mode"] == 'K':
                s1 -= THRUST_DIFF_RATE_PER_S * dt
                s2 += THRUST_DIFF_RATE_PER_S * dt
            else:
                s1 += THRUST_DIFF_RATE_PER_S * dt
                s2 -= THRUST_DIFF_RATE_PER_S * dt

        if pressed['U'] and (not pressed['I']) and (not pressed['K']) and (not pressed['J']) and (not pressed['L']):
            u_hold_s = max(0.0, now_s - dir_state["u_hold_start"])
            u_scale = 0.5 ** u_hold_s
            s1 = dir_state["u_base_s1"] * u_scale
            s2 = dir_state["u_base_s2"] * u_scale

        if pressed['I'] and (not pressed['K']) and dir_state["mode"] == 'I' and (not pressed['J']) and (not pressed['L']):
            hold_s = max(0.0, now_s - dir_state["hold_start"])
            scale = 2.0 ** hold_s
            s1 = dir_state["base_s1"] * scale
            s2 = dir_state["base_s2"] * scale
        elif pressed['K'] and (not pressed['I']) and dir_state["mode"] == 'K' and (not pressed['J']) and (not pressed['L']):
            hold_s = max(0.0, now_s - dir_state["hold_start"])
            scale = 2.0 ** hold_s
            s1 = dir_state["base_s1"] * scale
            s2 = dir_state["base_s2"] * scale

        s1 = _clamp(s1, THRUST_SPEED_MIN, THRUST_SPEED_MAX)
        s2 = _clamp(s2, THRUST_SPEED_MIN, THRUST_SPEED_MAX)
        packet["thrustmotor1se_speed_rpm"] = int(round(s1))
        packet["thrustmotor2se_speed_rpm"] = int(round(s2))
        packet["eth_control_flag"] = 1

        if now_s >= next_send_s:
            send_control_packet(
                conn,
                packet["pitchmotor_enable"],
                packet["pitchmotor_speed_rpm"],
                packet["rollmotor_enable"],
                packet["rollmotor_speed_rpm"],
                packet["thrustmotor_direction"],
                packet["thrustmotor_direction2"],
                packet["thrustmotor1se_speed_rpm"],
                packet["thrustmotor2se_speed_rpm"],
                packet["backup_for_thrust"],
                packet["pump_direction"],
                packet["pump_on_off"],
                packet["ask_for_status"],
                packet["eth_control_flag"],
                latched=False,
            )
            print(
                "[AUTO] send "
                f"pitch_en={packet['pitchmotor_enable']} pitch_spd={packet['pitchmotor_speed_rpm']} "
                f"pump=({packet['pump_direction']},{packet['pump_on_off']}) "
                f"thr_dir=({packet['thrustmotor_direction']},{packet['thrustmotor_direction2']}) "
                f"thr_spd=({packet['thrustmotor1se_speed_rpm']},{packet['thrustmotor2se_speed_rpm']})"
            )
            next_send_s += AUTO_SEND_PERIOD_S
            while next_send_s <= now_s:
                next_send_s += AUTO_SEND_PERIOD_S

        time.sleep(AUTO_KEY_POLL_S)

#  EKF 遥测($UGEKF)解析 + CSV 记录 
# 字段顺序必须与板端 EthernetUartDriver_SendEkfTelemetry() 完全一致。
EKF_BASE_FIELDS = [
    "seq", "t_ms", "status", "fl",
    "roll_deg", "pitch_deg", "yaw_deg",
    "pN", "pE", "depth",
    "vN", "vE", "vD",
    "bgx", "bgy", "bgz",
    "bax", "bay", "baz",
    "Ppos", "Pvel", "Patt",
    "nisD", "accD", "rejD", "dstreak", "dreset",
    "dmeas", "dt", "imuhz", "tlast",
]
EKF_ACK_FIELDS = ["cal_crc", "cal_valid"]
EKF_FIELDS = EKF_BASE_FIELDS + EKF_ACK_FIELDS
EKF_INT_FIELDS = {
    "seq", "t_ms", "status", "fl", "accD", "rejD", "dstreak", "dreset",
    "cal_crc", "cal_valid",
}
EKF_STATUS_NAMES = {0: "UNINIT", 1: "WAIT_BOOT_DELAY", 2: "WAIT_STILL",
                    3: "ALIGNING", 4: "RUNNING", 5: "FAULT"}
# CSV 表头：墙钟时间 + fl 展开成 5 个布尔列 + 其余数值字段
EKF_CSV_HEADER = (["host_iso"]
                  + ["seq", "t_ms", "status", "status_name",
                     "healthy", "valid", "still", "finite", "aligned"]
                  + EKF_FIELDS[4:])


def ekf_checksum_ok(line):
    """校验 $...*HH：异或 $ 与 * 之间所有字符。"""
    star = line.rfind("*")
    if star < 1 or star + 3 > len(line):
        return False
    cks = 0
    for ch in line[1:star]:
        cks ^= ord(ch)
    try:
        return cks == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def ekf_parse(line):
    """解析旧 31 字段或带磁标定 ACK 的新 33 字段 $UGEKF。"""
    star = line.rfind("*")
    parts = line[1:star].split(",")
    if not parts or parts[0] != "UGEKF":
        return None
    values = parts[1:]
    if len(values) == len(EKF_BASE_FIELDS):
        frame_fields = EKF_BASE_FIELDS
    elif len(values) == len(EKF_FIELDS):
        frame_fields = EKF_FIELDS
    else:
        return None
    row = {}
    try:
        for name, raw in zip(frame_fields, values):
            row[name] = int(raw) if name in EKF_INT_FIELDS else float(raw)
    except ValueError:
        return None
    for name in EKF_ACK_FIELDS:
        row.setdefault(name, None)
    return row


def ekf_csv_row(row):
    """把解析结果 + fl 位展开成与 EKF_CSV_HEADER 对齐的列表。"""
    fl = row["fl"]
    out = [datetime.now().isoformat(timespec="milliseconds"),
           row["seq"], row["t_ms"], row["status"],
           EKF_STATUS_NAMES.get(row["status"], "?"),
           (fl >> 0) & 1, (fl >> 1) & 1, (fl >> 2) & 1, (fl >> 3) & 1, (fl >> 4) & 1]
    out.extend(row[name] for name in EKF_FIELDS[4:])
    return out


# ==================== 接收线程函数 ====================
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
EKF_LOG_DIR = os.path.join(REPO_ROOT, "logs", "csv")


def receive_thread(conn):
    """负责不断接收并打印来自客户端的数据"""
    global running
    assembler = SnapshotAssembler(ETHERNET_SNAPSHOT_SIZE)
    line_buffer = ""

    # —— EKF 遥测 CSV：连接起把 $UGEKF 落盘，便于离线绘图 ——
    os.makedirs(EKF_LOG_DIR, exist_ok=True)
    ekf_csv_path = os.path.join(EKF_LOG_DIR, "ekf_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    ekf_csv = open(ekf_csv_path, "w", encoding="utf-8", newline="")
    ekf_csv.write(",".join(EKF_CSV_HEADER) + "\n")
    ekf_csv.flush()
    print(f"[EKF] 状态遥测记录（非磁标定） -> {ekf_csv_path}")
    ekf_stat = {"ok": 0, "bad": 0, "last_print": 0.0, "last_key": None}

    def handle_line(raw_line):
        line = raw_line.strip()
        if not line:
            return

        # —— EKF 遥测行：始终落 CSV；仅在 状态/健康/就绪 变化时打印，平时静默 ——
        #     （另留 30s 心跳确认链路存活；想完全静默把 HB 设成很大，如 1e9）
        if line.startswith("$UGEKF"):
            if not ekf_checksum_ok(line):
                ekf_stat["bad"] += 1
                return
            row = ekf_parse(line)
            if row is None:
                ekf_stat["bad"] += 1
                return
            ekf_csv.write(",".join(str(x) for x in ekf_csv_row(row)) + "\n")
            ekf_csv.flush()
            ekf_stat["ok"] += 1
            fl = row["fl"]
            key = (row["status"], (fl >> 0) & 1, (fl >> 1) & 1)  # (status, healthy, valid)
            now = time.time()
            HB = 10.0  # 心跳间隔(秒)
            if key != ekf_stat["last_key"] or (now - ekf_stat["last_print"] >= HB):
                ekf_stat["last_key"] = key
                ekf_stat["last_print"] = now
                print("\n[EKF] %s hlth=%d vld=%d roll=%.1f pitch=%.1f yaw=%.1f "
                      "depth=%.3f vD=%.3f ok=%d bad=%d"
                      % (EKF_STATUS_NAMES.get(row["status"], "?"),
                         (fl >> 0) & 1, (fl >> 1) & 1,
                         row["roll_deg"], row["pitch_deg"], row["yaw_deg"],
                         row["depth"], row["vD"], ekf_stat["ok"], ekf_stat["bad"]))
            return

        # 磁标定采集属于 station/tools/mag_logger.py。新固件默认不会发；
        # 这里仍防御性吞掉旧固件或上一次会话残留的数据，避免终端刷屏。
        if line.startswith("$UGMAG"):
            return

        if line == "AT":
            print("\n[客户端] AT")
            return

        snap = assembler.feed_line(line)
        if snap is not None:
            print(f"\n[客户端] 已还原 EthernetUartSystemSnapshot，大小={ctypes.sizeof(snap)} 字节")
            print_snapshot_struct(snap)
            return

        print(f"\n[客户端] {line}")

    while running:
        try:
            # 设置超时，以便能定期检查 running 标志
            conn.settimeout(1.0)
            data = conn.recv(4096)
            if not data:
                print("[信息] 客户端已断开连接。")
                break

            # STM32 发的是 ASCII 文本（AT 或快照十六进制行），这里做流式按行拼接。
            text = data.decode('ascii', errors='ignore')
            if not text:
                print(f"\n[客户端] (二进制) {data.hex()}")
                continue

            line_buffer += text
            while True:
                newline_idx = line_buffer.find('\n')
                if newline_idx < 0:
                    break
                one_line = line_buffer[:newline_idx].rstrip('\r')
                line_buffer = line_buffer[newline_idx + 1:]
                handle_line(one_line)
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
    try:
        ekf_csv.close()
    except Exception:
        pass
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
    if send_stream_control(conn, STREAM_EKF):
        print("[遥测] 已声明 EKF-only；磁标定采集请单独运行 station/tools/mag_logger.py")
    else:
        print("[遥测警告] EKF-only 声明发送失败，请检查连接后重试 ekfon")

    # 启动接收线程
    recv_thread = threading.Thread(target=receive_thread, args=(conn,), daemon=True)
    recv_thread.start()

    # ========== 主线程负责输入和发送 ==========
    print("\n========== 交互控制台 ==========")
    print("仅接受下列命令；未知文本不会发送到板端")
    print("输入 'pack' 发送锁存控制包（保持到下一条命令，没有看门狗保护）")
    print("输入 'auto' 进入连续控制模式（受2.5秒看门狗保护，按 Q/ESC 退出）")
    print("输入 'ekfon' / 'ekfoff' 开 / 停板子 EKF 遥测")
    print("输入 'snapshot' 请求一次系统状态快照")
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
            elif user_input.lower() == 'auto':
                run_auto_mode(conn)
            elif user_input.lower() == 'ekfoff':
                send_telem_control(conn, 0)
                print("[EKF] 已请求板子停发遥测")
            elif user_input.lower() == 'ekfon':
                send_telem_control(conn, 1)
                print("[EKF] 已请求板子开始遥测")
            elif user_input.lower() == 'snapshot':
                send_snapshot_request(conn)
            else:
                print("[命令] 未识别，未发送；可用 pack/auto/ekfon/ekfoff/snapshot/quit")
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，退出。")
    finally:
        print("[退出] 正在锁存中立命令并释放 Ethernet 控制权...")
        send_neutral_and_release(conn)
        send_stream_control(conn, 0)
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
