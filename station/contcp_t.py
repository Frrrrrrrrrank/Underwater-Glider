from encodings import hz
import socket
import threading
import time
import sys
import ctypes
import json
import os
import shlex
import math
from datetime import datetime

try:
    from .battery_telemetry import (
        BatteryAlarmMonitor, UGBAT_CSV_HEADER, ugbat_csv_row, ugbat_parse,
    )
except ImportError:
    from battery_telemetry import (
        BatteryAlarmMonitor, UGBAT_CSV_HEADER, ugbat_csv_row, ugbat_parse,
    )

try:
    from .gamepad_control import (
        ControlTarget, GamepadConfig, GamepadService, map_gamepad_to_target,
    )
except ImportError:
    from gamepad_control import (
        ControlTarget, GamepadConfig, GamepadService, map_gamepad_to_target,
    )

try:
    from .ugcp_protocol import (
        CASCADE_MODE_DEPTH,
        CASCADE_MODE_OFF,
        CASCADE_MODE_PITCH_HOLD,
        MSG_CONTROL19_LATCHED,
        MSG_CONTROL19_STREAM,
        MSG_CONTROL21_STREAM,
        MSG_CASCADE_COMMAND,
        MSG_PARAM_BLOCK,
        MSG_PARAM_COMMIT,
        MSG_SNAPSHOT_REQUEST,
        MSG_STREAM_CONTROL,
        PARAM_GROUP_PID,
        PID_PARAM_BLOCK_COUNT,
        PID_PARAM_FIELDS,
        STREAM_EKF,
        STREAM_CTL,
        STREAM_WRM,
        STREAM_BAT,
        SequenceCounter,
        build_frame,
        crc32_iso_hdlc,
        pack_cascade_command_payload,
        pack_control_payload,
        pack_control21_payload,
        pack_param_block_payload,
        pack_param_commit_payload,
        pack_pid_param_table,
        pack_snapshot_request_payload,
        pack_stream_control_payload,
        pack_worm_calib_payload,
        MSG_WORM_CALIB,
        split_param_table,
    )
except ImportError:
    from ugcp_protocol import (
        CASCADE_MODE_DEPTH,
        CASCADE_MODE_OFF,
        CASCADE_MODE_PITCH_HOLD,
        MSG_CONTROL19_LATCHED,
        MSG_CONTROL19_STREAM,
        MSG_CONTROL21_STREAM,
        MSG_CASCADE_COMMAND,
        MSG_PARAM_BLOCK,
        MSG_PARAM_COMMIT,
        MSG_SNAPSHOT_REQUEST,
        MSG_STREAM_CONTROL,
        PARAM_GROUP_PID,
        PID_PARAM_BLOCK_COUNT,
        PID_PARAM_FIELDS,
        STREAM_EKF,
        STREAM_CTL,
        STREAM_WRM,
        STREAM_BAT,
        SequenceCounter,
        build_frame,
        crc32_iso_hdlc,
        pack_cascade_command_payload,
        pack_control_payload,
        pack_control21_payload,
        pack_param_block_payload,
        pack_param_commit_payload,
        pack_pid_param_table,
        pack_snapshot_request_payload,
        pack_stream_control_payload,
        pack_worm_calib_payload,
        MSG_WORM_CALIB,
        split_param_table,
    )

# 全局标志
running = True          # 控制线程退出
UGCP_TX_SEQUENCE = SequenceCounter()
UGCP_TX_LOCK = threading.Lock()
TELEMETRY_STREAM_LOCK = threading.Lock()
TELEMETRY_STREAM_MASK = 0
# 主线程标记当前是否处于 PID 命令界面，接收线程据此选择快照显示方式。
PID_MODE_ACTIVE = threading.Event()

AUTO_SEND_PERIOD_S = 0.10  # 每 100ms 连续刷新一次，板端直控租约为 2.5s。
AUTO_KEY_POLL_S = 0.02
AUTO_RUDDER_LIMIT_DEG = 10.0  # 首次联调仅开放中位两侧各 10 度。
AUTO_RUDDER_RATE_DEG_S = 10.0
AUTO_RUDDER_MAX_DT_S = 0.10  # PC 卡顿后不补算长时间位移，单步最多 1 度。
AUTO_VK_MAP = {key: ord(key) for key in 'WSXEDCIKJLUNZHQRFVT'}
AUTO_VK_MAP['ESC'] = 0x1B
THRUST_SPEED_MIN = 0.0
THRUST_SPEED_MAX = 100.0
THRUST_SPEED_INIT = 10.0
THRUST_SPEED_REARM_AFTER_RESET = 10.0
THRUST_DIFF_RATE_PER_S = 20.0

PID_COMMAND_PERIOD_S = 0.10     #10hz PID控制心跳包
PID_ACK_TIMEOUT_S = 3.0

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
        # 蜗杆行程位置（板端同样追加在结构体末尾，顺序必须完全一致）
        ("worm_stroke_percent_x100", ctypes.c_int16),  # 0=前限位, 10000=后限位
        ("worm_span_valid", ctypes.c_uint8),           # 1=已跑过两端，百分比才有意义
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


def _print_snapshot_alarms(snap):
    alarms = decode_alarm_bits(snap.alarm_bits)
    if alarms:
        print("[报警] 触发报警如下:")
        for alarm in alarms:
            print(f"  - bit{alarm['bit']:02d} {alarm['code']}: {alarm['desc']}")
    else:
        print("[报警] 无报警")


def format_worm_status(snap):
    """蜗杆行程位置摘要。span_valid=0 时百分比无意义，显式说明而不是印一个假数。"""
    if not int(snap.worm_span_valid):
        return ("[蜗杆] stroke_percent 不可用 span_valid=0"
                "（本次上电尚未跑过两端限位，无法确定行程端点）")
    return ("[蜗杆] stroke_percent=%.2f%% span_valid=1"
            "（0=机头前限位 PB4，100=机尾后限位 PD7）"
            % (float(snap.worm_stroke_percent_x100) / 100.0))


def format_rudder_status(snap):
    """快照是板端已施加的 PWM 命令，三线舵机没有角度反馈。"""
    mode = int(snap.servo_mode)
    mode_text = {0: 'PWM 初始化失败/未启动', 1: '中位保护', 2: '网线命令'}.get(
        mode, '未知模式')
    return ("[尾舵] applied_pulse_us=%d servo_mode=%d(%s)；命令值，非实测角度"
            % (int(snap.servo_pulse_us), mode, mode_text))


def print_snapshot_struct(snap):
    """AUTO 和主菜单查询：完整结构 JSON，前面先给一行蜗杆摘要免得在 JSON 里翻。"""
    snap_dict = _ctypes_to_py(snap)
    _print_snapshot_alarms(snap)
    print(format_worm_status(snap))
    print(format_rudder_status(snap))
    print(json.dumps(snap_dict, ensure_ascii=False, indent=2))


def print_pid_snapshot_struct(snap):
    """PID status：只显示控制调试需要的输入状态。
    """
    limit_bits = int(snap.limit_switch_bits) & 0x0F
    print(
        "[输入状态] displacement=%.1f mm valid=%d; "
        "limit_switch_bits=0x%02X "
        "[PB3=%d PB4=%d PB5=%d PD7=%d，0=触发]"
        % (
            float(snap.displacement_mm_x10) / 10.0,
            int(snap.displacement_valid),
            limit_bits,
            (limit_bits >> 0) & 1,
            (limit_bits >> 1) & 1,
            (limit_bits >> 2) & 1,
            (limit_bits >> 3) & 1,
        )
    )
    print(format_worm_status(snap))


def print_snapshot_for_active_mode(snap):
    if PID_MODE_ACTIVE.is_set():
        print_pid_snapshot_struct(snap)
    else:
        print_snapshot_struct(snap)

# 控制包发送函数
def send_ugcp_frame(conn, msg_type, payload, label='UGCP', verbose=True):
    """串行化一次完整 UGCP 发送，保证多线程下 seq 顺序与线上顺序一致。"""
    try:
        with UGCP_TX_LOCK:
            seq = UGCP_TX_SEQUENCE.next()
            frame = build_frame(msg_type, payload, seq)
            conn.sendall(frame)
        if verbose:
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
    """编码同一份 CONTROL19；latched=False 是连续心跳包，True 是键盘锁存。"""
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


def send_auto_control_packet(conn, packet, label='AUTO'):
    """键盘和手柄共用 21 字节连续命令；退出包保持 CONTROL19 兼容。"""
    try:
        payload = pack_control21_payload(**packet)
    except Exception as e:
        print(f'[{label} 控制字段错误] {e}')
        return False
    return send_ugcp_frame(
        conn, MSG_CONTROL21_STREAM, payload, f'{label} 连续控制包', verbose=False)


def send_stream_control(conn, mask, label='遥测流控制', *, keep_battery=True):
    """设置完整遥测状态；控制会话始终保留电池流，退出时才显式全关。"""
    global TELEMETRY_STREAM_MASK
    if keep_battery:
        mask = int(mask) | STREAM_BAT
    ok = send_ugcp_frame(
        conn,
        MSG_STREAM_CONTROL,
        pack_stream_control_payload(mask),
        label,
    )
    if ok:
        with TELEMETRY_STREAM_LOCK:
            TELEMETRY_STREAM_MASK = int(mask)
    return ok


def send_worm_calib(conn, start):
    """蜗杆回零：start=True 开始，False 取消。

    开始前顺带打开 $UGWRM 流，否则回零过程完全看不见（板子会自己转电机）。
    板端只置标志、由 ServoTask 推进；回零自带分段超时，这里不需要心跳。
    """
    if start:
        send_stream_control(conn, get_stream_control_mask() | STREAM_WRM)
    return send_ugcp_frame(
        conn, MSG_WORM_CALIB, pack_worm_calib_payload(start), "蜗杆回零")


def get_stream_control_mask():
    with TELEMETRY_STREAM_LOCK:
        return TELEMETRY_STREAM_MASK


def send_telem_control(conn, enabled):
    """兼容交互命令仅切换 EKF，保留电池以及其他主题订阅。"""
    mask = get_stream_control_mask()
    mask = mask | STREAM_EKF if enabled else mask & ~STREAM_EKF
    return send_stream_control(conn, mask, 'EKF 遥测控制')


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


def send_cascade_command(conn, mode, armed, command_id,
                         target_depth_m, target_pitch_deg,
                         signed_thrust_percent, required_pid_crc,
                         verbose=False):
    """发送 PID/双环命令；持续刷新时默认静默，错误始终显示。"""
    try:
        payload = pack_cascade_command_payload(
            mode,
            armed,
            command_id,
            target_depth_m,
            target_pitch_deg,
            signed_thrust_percent,
            required_pid_crc,
        )
    except Exception as exc:
        print(f'[PID 命令字段错误] {exc}')
        return False
    return send_ugcp_frame(
        conn,
        MSG_CASCADE_COMMAND,
        payload,
        'PID 控制命令',
        verbose=verbose,
    )


def send_pid_param_table(conn, table, transaction):
    """发送 PID 参数的 5 个数据块和 1 个提交块，返回表 CRC 或 None。"""
    table = bytes(table)
    blocks = split_param_table(table)
    table_crc = crc32_iso_hdlc(table)
    transaction = int(transaction) & 0xFF
    for index, block in enumerate(blocks):
        payload = pack_param_block_payload(
            PARAM_GROUP_PID, transaction, index, block)
        if not send_ugcp_frame(
                conn, MSG_PARAM_BLOCK, payload,
                f'PID 参数块 {index + 1}/{PID_PARAM_BLOCK_COUNT}'):
            return None
    commit = pack_param_commit_payload(
        PARAM_GROUP_PID,
        transaction,
        PID_PARAM_BLOCK_COUNT,
        table_crc,
    )
    if not send_ugcp_frame(conn, MSG_PARAM_COMMIT, commit, 'PID 参数提交'):
        return None
    return table_crc

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
        "rudder_angle_cdeg": 0,
    }


def _build_auto_sti_packet(target, config):
    """把推进百分比和 pitch/roll 转速目标填入现有直控协议。"""
    packet = _build_default_auto_packet()
    speeds = (target.left_percent, target.right_percent)
    if not config.motor1_is_left:
        speeds = speeds[::-1]
    for speed, direction, speed_key, direction_key in zip(
            speeds, (config.motor1_forward_dir, config.motor2_forward_dir),
            ('thrustmotor1se_speed_rpm', 'thrustmotor2se_speed_rpm'),
            ('thrustmotor_direction', 'thrustmotor_direction2')):
        if not math.isfinite(speed) or abs(speed) > config.max_percent:
            raise ValueError('推进目标必须为限幅内的有限百分比')
        magnitude = min(int(round(abs(speed))), int(config.max_percent))
        packet[speed_key] = magnitude
        packet[direction_key] = (direction if speed >= 0 else 1 - direction) if magnitude else 0
    if (target.pitch_enable not in (0, 1) or
            target.pitch_rpm not in (-config.pitch_rpm, 0, config.pitch_rpm) or
            bool(target.pitch_enable) != bool(target.pitch_rpm)):
        raise ValueError('pitch 目标使能与转速不一致')
    packet['pitchmotor_enable'] = target.pitch_enable
    packet['pitchmotor_speed_rpm'] = target.pitch_rpm
    if (target.roll_enable not in (0, 1) or
            target.roll_rpm not in (-config.roll_rpm, 0, config.roll_rpm) or
            bool(target.roll_enable) != bool(target.roll_rpm)):
        raise ValueError('roll 目标使能与转速不一致')
    packet['rollmotor_enable'] = target.roll_enable
    packet['rollmotor_speed_rpm'] = target.roll_rpm
    return packet


def _auto_sti_escape_pressed():
    """保留控制台 ESC 退出入口；纯计算和离线测试可以在其他系统运行。"""
    return os.name == 'nt' and bool(ctypes.windll.user32.GetAsyncKeyState(0x1B) & 0x8000)


def _check_auto_sti_ready(ctl_status, wrm_status, after_ctl, after_wrm,
                          now, timeout_s):
    """返回观察条件是否满足及拒绝原因；状态观察不是逐命令执行 ACK。"""
    for status, generation in ((ctl_status, after_ctl), (wrm_status, after_wrm)):
        current, row, received_at = status
        if current <= generation or row is None or received_at is None:
            return False, ''
        age = now - received_at
        if not math.isfinite(age) or age < 0 or age > timeout_s:
            return False, '板端遥测已过期'
    ctl, wrm = ctl_status[1], wrm_status[1]
    if wrm.get('home_state') in (1, 2):
        return False, '蜗杆正在回零；请先执行 calibstop，观察结束后重新进入'
    if wrm.get('home_state') not in (0, 3, 4):
        return False, '蜗杆回零状态未知'
    return ctl.get('mode') == 0 and ctl.get('armed') == 0, ''


def run_auto_sti_mode(conn, gamepad_service, ctl_store, wrm_store, link_closed,
                      config=None, controller_index=None):
    """手柄接管：等待回中、观察板端状态、连续直控，所有已接管出口归中释放。"""
    config = config or GamepadConfig()
    if PID_MODE_ACTIVE.is_set():
        print('[AUTO_STI] 请先退出 PID 模式。')
        return
    if gamepad_service is None or not gamepad_service.is_alive():
        print('[AUTO_STI] 手柄采集服务不可用。')
        return
    indices = gamepad_service.connected_indices()
    if controller_index is None:
        if len(indices) != 1:
            print(f'[AUTO_STI] 已连接槽位 {indices}；请连接一只手柄或输入 auto_sti <槽位>。')
            return
        controller_index = indices[0]
    if controller_index not in (0, 1, 2, 3) or controller_index not in indices:
        print('[AUTO_STI] 指定手柄未连接。')
        return
    initial = gamepad_service.snapshot(controller_index)
    exit_seq, loss_seq = initial.exit_seq, initial.loss_seq
    previous_stream_mask = get_stream_control_mask()
    streams_changed = False
    control_attempted = False
    state = 'WAIT_CENTER'
    after_ctl = after_wrm = 0
    deadline = next_send = next_report = 0.0
    last_loop = time.monotonic()
    last_packet = None
    reason = '控制循环结束'
    print(f'[AUTO_STI] 绑定槽位 {controller_index}，上限 {config.max_percent:g}%；'
          '请回中并松开 Y/A 和十字键左/右，X 或 ESC 退出。')
    print('[AUTO_STI] 左右拨杆控制艇首转向，倒退时同向；纯横拨为零。'
          'Y/A 按住向前/后；'
          f'十字键左/右控制 roll +{config.roll_rpm}/-{config.roll_rpm}RPM，'
          '松开或左右同按停；退出会交还控制权。')
    try:
        while running and not link_closed.is_set():
            sample = gamepad_service.snapshot(controller_index)
            # 先复制快照再取时刻，避免后台刚发布的数据比旧 now 晚而被误判。
            now = time.monotonic()
            if _auto_sti_escape_pressed() or sample.button_x or sample.exit_seq != exit_seq:
                reason = 'X/ESC 退出'
                break
            if (not gamepad_service.is_alive() or not sample.connected or
                    sample.index != controller_index or sample.loss_seq != loss_seq):
                reason = '手柄断开、采集失败或连接曾中断；重新进入才能接管'
                break
            age = now - sample.sample_time
            if not math.isfinite(age) or age < 0 or age > config.input_timeout_s:
                reason = '手柄输入样本过期'
                break
            if state != 'WAIT_CENTER' and now - last_loop > config.input_timeout_s:
                reason = '控制循环发生长时间停顿'
                break
            last_loop = now
            # 每拍重新计算绝对目标，按键松开不会继承上一拍的运动命令。
            target = map_gamepad_to_target(sample, config)
            centered = (abs(sample.left_x) <= config.deadzone and
                        abs(sample.forward) <= config.deadzone and
                        not sample.button_y and not sample.button_a and
                        not sample.dpad_left and not sample.dpad_right)
            if state == 'WAIT_CENTER':
                if not centered:
                    time.sleep(config.poll_period_s)
                    continue
                after_ctl = ctl_store.snapshot()[0]
                after_wrm = wrm_store.snapshot()[0]
                streams_changed = True
                if not send_stream_control(
                        conn, previous_stream_mask | STREAM_CTL | STREAM_WRM,
                        'AUTO_STI 状态订阅'):
                    reason = '无法订阅板端状态'
                    break
                state = 'PREPARING'
                deadline = now + config.prepare_timeout_s
                next_send = now
                print('[AUTO_STI] 准备接管：发送中立，等待新 PID OFF 和回零非运行状态。')
            if state in ('PREPARING', 'ACTIVE'):
                ctl_status = ctl_store.snapshot_with_time()
                wrm_status = wrm_store.snapshot_with_time()
                now = time.monotonic()
                ready, rejection = _check_auto_sti_ready(
                    ctl_status, wrm_status,
                    after_ctl, after_wrm, now, config.telemetry_timeout_s)
                if rejection:
                    reason = rejection
                    break
                if state == 'ACTIVE' and not ready:
                    reason = '板端 PID 状态不再满足直控条件'
                    break
                if state == 'PREPARING':
                    if ready and centered and control_attempted:
                        state = 'ACTIVE'
                        next_send = now
                        print('[AUTO_STI] 已观察到进入条件满足，开始发送手柄目标（非执行 ACK）。')
                    elif now >= deadline:
                        reason = '准备超时；未观察到所需状态或手柄未回中'
                        break
            packet = _build_auto_sti_packet(
                target if state == 'ACTIVE' else ControlTarget(), config)
            # 回中和松键的停止不等待下一次常规心跳；普通运动最多按10Hz发送。
            force_stop = last_packet is not None and any(
                last_packet[key] != 0 and packet[key] == 0 for key in (
                    'thrustmotor1se_speed_rpm', 'thrustmotor2se_speed_rpm',
                    'pitchmotor_enable', 'rollmotor_enable'))
            if force_stop or now >= next_send:
                if (link_closed.is_set() or
                        time.monotonic() - sample.sample_time > config.input_timeout_s):
                    reason = '发送前连接失效或本轮输入已过期'
                    break
                # sendall 报错也可能已经发送部分字节，因此发送前就标记需要清理。
                control_attempted = True
                if not send_auto_control_packet(conn, packet, label='AUTO_STI'):
                    reason = '发送失败'
                    break
                last_packet = packet
                next_send = now + config.send_period_s
                if now >= next_report:
                    print(f'[AUTO_STI {state}] 输入前后={sample.forward:+.2f} '
                          f'左右={sample.left_x:+.2f}；目标左={target.left_percent:+.0f}% '
                          f'右={target.right_percent:+.0f}% pitch={packet["pitchmotor_speed_rpm"]:+d}RPM '
                          f'roll={packet["rollmotor_speed_rpm"]:+d}RPM'
                          + ('（准备阶段实际发送中立）' if state != 'ACTIVE' else ''))
                    next_report = now + 1.0
            time.sleep(config.poll_period_s)
        if link_closed.is_set():
            reason = '控制板连接已断开'
    except KeyboardInterrupt:
        reason = 'Ctrl+C 退出手柄模式'
    except (OSError, ValueError, RuntimeError) as exc:
        reason = f'输入或控制异常：{exc}'
    finally:
        print(f'[AUTO_STI] {reason}')
        if control_attempted:
            if send_neutral_and_release(conn):
                print('[AUTO_STI] 已发送中立和释放，后续可能由 SBUS 接管；非全局急停。')
            else:
                print('[AUTO_STI] 中立/释放未完整发出，依赖板端现有约2.5秒 STREAM 租约。')
        if streams_changed:
            send_stream_control(conn, previous_stream_mask, '恢复遥测流')


def _update_auto_rudder(packet, state, pressed, rising, dt):
    """只在 PC 积分目标位置；板端接收绝对角度，不锁存无限旋转命令。"""
    old_direction = state['direction']
    force_send = any(rising.get(key, False) for key in ('R', 'F', 'V', 'T', 'Z'))
    # 停止/回中按住期间优先；同时按 R/F 停止，松开后需重新按方向键。
    if pressed.get('Z') or pressed.get('T'):
        state['angle_deg'] = 0.0
        state['direction'] = 0
    elif pressed.get('V') or (pressed.get('R') and pressed.get('F')):
        state['direction'] = 0
    elif rising.get('R'):
        state['direction'] = 1
    elif rising.get('F'):
        state['direction'] = -1

    safe_dt = _clamp(dt, 0.0, AUTO_RUDDER_MAX_DT_S)
    state['angle_deg'] = _clamp(
        state['angle_deg'] + state['direction'] * AUTO_RUDDER_RATE_DEG_S * safe_dt,
        -AUTO_RUDDER_LIMIT_DEG, AUTO_RUDDER_LIMIT_DEG)
    if (state['angle_deg'] >= AUTO_RUDDER_LIMIT_DEG and state['direction'] > 0) or (
            state['angle_deg'] <= -AUTO_RUDDER_LIMIT_DEG and state['direction'] < 0):
        state['direction'] = 0
    packet['rudder_angle_cdeg'] = int(round(state['angle_deg'] * 100.0))
    return force_send or old_direction != state['direction']


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
    print("  W: pitch_enable=1, pitch_speed=-50")
    print("  S: pitch_enable=1, pitch_speed=50")
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
    print("  R/F: 尾舵顺/逆向，按一次持续改变目标；首测核对实际转向")
    print(f"       速度 {AUTO_RUDDER_RATE_DEG_S:g}°/s，限位 ±{AUTO_RUDDER_LIMIT_DEG:g}°；到端点自动停")
    print("  V: 尾舵停止改变目标并保持；T: 尾舵回中；R+F: 停止")
    print("  Z: 全部清零并使尾舵回中，eth_control_flag=1")
    print("  N/Z 后首次 I/K: 双推进速度置10")
    print("  H: 显示帮助与当前包状态")
    print("  Q/ESC: 全中立、尾舵回中并退出 auto")
    print(
        "[AUTO] 当前包状态: "
        f"pitch=({packet['pitchmotor_enable']},{packet['pitchmotor_speed_rpm']}) "
        f"roll=({packet['rollmotor_enable']},{packet['rollmotor_speed_rpm']}) "
        f"thrust_dir=({packet['thrustmotor_direction']},{packet['thrustmotor_direction2']}) "
        f"thrust_spd=({packet['thrustmotor1se_speed_rpm']},{packet['thrustmotor2se_speed_rpm']}) "
        f"pump=({packet['pump_direction']},{packet['pump_on_off']}) "
        f"rudder_target={packet['rudder_angle_cdeg'] / 100.0:+.2f}°(目标，非实测) "
        f"backup={packet['backup_for_thrust']} ask={packet['ask_for_status']} eth_flag={packet['eth_control_flag']}"
    )


def run_auto_mode(conn):
    """网线 AUTO 手动控制：10Hz 刷新绝对目标，所有退出路径都尝试回中并释放。"""
    if os.name != 'nt':
        print("[AUTO] 仅支持 Windows 控制台键盘读取。")
        return

    user32 = ctypes.windll.user32
    vk_map = AUTO_VK_MAP

    packet = _build_default_auto_packet()
    rudder_state = {'angle_deg': 0.0, 'direction': 0}
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
    next_report_s = last_loop_s

    print("[AUTO] 已进入网线手动模式：10Hz 发送控制包，尾舵初始回中。")
    print("[AUTO] 按键: W/S/X 俯仰, E/D/C 泵, I/K/J/L/U/N 推进, R/F 尾舵顺逆转, V 尾舵停, T 尾舵回中, Z 全清零, H 帮助/快照, Q/ESC 退出。")
    _print_auto_help(packet)

    try:
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
                break

            if rising['H']:
                _print_auto_help(packet)
                packet["ask_for_status"] = 1

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
                packet["pitchmotor_speed_rpm"] = -50
            if rising['S']:
                packet["pitchmotor_enable"] = 1
                packet["pitchmotor_speed_rpm"] = 50
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
            # Z 持续按下时覆盖同轮其他按键，避免清零后又被方向键重新启动。
            if pressed['Z']:
                packet = _build_default_auto_packet()
                dir_state['mode'] = None
                dir_state['base_s1'] = 0.0
                dir_state['base_s2'] = 0.0
                dir_state['hold_start'] = now_s
                dir_state['rearm_init_pending'] = True
            force_send = _update_auto_rudder(packet, rudder_state, pressed, rising, dt)

            if force_send or now_s >= next_send_s:
                if not send_auto_control_packet(conn, packet):
                    print("[AUTO] 发送失败，停止本地目标更新并退出；板端由通信超时保护回中。")
                    break
                if force_send or now_s >= next_report_s:
                    print(
                        "[AUTO] send "
                        f"pitch_en={packet['pitchmotor_enable']} pitch_spd={packet['pitchmotor_speed_rpm']} "
                        f"pump=({packet['pump_direction']},{packet['pump_on_off']}) "
                        f"thr_dir=({packet['thrustmotor_direction']},{packet['thrustmotor_direction2']}) "
                        f"thr_spd=({packet['thrustmotor1se_speed_rpm']},{packet['thrustmotor2se_speed_rpm']}) "
                        f"rudder_target={packet['rudder_angle_cdeg'] / 100.0:+.2f}° "
                        f"rudder_motion={rudder_state['direction']}"
                    )
                    next_report_s = now_s + 1.0
                while next_send_s <= now_s:
                    next_send_s += AUTO_SEND_PERIOD_S
                packet["ask_for_status"] = 0

            time.sleep(AUTO_KEY_POLL_S)
    finally:
        rudder_state['direction'] = 0
        rudder_state['angle_deg'] = 0.0
        if send_neutral_and_release(conn):
            print("[AUTO] 已发送全中立及释放命令，尾舵回中；命令发送成功不等于机械动作已验证。")
        else:
            print("[AUTO] 回中/释放命令未完整发出；等待板端通信超时保护。")


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

# PID 控制遥测；顺序必须与板端 EthernetUartDriver_SendControlTelemetry 一致。
UGCTL_FIELDS = [
    "schema", "seq", "t_ms", "mode", "phase", "armed", "output_valid",
    "fault", "core_fault", "config_reject", "command_id", "age_ms",
    "pid_valid", "required_crc", "active_crc", "depth_seq",
    "target_depth", "depth", "depth_error", "target_pitch_deg",
    "pitch_sp_deg", "pitch_deg", "pitch_error_deg",
    "outer_p_mm", "outer_i_mm", "outer_d_mm", "outer_out_mm",
    "outer_sat", "target_displacement_mm", "displacement_mm",
    "displacement_error_mm", "inner_p_rpm", "inner_i_rpm", "inner_d_rpm",
    "inner_out_rpm", "inner_sat", "worm_rpm", "pump_enable",
    "pump_direction", "thrust_pct", "config_req", "config_apply",
    "config_reject_count",
]
UGCTL_INT_FIELDS = {
    "schema", "seq", "t_ms", "mode", "phase", "armed", "output_valid",
    "fault", "core_fault", "config_reject", "command_id", "age_ms",
    "pid_valid", "required_crc", "active_crc", "depth_seq", "outer_sat",
    "inner_sat", "worm_rpm", "pump_enable", "pump_direction", "config_req",
    "config_apply", "config_reject_count",
}
UGCTL_CSV_HEADER = ["host_iso"] + UGCTL_FIELDS

#  蜗杆位置遥测($UGWRM)解析 + CSV 记录
# 字段顺序必须与板端 EthernetUartDriver_SendWormTelemetry() 完全一致。
UGWRM_FIELDS = [
    "schema", "seq", "t_ms",
    # 位置量：stroke_percent 是主输出（0=前限位，100=后限位），pulse 是原始真值
    "pulse", "stroke_percent", "mm_from_front",
    "span_valid", "data_fresh", "ref_kind", "ref_lost",
    "pulse_min", "pulse_max", "span_pulses", "moving", "alarm",
    # 回零进度（schema 2 新增）
    "home_state", "home_fail", "pulse_home",
    # 电机状态：安装转换后的驱动请求 vs 原始反馈，用于看"发了没执行"
    "cmd_rpm", "fb_rpm", "status_word", "err", "err_sub",
    "input_ports", "limit_bits",
    # 总线健康：与位置同一时刻，用于判断上面那些数可不可信
    "poll_ok", "poll_fail", "overrun", "enable_resync", "last_error",
]
# 除两个百分比/毫米外全部按整数解析。
UGWRM_INT_FIELDS = set(UGWRM_FIELDS) - {"stroke_percent", "mm_from_front"}
UGWRM_CSV_HEADER = ["host_iso"] + UGWRM_FIELDS
WORM_REF_KIND_NAMES = {0: "NONE", 1: "POWER_ON", 2: "HOMED"}
WORM_HOME_STATE_NAMES = {
    0: "IDLE 未回零", 1: "SEEK 快速趋近前限位", 2: "BACKOFF 慢速退出记零点",
    3: "DONE 回零完成", 4: "FAILED 回零失败",
}
WORM_HOME_FAIL_NAMES = {
    0: "-", 1: "SEEK_TIMEOUT 一直没撞到前限位", 2: "BACKOFF_TIMEOUT 退不出前限位",
    3: "DATA_STALE 回零途中位置数据失效", 4: "CANCELLED 被取消或被 PID 接管",
    5: "POSITION_RANGE 回零端点超出脉冲范围",
}


def ugwrm_parse(line):
    """严格解析 schema 2 的 $UGWRM；校验或字段数不符一律返回 None。"""
    if not ekf_checksum_ok(line):
        return None
    star = line.rfind("*")
    parts = line[1:star].split(",")
    if not parts or parts[0] != "UGWRM":
        return None
    values = parts[1:]
    if len(values) != len(UGWRM_FIELDS):
        return None
    row = {}
    try:
        for name, raw in zip(UGWRM_FIELDS, values):
            row[name] = int(raw) if name in UGWRM_INT_FIELDS else float(raw)
    except ValueError:
        return None
    if row["schema"] != 2:
        return None
    return row


def ugwrm_csv_row(row):
    return [datetime.now().isoformat(timespec="milliseconds")] + [
        row[name] for name in UGWRM_FIELDS]

PID_MODE_NAMES = {0: "OFF", 1: "PITCH_HOLD", 2: "DEPTH_CASCADE"}
PID_PHASE_NAMES = {0: "DIVE", 1: "CLIMB"}
PID_FAULT_NAMES = {
    0: "NONE",
    1: "DISARMED",
    2: "COMMAND_TIMEOUT",
    3: "PID_NOT_VALID",
    4: "PID_CRC_MISMATCH",
    5: "EKF_INVALID",
    6: "DEPTH_STALE",
    7: "BAD_COMMAND",
    8: "CORE_REJECTED",
    9: "DISPLACEMENT_INVALID",
}
PID_CORE_FAULT_NAMES = {
    0: "NONE",
    1: "DISARMED",
    2: "EKF_INVALID",
    3: "BAD_CONFIG",
    4: "BAD_COMMAND",
    5: "BAD_DT",
    6: "DEPTH_SAMPLE_INVALID",
    7: "DISPLACEMENT_INVALID",
}
PID_CONFIG_REJECT_NAMES = {
    0: "NONE",
    1: "ARMED",
    2: "CRC",
    3: "VALUE",
    4: "NEUTRAL_DISPLACEMENT",
}


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


def ugctl_parse(line):
    """严格解析 schema v2 的 43 字段 $UGCTL。"""
    if not ekf_checksum_ok(line):
        return None
    star = line.rfind("*")
    parts = line[1:star].split(",")
    if not parts or parts[0] != "UGCTL":
        return None
    values = parts[1:]
    if len(values) != len(UGCTL_FIELDS):
        return None
    row = {}
    try:
        for name, raw in zip(UGCTL_FIELDS, values):
            row[name] = int(raw) if name in UGCTL_INT_FIELDS else float(raw)
    except ValueError:
        return None
    if row["schema"] != 2:
        return None
    return row


def ugctl_csv_row(row):
    return [datetime.now().isoformat(timespec="milliseconds")] + [
        row[name] for name in UGCTL_FIELDS]


class UgctlStore:
    """由唯一 socket 接收线程发布，PID 界面只等待/读取，不直接 recv。"""

    def __init__(self):
        self._condition = threading.Condition()
        self._generation = 0
        self._latest = None
        self._received_at = None

    def publish(self, row):
        with self._condition:
            self._latest = dict(row)
            self._received_at = time.monotonic()
            self._generation += 1
            self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            latest = None if self._latest is None else dict(self._latest)
            return self._generation, latest

    def snapshot_with_time(self):
        """带本机接收时刻的快照；旧 PID 使用的二元 snapshot 接口不变。"""
        with self._condition:
            latest = None if self._latest is None else dict(self._latest)
            return self._generation, latest, self._received_at

    def wait_for(self, predicate, after_generation, timeout_s):
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                if (self._generation > after_generation and
                        self._latest is not None and
                        predicate(self._latest)):
                    return dict(self._latest)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)


class UgwrmStore(UgctlStore):
    """蜗杆遥测独立缓存，复用同样的条件变量、代数和接收时间机制。"""


def ekf_csv_row(row):
    """把解析结果 + fl 位展开成与 EKF_CSV_HEADER 对齐的列表。"""
    fl = row["fl"]
    out = [datetime.now().isoformat(timespec="milliseconds"),
           row["seq"], row["t_ms"], row["status"],
           EKF_STATUS_NAMES.get(row["status"], "?"),
           (fl >> 0) & 1, (fl >> 1) & 1, (fl >> 2) & 1, (fl >> 3) & 1, (fl >> 4) & 1]
    out.extend(row[name] for name in EKF_FIELDS[4:])
    return out


def print_ugctl_status(row):
    if row is None:
        print("[PID] 尚未收到 $UGCTL；不能确认参数或 ARM 状态")
        return
    print(
        "[PID/板端] mode=%s phase=%s armed=%d valid=%d "
        "fault=%s core=%s age=%dms cmd=%d"
        % (
            PID_MODE_NAMES.get(row["mode"], "?"),
            PID_PHASE_NAMES.get(row["phase"], "?"),
            row["armed"],
            row["output_valid"],
            PID_FAULT_NAMES.get(row["fault"], str(row["fault"])),
            PID_CORE_FAULT_NAMES.get(
                row["core_fault"], str(row["core_fault"])),
            row["age_ms"],
            row["command_id"],
        )
    )
    print(
        "  depth target/meas/error = %.3f / %.3f / %.3f m; "
        "pitch target/meas/error = %.2f / %.2f / %.2f deg"
        % (
            row["target_depth"], row["depth"], row["depth_error"],
            row["pitch_sp_deg"], row["pitch_deg"], row["pitch_error_deg"],
        )
    )
    print(
        "  buoyancy P/I/D/out = %.3f / %.3f / %.3f / %.3f mm; "
        "inner P/I/D/out = %.2f / %.2f / %.2f / %.2f RPM"
        % (
            row["outer_p_mm"], row["outer_i_mm"],
            row["outer_d_mm"], row["outer_out_mm"],
            row["inner_p_rpm"], row["inner_i_rpm"],
            row["inner_d_rpm"], row["inner_out_rpm"],
        )
    )
    print(
        "  displacement target/meas/error = %.3f / %.3f / %.3f mm"
        % (
            row["target_displacement_mm"], row["displacement_mm"],
            row["displacement_error_mm"],
        )
    )
    print(
        "  worm=%d RPM pump=(%d,%d) thrust=%+.2f%% sat=(outer:%d inner:%d)"
        % (
            row["worm_rpm"], row["pump_enable"], row["pump_direction"],
            row["thrust_pct"], row["outer_sat"], row["inner_sat"],
        )
    )
    print(
        "  PID valid=%d required_crc=0x%08X active_crc=0x%08X "
        "config(req/apply/reject)=%d/%d/%d reject_reason=%s"
        % (
            row["pid_valid"], row["required_crc"], row["active_crc"],
            row["config_req"], row["config_apply"],
            row["config_reject_count"],
            PID_CONFIG_REJECT_NAMES.get(
                row["config_reject"], str(row["config_reject"])),
        )
    )


class PidSession:
    """PID 子界面的命令租约，即心跳包、参数事务和安全状态。"""

    def __init__(self, conn, ctl_store, send_period_s=PID_COMMAND_PERIOD_S):
        self.conn = conn
        self.ctl_store = ctl_store
        self.send_period_s = float(send_period_s)
        self._state_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._worker = None
        self._command_id = 0
        self._transaction = int(time.time()) & 0xFF
        self._mode = CASCADE_MODE_PITCH_HOLD
        self._armed = False
        self._board_may_be_armed = False
        self._target_depth_m = 0.0
        self._target_pitch_deg = 0.0
        self._signed_thrust_percent = 0.0
        self._loaded_path = None
        self._parameters = None
        self._parameter_table = None
        self._loaded_crc = None
        self._confirmed_crc = None

    def start(self):
        if self._worker is None:
            self._worker = threading.Thread(
                target=self._worker_loop,
                name="pid-command-lease",
                daemon=True,
            )
            self._worker.start()

    def _next_command_id_locked(self):
        self._command_id = (self._command_id + 1) & 0xFFFF
        return self._command_id

    def _snapshot_command_locked(self, force_off=False):
        return {
            "mode": CASCADE_MODE_OFF if force_off else self._mode,
            "armed": False if force_off else self._armed,
            "command_id": self._command_id,
            "target_depth_m": self._target_depth_m,
            "target_pitch_deg": self._target_pitch_deg,
            "signed_thrust_percent": (
                0.0 if force_off else self._signed_thrust_percent),
            "required_pid_crc": self._confirmed_crc or 0,
        }

    def local_snapshot(self):
        with self._state_lock:
            return {
                "mode": self._mode,
                "armed": self._armed,
                "command_id": self._command_id,
                "target_depth_m": self._target_depth_m,
                "target_pitch_deg": self._target_pitch_deg,
                "signed_thrust_percent": self._signed_thrust_percent,
                "loaded_path": self._loaded_path,
                "loaded_crc": self._loaded_crc,
                "confirmed_crc": self._confirmed_crc,
                "parameters": (
                    None if self._parameters is None
                    else dict(self._parameters)),
            }

    def _send_command_dict(self, command, verbose=False):
        return send_cascade_command(
            self.conn,
            command["mode"],
            command["armed"],
            command["command_id"],
            command["target_depth_m"],
            command["target_pitch_deg"],
            command["signed_thrust_percent"],
            command["required_pid_crc"],
            verbose=verbose,
        )

    def _worker_loop(self):
        while not self._stop.is_set():
            self._wake.wait(self.send_period_s)
            self._wake.clear()
            if self._stop.is_set():
                break
            with self._send_lock:
                with self._state_lock:
                    if not self._armed:
                        continue
                    command = self._snapshot_command_locked()
                if not self._send_command_dict(command, verbose=False):
                    with self._state_lock:
                        self._armed = False
                    print("\n[PID 安全停机] 命令刷新发送失败，停止本地心跳包；板端将在 500ms 内超时归零")

    def _mark_semantic_change(self):
        with self._state_lock:
            self._next_command_id_locked()
        self._wake.set()

    def load_parameters(self, path):
        with self._state_lock:
            if self._armed:
                raise RuntimeError("请先 disarm，再加载或修改参数")
        path = os.path.abspath(os.path.expanduser(path))
        with open(path, "r", encoding="utf-8") as handle:
            parameters = json.load(handle)
        table = pack_pid_param_table(parameters)
        normalized = {
            name: float(parameters[name]) for name in PID_PARAM_FIELDS}
        crc = crc32_iso_hdlc(table)
        with self._state_lock:
            self._loaded_path = path
            self._parameters = normalized
            self._parameter_table = table
            self._loaded_crc = crc
            if self._confirmed_crc != crc:
                self._confirmed_crc = None
        return crc

    def set_parameter(self, name, value):
        with self._state_lock:
            if self._armed:
                raise RuntimeError("请先 disarm，再修改 PID 参数")
            if self._parameters is None:
                raise RuntimeError("请先 load 一个 PID 参数 JSON")
            candidate = dict(self._parameters)
        if name not in PID_PARAM_FIELDS:
            raise ValueError(
                "未知参数；可用 params 查看 16 个合法字段名")
        candidate[name] = float(value)
        table = pack_pid_param_table(candidate)
        crc = crc32_iso_hdlc(table)
        with self._state_lock:
            self._parameters = candidate
            self._parameter_table = table
            self._loaded_crc = crc
            self._confirmed_crc = None
        return crc

    def save_parameters(self, path):
        with self._state_lock:
            if self._parameters is None:
                raise RuntimeError("当前没有可保存的参数；请先 load")
            parameters = dict(self._parameters)
        path = os.path.abspath(os.path.expanduser(path))
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(parameters, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        return path
     #通过上位机显式切换双/内环PID控制两个模式
    def set_pitch_target(self, target_pitch_deg):
        target = float(target_pitch_deg)
        if not -90.0 <= target <= 90.0:
            raise ValueError("俯仰目标必须在 -90..90 deg")
        with self._state_lock:
            if (self._parameters is not None and
                    abs(target) > self._parameters["max_pitch_deg"]):
                raise ValueError(
                    "俯仰目标超过当前 max_pitch_deg=%.2f"
                    % self._parameters["max_pitch_deg"])
            self._mode = CASCADE_MODE_PITCH_HOLD
            self._target_pitch_deg = target
            self._next_command_id_locked()
        self._wake.set()

    def set_depth_target(self, target_depth_m):
        target = float(target_depth_m)
        if not -0.5 <= target <= 1000.0:
            raise ValueError("深度目标必须在 0..1000 m")
        with self._state_lock:
            if self._parameters is not None:
                minimum = (
                    self._parameters["surface_depth_m"] +
                    self._parameters["depth_switch_threshold_m"])
                if target <= minimum:
                    raise ValueError(
                        "最大下潜深度必须大于 surface_depth + threshold = %.3f m"
                        % minimum)
            self._mode = CASCADE_MODE_DEPTH
            self._target_depth_m = target
            self._next_command_id_locked()
        self._wake.set()

    def set_thrust(self, signed_percent):
        value = float(signed_percent)
        if not -100.0 <= value <= 100.0:
            raise ValueError("推进器输出必须在 -100..100%")
        with self._state_lock:
            if (self._parameters is not None and
                    abs(value) > self._parameters["max_thrust_percent"]):
                raise ValueError(
                    "推进器输出超过当前 max_thrust_percent=%.2f%%"
                    % self._parameters["max_thrust_percent"])
            self._signed_thrust_percent = value
            self._next_command_id_locked()
        self._wake.set()

    def disarm(self, verbose=True, force=False):
        with self._send_lock:
            with self._state_lock:
                if not force and not self._board_may_be_armed:
                    self._armed = False
                    return True
                self._armed = False
                self._next_command_id_locked()
                command = self._snapshot_command_locked(force_off=True)
            ok = self._send_command_dict(command, verbose=verbose)
            if ok:
                with self._state_lock:
                    self._board_may_be_armed = False
            return ok

    def _ensure_board_off(self):
        generation, row = self.ctl_store.snapshot()
        with self._state_lock:
            may_be_armed = self._board_may_be_armed
        need_off = (may_be_armed or row is None or row["mode"] != 0 or
                    row["armed"] != 0)
        if need_off and not self.disarm(verbose=True, force=True):
            return None
        if not need_off:
            return row
        return self.ctl_store.wait_for(
            lambda value: value["mode"] == 0 and value["armed"] == 0,
            generation,
            PID_ACK_TIMEOUT_S,
        )

    def push_parameters(self):
        with self._state_lock:
            if self._parameter_table is None:
                raise RuntimeError("请先 load PID 参数 JSON")
            table = bytes(self._parameter_table)
            expected_crc = self._loaded_crc

        off_row = self._ensure_board_off()
        if off_row is None:
            print("[PID 参数] 未收到 OFF 的 $UGCTL 确认，取消参数下发")
            return False

        with self._state_lock:
            self._transaction = (self._transaction + 1) & 0xFF
            transaction = self._transaction
        generation, before = self.ctl_store.snapshot()
        before_apply = before["config_apply"] if before else None
        before_reject = before["config_reject_count"] if before else None

        sent_crc = send_pid_param_table(self.conn, table, transaction)
        if sent_crc is None:
            return False
        if sent_crc != expected_crc:
            print("[PID 参数] 本地 CRC 在发送期间发生变化，拒绝确认")
            return False

        def transaction_finished(row):
            if before_apply is None or before_reject is None:
                return (row["active_crc"] == expected_crc and
                        row["pid_valid"] == 1)
            return (row["config_apply"] != before_apply or
                    row["config_reject_count"] != before_reject)

        ack = self.ctl_store.wait_for(
            transaction_finished,
            generation,
            PID_ACK_TIMEOUT_S,
        )
        if ack is None:
            print("[PID 参数] 等待 $UGCTL ACK 超时；参数身份未确认，禁止 ARM")
            return False
        if (ack["pid_valid"] != 1 or
                ack["active_crc"] != expected_crc or
                (before_reject is not None and
                 ack["config_reject_count"] != before_reject)):
            print(
                "[PID 参数] 板端拒绝或 CRC 不匹配：active=0x%08X reason=%s"
                % (
                    ack["active_crc"],
                    PID_CONFIG_REJECT_NAMES.get(
                        ack["config_reject"], str(ack["config_reject"])),
                )
            )
            return False
        with self._state_lock:
            if self._loaded_crc != expected_crc:
                print("[PID 参数] ACK 到达时本地参数已变化，必须重新 push")
                return False
            self._confirmed_crc = expected_crc
        print("[PID 参数] 已原子应用并确认 active_crc=0x%08X" % expected_crc)
        return True

    def arm(self):
        with self._send_lock:
            with self._state_lock:
                if self._confirmed_crc is None or self._parameters is None:
                    print("[PID] 必须先 load + push，并收到 CRC ACK 后才能 arm")
                    return False
                if (abs(self._signed_thrust_percent) >
                        self._parameters["max_thrust_percent"]):
                    print("[PID] 当前推进器输出超过已确认参数限幅")
                    return False
                if (self._mode == CASCADE_MODE_PITCH_HOLD and
                        abs(self._target_pitch_deg) >
                        self._parameters["max_pitch_deg"]):
                    print("[PID] 当前俯仰目标超过已确认参数限幅")
                    return False
                if self._mode == CASCADE_MODE_DEPTH:
                    minimum = (
                        self._parameters["surface_depth_m"] +
                        self._parameters["depth_switch_threshold_m"])
                    if self._target_depth_m <= minimum:
                        print("[PID] 深度目标必须大于 %.3f m" % minimum)
                        return False
                _, board = self.ctl_store.snapshot()
                if (board is None or board["pid_valid"] != 1 or
                        board["active_crc"] != self._confirmed_crc):
                    print("[PID] 板端当前 CRC/valid 与本地确认不一致，拒绝 arm")
                    return False
                self._armed = True
                self._next_command_id_locked()
                command = self._snapshot_command_locked()
                command_id = self._command_id
            generation, _ = self.ctl_store.snapshot()
            if not self._send_command_dict(command, verbose=True):
                with self._state_lock:
                    self._armed = False
                return False
            with self._state_lock:
                self._board_may_be_armed = True

        ack = self.ctl_store.wait_for(
            lambda row: row["command_id"] == command_id,
            generation,
            PID_ACK_TIMEOUT_S,
        )
        if ack is None or ack["armed"] != 1 or ack["fault"] != 0:
            if ack is None:
                print("[PID] ARM ACK 超时，立即发送 OFF")
            else:
                print(
                    "[PID] ARM 被板端拒绝：fault=%s core=%s，立即发送 OFF"
                    % (
                        PID_FAULT_NAMES.get(ack["fault"], str(ack["fault"])),
                        PID_CORE_FAULT_NAMES.get(
                            ack["core_fault"], str(ack["core_fault"])),
                    )
                )
            self.disarm(verbose=True, force=True)
            return False
        print("[PID] ARM 已确认；10Hz 命令心跳包已启动")
        self._wake.set()
        return True

    def close(self):
        with self._state_lock:
            need_off = self._board_may_be_armed
        if need_off:
            self.disarm(verbose=True, force=True)
        self._stop.set()
        self._wake.set()
        if self._worker is not None:
            self._worker.join(timeout=1.0)


def print_pid_help():
    print("\n========== PID 调试界面 ==========")
    print("  load <json>          加载并校验 18 个控制参数,不下发")
    print("  set <field> <value>  修改内存参数,应用则必须重新 push")
    print("  save <json>          保存当前内存参数")
    print("  params               显示当前 18 个参数及 CRC")
    print("  push                 OFF 后发送 5+1 参数帧并等待 CRC ACK")
    print("  pitch <deg>          选择俯仰内环；负值低头，不会启动泵")
    print("  depth <m>            选择深度外环+俯仰内环；启动油泵状态机")
    print("  thrust <signed %>    推进器；+ 为 AUTO-I 方向，- 为 AUTO-K")
    print("  thrust off           推进器目标设为 0（仍需心跳包帧送达）")
    print("  arm / disarm         启动闭环心跳包 / 立即发送一次 OFF")
    print("  status               显示控制状态并请求一次传感器输入快照")
    print("  help / back          显示帮助 / 安全退出 PID 界面")
    print("注意：推进器正负对应实物前进/后退的命名，必须先在台架确认。")
    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n")


def run_pid_mode(conn, ctl_store):
    previous_stream_mask = get_stream_control_mask()
    required_stream_mask = previous_stream_mask | STREAM_EKF | STREAM_CTL
    if not send_stream_control(conn, required_stream_mask, 'PID 遥测流控制'):
        print("[PID] 无法开启 EKF+UGCTL 遥测，取消进入")
        return

    session = PidSession(conn, ctl_store)
    session.start()
    print_pid_help()
    generation, _ = ctl_store.snapshot()
    session.disarm(verbose=True, force=True)
    first_status = ctl_store.wait_for(
        lambda row: row["mode"] == 0 and row["armed"] == 0,
        generation,
        PID_ACK_TIMEOUT_S,
    )
    if first_status is None:
        print("[PID 警告] 未收到初始 OFF 回执；可查看 status，但 ARM 会被拒绝")
    else:
        print(
            "[PID] 板端已处于 OFF；pid_valid=%d active_crc=0x%08X"
            % (first_status["pid_valid"], first_status["active_crc"]))

    PID_MODE_ACTIVE.set()
    try:
        while True:
            try:
                tokens = shlex.split(input("pid> ").strip())
            except ValueError as exc:
                print(f"[PID 输入错误] {exc}")
                continue
            if not tokens:
                continue
            command = tokens[0].lower()
            try:
                if command in ("back", "quit", "exit"):
                    break
                if command == "help":
                    print_pid_help()
                elif command == "load" and len(tokens) == 2:
                    crc = session.load_parameters(tokens[1])
                    print("[PID 参数] 已加载，未下发，crc=0x%08X" % crc)
                elif command == "set" and len(tokens) == 3:
                    crc = session.set_parameter(tokens[1], tokens[2])
                    print("[PID 参数] 已修改，旧 ACK 失效，新 crc=0x%08X" % crc)
                elif command == "save" and len(tokens) == 2:
                    print("[PID 参数] 已保存 -> %s" % session.save_parameters(tokens[1]))
                elif command == "params" and len(tokens) == 1:
                    local = session.local_snapshot()
                    if local["parameters"] is None:
                        print("[PID 参数] 尚未 load")
                    else:
                        print(json.dumps(
                            local["parameters"], ensure_ascii=False, indent=2))
                        print(
                            "loaded_crc=0x%08X confirmed_crc=%s"
                            % (
                                local["loaded_crc"],
                                ("无" if local["confirmed_crc"] is None else
                                 "0x%08X" % local["confirmed_crc"]),
                            )
                        )
                elif command == "push" and len(tokens) == 1:
                    session.push_parameters()
                elif command == "pitch" and len(tokens) == 2:
                    session.set_pitch_target(tokens[1])
                    print("[PID] 已选择 PITCH_HOLD；泵关闭，俯仰目标=%s deg" % tokens[1])
                elif command == "depth" and len(tokens) == 2:
                    session.set_depth_target(tokens[1])
                    print("[PID] 已选择 DEPTH_CASCADE；最大下潜深度=%s m" % tokens[1])
                elif command == "thrust" and len(tokens) == 2:
                    value = 0.0 if tokens[1].lower() == "off" else float(tokens[1])
                    session.set_thrust(value)
                    print("[PID] 开环推进器目标=%+.2f%%" % value)
                elif command == "arm" and len(tokens) == 1:
                    session.arm()
                elif command == "disarm" and len(tokens) == 1:
                    if session.disarm(verbose=True, force=True):
                        print("[PID] 已发送 OFF；蜗杆、泵、推进器目标归零")
                elif command == "status" and len(tokens) == 1:
                    local = session.local_snapshot()
                    print(
                        "[PID/本地] mode=%s armed=%d cmd=%d depth=%.3f m "
                        "pitch=%+.2f deg thrust=%+.2f%% confirmed_crc=%s"
                        % (
                            PID_MODE_NAMES.get(local["mode"], "?"),
                            local["armed"], local["command_id"],
                            local["target_depth_m"], local["target_pitch_deg"],
                            local["signed_thrust_percent"],
                            ("无" if local["confirmed_crc"] is None else
                             "0x%08X" % local["confirmed_crc"]),
                        )
                    )
                    _, board = ctl_store.snapshot()
                    print_ugctl_status(board)
                    # 使用独立 0x03 请求，不发送 CONTROL19，避免抢占 PID 控制权。
                    send_snapshot_request(conn)
                else:
                    print("[PID 命令] 格式错误；输入 help 查看用法")
            except (OSError, ValueError, RuntimeError) as exc:
                print(f"[PID 操作拒绝] {exc}")
    except KeyboardInterrupt:
        print("\n[PID] Ctrl+C：安全退出 PID 界面")
    finally:
        PID_MODE_ACTIVE.clear()
        session.close()
        send_stream_control(conn, previous_stream_mask, '恢复遥测流')


#接收线程函数
STATION_DIR = os.path.dirname(os.path.abspath(__file__))
EKF_LOG_DIR = os.path.join(STATION_DIR, "logs", "ekf")
CONTROL_LOG_DIR = os.path.join(STATION_DIR, "logs", "control")
BATTERY_LOG_DIR = os.path.join(STATION_DIR, "logs", "battery")


def receive_thread(conn, ctl_store=None, wrm_store=None, link_closed=None):
    """唯一接收入口，初始化失败及任何退出路径均通知控制循环连接已结束。"""
    battery_monitor = BatteryAlarmMonitor()
    try:
        _receive_thread_body(conn, ctl_store, wrm_store, battery_monitor)
    except Exception as exc:
        print(f'[接收线程异常] {exc}')
    finally:
        if link_closed is not None:
            link_closed.set()
        battery_monitor.disconnect()
        conn.close()


def _receive_thread_body(conn, ctl_store=None, wrm_store=None, battery_monitor=None):
    """负责不断接收并打印来自客户端的数据"""
    global running
    if ctl_store is None:
        ctl_store = UgctlStore()
    if wrm_store is None:
        wrm_store = UgwrmStore()
    if battery_monitor is None:
        battery_monitor = BatteryAlarmMonitor()
    assembler = SnapshotAssembler(ETHERNET_SNAPSHOT_SIZE)
    line_buffer = ""

    # —— EKF 遥测 CSV：连接起把 $UGEKF 落盘，便于离线绘图 ——
    os.makedirs(EKF_LOG_DIR, exist_ok=True)
    os.makedirs(CONTROL_LOG_DIR, exist_ok=True)
    ekf_csv_path = os.path.join(EKF_LOG_DIR, "ekf_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    ekf_csv = open(ekf_csv_path, "w", encoding="utf-8", newline="")
    ekf_csv.write(",".join(EKF_CSV_HEADER) + "\n")
    ekf_csv.flush()
    print(f"[EKF] 状态遥测记录（非磁标定） -> {ekf_csv_path}")
    ekf_stat = {"ok": 0, "bad": 0, "last_print": 0.0, "last_key": None}
    ctl_csv = None
    ctl_csv_path = None
    ctl_stat = {"ok": 0, "bad": 0}
    wrm_csv = None
    wrm_csv_path = None
    wrm_stat = {"ok": 0, "bad": 0, "last_home": None}
    battery_csv = None
    battery_csv_failed = False

    def record_battery(row):
        """告警先处理，电池日志失败只提示一次，不终止接收和控制会话。"""
        nonlocal battery_csv, battery_csv_failed
        if battery_csv_failed:
            return
        try:
            if battery_csv is None:
                os.makedirs(BATTERY_LOG_DIR, exist_ok=True)
                path = os.path.join(
                    BATTERY_LOG_DIR,
                    "battery_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv",
                )
                battery_csv = open(path, "w", encoding="utf-8", newline="")
                battery_csv.write(",".join(UGBAT_CSV_HEADER) + "\n")
                print(f"\n[电池] 电压记录 -> {path}")
            battery_csv.write(",".join(str(value) for value in ugbat_csv_row(row)) + "\n")
            battery_csv.flush()
        except (OSError, UnicodeError) as exc:
            battery_csv_failed = True
            print(f"\n[电池记录失败] {exc}；本次会话停止写电池 CSV，主动告警继续。", flush=True)
            if battery_csv is not None:
                try:
                    battery_csv.close()
                except OSError:
                    pass
                battery_csv = None

    def handle_line(raw_line):
        nonlocal ctl_csv, ctl_csv_path, wrm_csv, wrm_csv_path
        line = raw_line.strip()
        if not line:
            return

        # 电池流只由本接收线程消费，状态变化和限频提醒主动打印，无须输入命令。
        if line.startswith("$UGBAT"):
            row = ugbat_parse(raw_line)
            if row is not None:
                battery_monitor.ingest(row)
                record_battery(row)
            return

        # 蜗杆位置遥测：落 CSV，不打印（10Hz，打印会把控制台刷爆）。
        # 坏帧计入 wrm_stat["bad"]，与 EKF 的 bad 一样用于观察链路误码。
        if line.startswith("$UGWRM"):
            row = ugwrm_parse(line)
            if row is None:
                wrm_stat["bad"] += 1
                return
            wrm_store.publish(row)
            if wrm_csv is None:
                wrm_csv_path = os.path.join(
                    CONTROL_LOG_DIR,
                    "worm_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv",
                )
                wrm_csv = open(
                    wrm_csv_path, "w", encoding="utf-8", newline="")
                wrm_csv.write(",".join(UGWRM_CSV_HEADER) + "\n")
                wrm_csv.flush()
                print(f"\n[WORM] 蜗杆位置遥测记录 -> {wrm_csv_path}")
            wrm_csv.write(
                ",".join(str(value) for value in ugwrm_csv_row(row)) + "\n")
            wrm_csv.flush()
            wrm_stat["ok"] += 1
            # 回零会让电机自己跑，状态每次变化都要让人看见；不变则静默。
            home_state = row["home_state"]
            if home_state != wrm_stat["last_home"]:
                wrm_stat["last_home"] = home_state
                print()
                name = WORM_HOME_STATE_NAMES.get(home_state, home_state)
                if home_state == 4:
                    reason = WORM_HOME_FAIL_NAMES.get(
                        row["home_fail"], row["home_fail"])
                    print("[回零] %s：%s" % (name, reason))
                elif home_state == 3:
                    print("[回零] %s  零点脉冲=%d  当前 stroke_percent=%.2f%%"
                          % (name, row["pulse_home"], row["stroke_percent"]))
                else:
                    print("[回零] %s" % name)
            return

        # PID 遥测由本线程独占解析，再发布给 UgctlStore；PID 界面禁止 recv。
        if line.startswith("$UGCTL"):
            row = ugctl_parse(line)
            if row is None:
                ctl_stat["bad"] += 1
                return
            if ctl_csv is None:
                ctl_csv_path = os.path.join(
                    CONTROL_LOG_DIR,
                    "ctl_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv",
                )
                ctl_csv = open(
                    ctl_csv_path, "w", encoding="utf-8", newline="")
                ctl_csv.write(",".join(UGCTL_CSV_HEADER) + "\n")
                ctl_csv.flush()
                print(f"\n[PID] 控制遥测记录 -> {ctl_csv_path}")
            ctl_csv.write(
                ",".join(str(value) for value in ugctl_csv_row(row)) + "\n")
            ctl_csv.flush()
            ctl_stat["ok"] += 1
            ctl_store.publish(row)
            return

        # EKF 遥测行：始终落 CSV；仅在 状态/健康/就绪 变化时打印，平时静默
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
            HB = 2.0  # 心跳间隔(秒)
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
            print_snapshot_for_active_mode(snap)
            return

        print(f"\n[客户端] {line}")

    while running:
        # 即使 EKF 等主题持续有包，也必须发现电池流单独停止或损坏。
        battery_monitor.poll()
        try:
            # 设置超时，以便能定期检查 running 标志
            conn.settimeout(1.0)
            data = conn.recv(4096)
            if not data:
                print("[信息] 客户端已断开连接。")
                break

            # STM32 发的是 ASCII 文本（AT 或快照十六进制行），这里做流式按行拼接。
            # 非 ASCII 字节必须留下替代标记，不能删除坏字节后误认成完整电池帧。
            text = data.decode('ascii', errors='replace')
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
    if battery_csv is not None:
        try:
            battery_csv.close()
        except OSError:
            pass
    try:
        ekf_csv.close()
    except Exception:
        pass
    try:
        if ctl_csv is not None:
            ctl_csv.close()
        if wrm_csv is not None:
            wrm_csv.close()
    except Exception:
        pass
    conn.close()
    print("[接收线程] 已退出。")

# ==================== 主程序 ====================
def _run_console(conn, gamepad_service, ctl_store, wrm_store, link_closed):
    """主线程串行运行控制模式，手柄采集线程始终只读取设备。"""
    print("\n========== 交互控制台 ==========")
    print("仅接受下列命令；未知文本不会发送到板端")
    print("输入 'pack' 发送锁存控制包（保持到下一条命令，没有看门狗保护）")
    print("输入 'auto' 进入连续控制模式（受2.5秒看门狗保护，按 Q/ESC 退出）")
    print("输入 'auto_sti [槽位0..3] [上限%]' 进入 Xbox 模式，默认上限100%；X/ESC退出")
    print("  例：auto_sti 0 20 使用槽位0、推进限制20%；Y/A按住移动pitch，十字键左/右控制roll +50/-50RPM")
    print("输入 'pid' 进入双环 PID 调参界面（内含独立开环推进器接口）")
    print("输入 'ekfon' / 'ekfoff' 开 / 停板子 EKF 遥测")
    print("输入 'wormon' / 'wormoff' 开 / 停蜗杆位置遥测 $UGWRM（落 CSV，不打印）")
    print("电池监测默认常开：低压主动报警；预警每30秒、严重低压每10秒提醒，数据失效会另行提示")
    print("输入 'calib' 开始蜗杆回零（电机会自己走到前限位）")
    print("输入 'calibstop' 中止回零")
    print("输入 'snapshot' 请求一次系统状态快照")
    print("输入 'quit' 或 'exit' 退出程序")
    print("==================================\n")

    try:
        while running and not link_closed.is_set():
            user_input = input(">>> ").strip()
            if link_closed.is_set():
                print('[命令] 控制板已断开，结束会话。')
                break
            if not user_input:
                continue

            if user_input.lower() in ('quit', 'exit'):
                print("退出程序。")
                break
            elif user_input.lower() == 'pack':
                input_control_packet(conn)
            elif user_input.lower() == 'auto':
                run_auto_mode(conn)
            elif user_input.lower().split()[0] == 'auto_sti':
                try:
                    tokens = shlex.split(user_input)
                    if len(tokens) > 3:
                        raise ValueError('格式：auto_sti [槽位0..3] [上限%]')
                    index = int(tokens[1]) if len(tokens) >= 2 else None
                    config = GamepadConfig(
                        max_percent=float(tokens[2]) if len(tokens) == 3 else 100)
                    run_auto_sti_mode(
                        conn, gamepad_service, ctl_store, wrm_store, link_closed,
                        config=config, controller_index=index)
                except ValueError as exc:
                    print(f'[AUTO_STI 参数错误] {exc}')
            elif user_input.lower() == 'pid':
                run_pid_mode(conn, ctl_store)
            elif user_input.lower() == 'ekfoff':
                send_telem_control(conn, 0)
                print("[EKF] 已请求板子停发遥测")
            elif user_input.lower() == 'ekfon':
                send_telem_control(conn, 1)
                print("[EKF] 已请求板子开始遥测")
            elif user_input.lower() == 'wormon':
                send_stream_control(
                    conn, get_stream_control_mask() | STREAM_WRM)
                print("[WORM] 已请求板子开始发送 $UGWRM")
            elif user_input.lower() == 'wormoff':
                send_stream_control(
                    conn, get_stream_control_mask() & ~STREAM_WRM)
                print("[WORM] 已请求板子停发 $UGWRM")
            elif user_input.lower() == 'calib':
                print("[回零] 电机即将自行走到前限位")
                if input("确认开始？输入 yes 继续：").strip().lower() == 'yes':
                    send_worm_calib(conn, True)
                else:
                    print("[回零] 已取消，未发送。")
            elif user_input.lower() == 'calibstop':
                send_worm_calib(conn, False)
                print("[回零] 已请求中止")
            elif user_input.lower() == 'snapshot':
                send_snapshot_request(conn)
            else:
                print("[命令] 未识别，未发送；可用 pack/auto/auto_sti/pid/ekfon/ekfoff/wormon/wormoff/calib/calibstop/snapshot/quit")
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，退出。")


def main():
    global running
    running = True
    server_socket = conn = recv_thread = gamepad_service = None
    link_closed = threading.Event()
    try:
        # 手柄模块默认检测；未安装 XInput 或不支持的平台仍保留原键盘控制台。
        try:
            gamepad_service = GamepadService()
            gamepad_service.start()
            print('[手柄] Xbox 检测已启动；输入 auto_sti 才接管，读数检查可运行 gamepad_monitor.py。')
        except (OSError, RuntimeError) as exc:
            print(f'[手柄] 检测服务不可用：{exc}')
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('192.168.57.100', 8800))
        server_socket.listen(1)
        print('服务端已绑定 192.168.57.100:8800，等待客户端连接...')
        conn, addr = server_socket.accept()
        print(f'接收到客户端连接：{addr}')
        # 发送与接收共用有限 socket 超时；不把它视作硬实时停止保证。
        conn.settimeout(1.0)
        # 先给旧固件可识别的声明；旧固件若拒绝 BAT 位仍能保留 EKF，并明确报监测不可用。
        send_stream_control(conn, STREAM_EKF, '兼容 EKF 遥测声明', keep_battery=False)
        if send_stream_control(conn, STREAM_EKF | STREAM_BAT):
            print('[遥测] 已请求 EKF + BAT；收到有效 UGBAT 后才确认电池监测就绪')
        else:
            print('[遥测警告] EKF + BAT 声明发送失败，请检查连接后重试 ekfon')
        ctl_store, wrm_store = UgctlStore(), UgwrmStore()
        recv_thread = threading.Thread(
            target=receive_thread,
            args=(conn, ctl_store, wrm_store, link_closed), daemon=True)
        recv_thread.start()
        _run_console(conn, gamepad_service, ctl_store, wrm_store, link_closed)
    except (KeyboardInterrupt, EOFError):
        print('\n[退出] 控制台关闭。')
    finally:
        # 清理覆盖初始化、等待连接和交互阶段，避免 Ctrl+C 后留下采集线程。
        try:
            if conn is not None and not link_closed.is_set():
                print('[退出] 正在发送中立命令并释放 Ethernet 控制权...')
                send_neutral_and_release(conn)
                send_stream_control(conn, 0, keep_battery=False)
        finally:
            running = False
            if gamepad_service is not None:
                try:
                    gamepad_service.stop()
                except (OSError, RuntimeError) as exc:
                    print(f'[退出] 手柄线程清理失败：{exc}')
            if conn is not None:
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    conn.close()
                except OSError:
                    pass
            if recv_thread is not None and recv_thread.ident is not None:
                recv_thread.join(timeout=2)
            if server_socket is not None:
                server_socket.close()
            print('服务端已关闭。')

if __name__ == "__main__":
    main()
