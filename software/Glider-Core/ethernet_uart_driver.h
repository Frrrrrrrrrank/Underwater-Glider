#ifndef ETHERNET_UART_DRIVER_H
#define ETHERNET_UART_DRIVER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ETHERNET_UART_RX_RING_SIZE 512U /* ISR 与任务之间的单生产者/单消费者环形缓冲区。 */
#define ETHERNET_UART_RX_LINE_MAX 96U
#define ETHERNET_UART_TX_LINE_MAX 160U
#define ETHERNET_UART_TASK_PERIOD_MS 20U
#define ETHERNET_UART_HEARTBEAT_PERIOD_MS 1000U

//PC -> STM32 二进制传输层UGCP，旧 19 字节控制内容保留为一种 payload，
//不再接受无帧头裸包，避免任意错位字节直接驱动执行器
#define ETHERNET_UART_FRAME_VERSION 1U
#define ETHERNET_UART_FRAME_MAX_PAYLOAD 64U
#define ETHERNET_UART_FRAME_PARTIAL_TIMEOUT_MS 100U
#define ETHERNET_UART_CONTROL_TIMEOUT_MS 2500U

/*
 * UGCP v1 线格式（所有多字节整数均为小端）：
 * UGCP | version:u8 | type:u8 | sequence:u16 | payload_length:u16
 *          | payload:N bytes | crc32:u32
 * CRC32 只覆盖 version 到 payload 末尾，不覆盖 4 字节 magic 和 CRC 字段本身。
 */
#define ETHERNET_UART_FRAME_MAGIC_SIZE 4U
#define ETHERNET_UART_FRAME_HEADER_SIZE 10U
#define ETHERNET_UART_FRAME_CRC_SIZE 4U
#define ETHERNET_UART_FRAME_MAX_SIZE (ETHERNET_UART_FRAME_HEADER_SIZE + ETHERNET_UART_FRAME_MAX_PAYLOAD + ETHERNET_UART_FRAME_CRC_SIZE)

#define ETHERNET_UART_MSG_CONTROL19_STREAM  0x01U
/* 兼容旧名称：0x01 始终表示需要周期刷新的连续/租约控制。 */
#define ETHERNET_UART_MSG_CONTROL19         ETHERNET_UART_MSG_CONTROL19_STREAM
#define ETHERNET_UART_MSG_TELEM_CONTROL     0x02U
#define ETHERNET_UART_MSG_SNAPSHOT_REQ      0x03U
#define ETHERNET_UART_MSG_CONTROL19_LATCHED 0x04U
#define ETHERNET_UART_MSG_CASCADE_COMMAND   0x05U
/* 双轴校准：沿用 0x06/1 字节，1=同时校准 pitch+roll，0=取消两轴。 */
#define ETHERNET_UART_MSG_WORM_CALIB        0x06U
/* AUTO 尾舵扩展：原 19 字节追加 int16 小端角度，单位 0.01 度；仅支持租约控制。 */
#define ETHERNET_UART_MSG_CONTROL21_STREAM  0x07U
#define ETHERNET_UART_MSG_PARAM_BLOCK     0x10U
#define ETHERNET_UART_MSG_PARAM_COMMIT    0x11U

/* 控制策略不塞进 19 字节 payload，避免破坏旧字段布局。 */
#define ETHERNET_UART_CONTROL_POLICY_NONE    0U
#define ETHERNET_UART_CONTROL_POLICY_STREAM  1U
#define ETHERNET_UART_CONTROL_POLICY_LATCHED 2U

/* TELEM_CONTROL 的 1 字节 payload 是完整状态掩码，不是增量命令。 */
#define ETHERNET_UART_STREAM_EKF 0x01U
#define ETHERNET_UART_STREAM_MAG 0x02U
#define ETHERNET_UART_STREAM_CTL 0x04U
/* bit3：双轴位置与校准遥测 $UGWRM，schema3 追加 roll 字段。 */
#define ETHERNET_UART_STREAM_WRM 0x08U
/* bit4：电池总压与采样质量 $UGBAT，不影响板端独立采样。 */
#define ETHERNET_UART_STREAM_BAT 0x10U
#define ETHERNET_UART_STREAM_RDSS 0x20U
#define ETHERNET_UART_STREAM_ALL \
  (ETHERNET_UART_STREAM_EKF | ETHERNET_UART_STREAM_MAG | ETHERNET_UART_STREAM_CTL | ETHERNET_UART_STREAM_WRM | ETHERNET_UART_STREAM_BAT | ETHERNET_UART_STREAM_RDSS)

/* EKF 遥测定义，单行 ASCII 帧上限（NMEA 风格 $UGEKF,...*HH），以及周期(ms)。
 * 行内全部字段定点格式化，正常约 200B，留足余量到 512；100ms=10Hz。 */
#define ETHERNET_UART_EKF_LINE_MAX 512U
#define ETHERNET_UART_EKF_PERIOD_MS 100U//遥测频率定义
#define ETHERNET_UART_MAG_PERIOD_MS 100U
#define ETHERNET_UART_CTL_PERIOD_MS 100U
#define ETHERNET_UART_WRM_PERIOD_MS 200U /* 双轴合并帧先用 5Hz，保留 BAT 优先调度。 */
#define ETHERNET_UART_BAT_PERIOD_MS 1000U
#define ETHERNET_UART_RDSS_PERIOD_MS 1000U

typedef struct
{
  uint32_t initialized;
  uint32_t rx_armed;
  uint32_t task_loop_count;

  uint32_t rx_irq_count;
  uint32_t rx_rearm_ok_count;
  uint32_t rx_rearm_busy_count;
  uint32_t rx_rearm_fail_count;
  uint32_t rx_byte_count;
  uint32_t rx_ring_overflow_count;
  uint32_t rx_line_count;
  uint32_t rx_line_overflow_count;

  uint32_t tx_count;
  uint32_t tx_complete_count;
  uint32_t tx_busy_count;
  uint32_t tx_fail_count;
  uint32_t tx_in_flight;

  uint32_t command_ping_count;
  uint32_t command_status_count;
  uint32_t command_help_count;
  uint32_t command_unknown_count;
  uint32_t heartbeat_count;

 // UGCP 帧解析与控制安全诊断,均由 Ethernet task 更新，供 Ozone 观察
  uint32_t frame_ok_count;
  uint32_t frame_crc_error_count;
  uint32_t frame_length_error_count;
  uint32_t frame_version_error_count;
  uint32_t frame_type_error_count;
  uint32_t frame_resync_discard_count;
  uint32_t frame_partial_timeout_count;
  uint32_t frame_seq_gap_count;
  uint32_t frame_last_seq;
  uint32_t control_packet_count;
  uint32_t control_reject_count;
  uint32_t control_payload_error_count;
  uint32_t control_timeout_count;
  uint32_t control_age_ms;
  uint32_t telemetry_control_count;
  uint32_t snapshot_request_count;
  uint32_t parameter_block_count;
  uint32_t parameter_commit_count;
  uint32_t parameter_reject_count;
  uint32_t parameter_pending_mask;
  uint32_t parameter_last_group;
  uint32_t parameter_last_transaction;
  uint32_t parameter_last_crc32;

  uint32_t ring_available_bytes;
  uint32_t last_rx_tick;
  uint32_t last_tx_tick;
  uint32_t last_error;
  uint32_t last_hal_status;
  uint32_t uart_error_code;
} EthernetUartDiag;

typedef struct
{
  uint32_t last_rx_byte;
  uint32_t ring_available_bytes;
  char last_rx_line[ETHERNET_UART_RX_LINE_MAX];
  char last_tx_line[ETHERNET_UART_TX_LINE_MAX];
} EthernetUartReadout;

typedef struct
{
  EthernetUartDiag diag;
  EthernetUartReadout readout;
} EthernetUartState;

typedef struct
{
  uint8_t pitchmotor_enable;
  int32_t pitchmotor_speed_rpm;
  uint8_t rollmotor_enable;
  int32_t rollmotor_speed_rpm;
  uint8_t thrustmotor_direction;
  uint8_t thrustmotor_direction2;
  uint8_t thrustmotor1se_speed_rpm;
  uint8_t thrustmotor2se_speed_rpm;
  uint8_t backup_for_thrust;
  uint8_t pump_direction;
  uint8_t pump_on_off;
  uint8_t ask_for_status;
  uint8_t eth_control_flag;
  /* 内部邮箱扩展，不改变旧 CONTROL19 线上布局；旧包明确清零新增字段。 */
  int16_t rudder_angle_cdeg;
  uint8_t rudder_valid;

} EthernetUartReadoutPack;

/* 系统快照：把各传感器/执行器/EKF 全局镜像定点打包，供以太网上报，
 * 也供 SD 卡日志任务周期落盘。字段单位见各成员后缀（_x10=×10，_e6=×1e6 等）。 */
typedef struct
{
  uint32_t alive_counter;
  int32_t gnrmc_latitude_e6;
  int32_t gnrmc_longitude_e6;
  uint32_t alarm_bits;
  int16_t displacement_mm_x10;
  int16_t imu_roll_x10;
  int16_t imu_pitch_x10;
  int16_t imu_yaw_x10;
  int16_t imu_height_dm;
  int16_t imu_temp_c_x10;
  uint16_t pressure_bar_x100;
  int16_t pressure_temp_c_x10;
  int16_t pump_feedback_rpm;
  int16_t umotor0_feedback_rpm;
  int16_t umotor1_feedback_rpm;
  uint16_t servo_pulse_us; /* 尾舵已下发脉宽，非实测位置。 */
  uint8_t limit_switch_bits;
  uint8_t displacement_valid;
  uint8_t gnrmc_valid;
  uint8_t pressure_valid;
  uint8_t servo_mode; /* 0=未启动，1=中位保护，2=网线尾舵命令。 */
  uint8_t pump_speed_triggered;
  uint8_t pump_direction_reversed;
  /*EKF 状态估计（供以太网快照直接观察；x=pN, y=pE 为 NED 水平位置）*/
  int32_t ekf_pN_cm;     /* x：北向位置 (cm)，Phase1 无水平观测会漂移 */
  int32_t ekf_pE_cm;     /* y：东向位置 (cm)，同上 */
  int32_t ekf_depth_mm;  /* 深度 (mm，正=水下) */
  int16_t ekf_roll_x10;  /* 横滚 (deg×10) */
  int16_t ekf_pitch_x10; /* 俯仰 (deg×10) */
  int16_t ekf_yaw_x10;   /* 航向 (deg×10) */
  uint8_t ekf_status;    /* UgEkfStatus 0..5 */
  uint8_t ekf_valid;     /* 1 = valid_for_control(RUNNING&healthy) */
  /*
   * 蜗杆(移动质量块)行程位置。新增字段一律追加在结构体末尾：
   * Python 侧 ctypes 镜像必须与此严格同序，插在中间会让全部后续字段解码错位。
   */
  int16_t worm_stroke_percent_x100; /* 行程百分比×100：0=前限位, 10000=后限位 */
  uint8_t worm_span_valid;          /* 1=本次上电已跑过两端，上面那个数才有意义 */
} EthernetUartSystemSnapshot;

extern volatile EthernetUartState g_ethernet_uart_state;
extern volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
extern volatile uint8_t g_ethernet_rx_message_flag;
extern volatile uint32_t g_ethernet_control_last_rx_tick;
/* Ozone 诊断：NONE=0、STREAM=1（受 2.5s 看门狗）、LATCHED=2（保持）。 */
extern volatile uint8_t g_ethernet_control_policy;

/* 系统快照全局实例，由 EthernetUartDriver_UpdateSystemSnapshot() 刷新 */
extern volatile EthernetUartSystemSnapshot g_ethernet_system_snapshot;

/* EKF 遥测帧累计成功发送计数，供 Ozone 确认遥测在持续推送 */
extern volatile uint32_t g_ekf_telem_tx_count;

#define g_ethernet_uart_diag g_ethernet_uart_state.diag
#define g_ethernet_uart_readout g_ethernet_uart_state.readout

void EthernetUartDriver_Init(UART_HandleTypeDef *huart);
void EthernetUartDriver_RunTask(void *argument);
void EthernetUartDriver_SendText(const char *text);
void EthernetUartDriver_SendSystemSnapshot(void);
/* 从各全局镜像刷新 g_ethernet_system_snapshot（线程上下文调用即可）。 */
void EthernetUartDriver_UpdateSystemSnapshot(void);
/* 拼一行 $UGEKF,...*HH 的 EKF 状态估计遥测并发送（线程上下文，由以太网任务周期调用）。 */
void EthernetUartDriver_SendEkfTelemetry(void);
/* 拼一行 $UGMAG,seq,t_ms,mx,my,mz*HH 的原始磁力计遥测并发送（磁标定采集用；
 * 线程上下文，由独立 MAG 遥测开关和周期调度，~10Hz）。 */
HAL_StatusTypeDef EthernetUartDriver_SendMagTelemetry(void);
/* PID 调参与命令 ACK：$UGCTL 独立于 $UGEKF，避免破坏旧字段解析。 */
HAL_StatusTypeDef EthernetUartDriver_SendControlTelemetry(void);

/* 蜗杆位置遥测 $UGWRM：位置量与总线诊断量同行，便于对照判断数据可信度。 */
HAL_StatusTypeDef EthernetUartDriver_SendWormTelemetry(void);
/* 电池状态独立低频上报，使用整数毫伏，保留现有快照布局。 */
HAL_StatusTypeDef EthernetUartDriver_SendBatteryTelemetry(void);
HAL_StatusTypeDef EthernetUartDriver_SendRdssTelemetry(void);
/* 控制/泵任务每周期先整体复制一次，避免逐字段散读时跨越一次新包发布。 */
bool EthernetUartDriver_CopyControlPack(
    EthernetUartReadoutPack *out_pack,
    uint8_t *out_eth_control_flag);
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnError(UART_HandleTypeDef *huart);

#endif /* ETHERNET_UART_DRIVER_H */
