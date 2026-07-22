#ifndef ETHERNET_UART_DRIVER_H
#define ETHERNET_UART_DRIVER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ETHERNET_UART_RX_RING_SIZE 256U
#define ETHERNET_UART_RX_LINE_MAX 96U
#define ETHERNET_UART_TX_LINE_MAX 160U
#define ETHERNET_UART_TASK_PERIOD_MS 20U
#define ETHERNET_UART_HEARTBEAT_PERIOD_MS 1000U

/* EKF 遥测定义，单行 ASCII 帧上限（NMEA 风格 $UGEKF,...*HH），以及周期(ms)。
 * 行内全部字段定点格式化，正常约 200B，留足余量到 512；100ms=10Hz。 */
#define ETHERNET_UART_EKF_LINE_MAX 512U
#define ETHERNET_UART_EKF_PERIOD_MS 100U

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
  uint32_t pitchmotor_speed_rpm;
  uint8_t rollmotor_enable;
  uint32_t rollmotor_speed_rpm;
  uint8_t thrustmotor_direction;
  uint8_t thrustmotor_direction2;
  uint8_t thrustmotor1se_speed_rpm;
  uint8_t thrustmotor2se_speed_rpm;
  uint8_t backup_for_thrust;
  uint8_t pump_direction;
  uint8_t pump_on_off;
  uint8_t ask_for_status;
  uint8_t eth_control_flag;

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
  uint16_t servo_pulse_us;
  uint8_t limit_switch_bits;
  uint8_t displacement_valid;
  uint8_t gnrmc_valid;
  uint8_t pressure_valid;
  uint8_t servo_mode;
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
} EthernetUartSystemSnapshot;

extern volatile EthernetUartState g_ethernet_uart_state;
extern volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
extern volatile uint8_t g_ethernet_rx_message_flag;

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
void EthernetUartDriver_ClearRxRing(void);
bool EthernetUartDriver_ConvertFront19BytesToPack(EthernetUartReadoutPack *out_pack);
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnError(UART_HandleTypeDef *huart);

#endif /* ETHERNET_UART_DRIVER_H */
