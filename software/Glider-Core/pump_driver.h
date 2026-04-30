#ifndef PUMP_DRIVER_H
#define PUMP_DRIVER_H

#include "stm32h7xx_hal.h"

typedef struct
{
  uint8_t initialized;
  uint8_t node_id;
  uint8_t valid;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint8_t last_rx_frame[64];
  uint8_t last_tx_frame[16];
  uint8_t last_error;
  uint16_t status_code;
  uint16_t alarm_code;
  int32_t feedback_speed_rpm;
} PumpMotorUart3Diag;

typedef struct
{
  uint8_t online;
  uint8_t valid;
  uint8_t sensor_id;
  float displacement_mm;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t last_error;
  uint32_t last_seen_tick;
} PumpDisplacementStatus;

typedef struct
{
  uint8_t online;
  uint8_t initialized;
  uint8_t node_id;
  uint8_t valid;
  int32_t feedback_speed_rpm;
  uint32_t tx_count;
  uint32_t rx_count;
  uint16_t status_code;
  uint16_t alarm_code;
  uint32_t timeout_count;
  uint32_t last_error;
  uint32_t last_seen_tick;
} PumpMotorStatus;

typedef struct
{
  PumpDisplacementStatus displacement;
  PumpMotorStatus motor;
} PumpDriverStatus;

#define PUMP_MOTOR_UART3_DEFAULT_NODE_ID 0x01U
#define PUMP_MOTOR_UART3_TX_TIMEOUT_MS 50U
#define PUMP_MOTOR_UART3_RX_TIMEOUT_MS 200U
#define PUMP_MOTOR_UART3_POLL_PERIOD_MS 1000U

/* IO Control pins */
#define IO_PUMP_SPEED_PORT GPIOB
#define IO_PUMP_SPEED_PIN GPIO_PIN_10
#define IO_PUMP_DIR_PORT GPIOD
#define IO_PUMP_DIR_PIN GPIO_PIN_14

/* IO Control debug info */
typedef struct
{
  uint16_t right_outer_switch;    /* CH7 raw value */
  uint16_t left_mid2_switch;      /* CH6 raw value */
  uint8_t speed_triggered;        /* 1 when speed control active */
  uint8_t direction_reversed;     /* 1 when direction reversed */
} IoControlPumpDebugInfo;

extern volatile IoControlPumpDebugInfo g_io_pump_debug;

extern volatile PumpMotorUart3Diag g_pump_motor_uart3_diag;
extern volatile PumpDriverStatus g_pump_driver_status;

void PumpDriver_Init(UART_HandleTypeDef *huart);
void PumpDriver_Poll(uint32_t now_ms);

/* 高级控制接口 */
uint8_t PumpDriver_Start(void);
uint8_t PumpDriver_Stop(void);
uint8_t PumpDriver_SetSpeed(uint16_t speed_240rps);  /* 单位: 1/240 RPS */
uint8_t PumpDriver_ReadSpeed(int32_t *out_speed_rpm); /* 输出单位: RPM */

/* IO 控制接口 */
void IoControlPump_Init(void);
void IoControlPump_Poll(uint32_t now_ms);
void IoControlPump_SetSpeed(uint8_t enabled);  /* 1=拉低PB10, 0=拉高PB10 */
void IoControlPump_SetDirection(uint8_t reversed);  /* 1=拉低PD14, 0=拉高PD14 */

#endif
