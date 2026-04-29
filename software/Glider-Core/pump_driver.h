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

#define PUMP_MOTOR_UART3_DEFAULT_NODE_ID 0x03U
#define PUMP_MOTOR_UART3_TX_TIMEOUT_MS 50U
#define PUMP_MOTOR_UART3_RX_TIMEOUT_MS 200U
#define PUMP_MOTOR_UART3_POLL_PERIOD_MS 1000U


extern volatile PumpMotorUart3Diag g_pump_motor_uart3_diag;
extern volatile PumpDriverStatus g_pump_driver_status;

void PumpDriver_Init(UART_HandleTypeDef *huart);
void PumpDriver_Poll(uint32_t now_ms);

#endif
