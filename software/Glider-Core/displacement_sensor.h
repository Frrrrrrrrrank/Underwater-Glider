/*
 * displacement_sensor.h
 *
 * MPS-XS-400mm linear displacement sensor on USART3 / RS485_Bus3.
 */

#ifndef DISPLACEMENT_SENSOR_H
#define DISPLACEMENT_SENSOR_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#define DISPLACEMENT_SENSOR_ID          0x02U
#define DISPLACEMENT_SENSOR_BAUD_RATE   115200U
#define DISPLACEMENT_SENSOR_TIMEOUT_MS  100U
#define DISPLACEMENT_SENSOR_UPDATE_MS   100U
#define DISPLACEMENT_SENSOR_STARTUP_DELAY_MS 1500U
#define DISPLACEMENT_REQUEST_FRAME_LEN  8U
#define DISPLACEMENT_RESPONSE_FRAME_LEN 37U

#define DISPLACEMENT_REG_VALUE_FLOAT_H  0x0008U
#define DISPLACEMENT_REG_VALUE_FLOAT_L  0x0009U

#define MODBUS_FUNC_READ_HOLDING_REGS   0x03U

/* Ozone 中 last_error 的含义；UART 原始错误位另看 uart_error_code。 */
typedef enum
{
  DISPLACEMENT_ERROR_NONE = 0U,
  DISPLACEMENT_ERROR_FRAME_SHORT = 1U,
  DISPLACEMENT_ERROR_SENSOR_ID = 2U,
  DISPLACEMENT_ERROR_FUNCTION = 3U,
  DISPLACEMENT_ERROR_BYTE_COUNT = 4U,
  DISPLACEMENT_ERROR_CRC = 5U,
  DISPLACEMENT_ERROR_TIMEOUT = 6U,
  DISPLACEMENT_ERROR_TRANSMIT = 7U,
  DISPLACEMENT_ERROR_RX_ARM = 8U,
  DISPLACEMENT_ERROR_UART = 9U,
  DISPLACEMENT_ERROR_VALUE_INVALID = 10U
} DisplacementSensorError;

typedef struct
{
  float displacement_mm;
  uint8_t sensor_id;
  uint32_t valid;
} DisplacementSensorData;

typedef struct
{
  uint32_t last_error;
  uint32_t comm_ok;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint16_t last_rx_len;
  uint8_t last_rx_frame[40];
  uint16_t last_tx_len;
  uint8_t last_tx_frame[8];
  uint32_t uart_rx_state;
  uint32_t uart_error_code;
  uint32_t last_seen_tick;
  uint32_t rx_pending;
  uint32_t rx_complete_count;
  uint32_t uart_error_count;
  uint32_t rx_arm_fail_count;
  uint32_t request_tick;
} DisplacementSensorDiag;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint32_t initialized;
  uint32_t startup_tick;
  uint32_t comm_ok;
  uint32_t last_error;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t last_poll_tick;
  uint8_t rx_buffer[40];
  uint16_t rx_data_len;
  volatile uint32_t rx_events;
  volatile uint32_t rx_pending;
  uint32_t request_tick;
  volatile uint32_t rx_complete_count;
  volatile uint32_t uart_error_count;
  uint32_t rx_arm_fail_count;
  volatile uint32_t last_uart_error;
} DisplacementSensorInternal;

extern volatile DisplacementSensorData g_displacement_sensor;
extern volatile DisplacementSensorDiag g_displacement_diag;

void DisplacementSensor_Init(UART_HandleTypeDef *huart);
void DisplacementSensor_Poll(uint32_t now_ms);
void DisplacementSensor_OnRxComplete(UART_HandleTypeDef *huart);
void DisplacementSensor_OnError(UART_HandleTypeDef *huart);
void Displacement_StartTask(void *argument);

#endif /* DISPLACEMENT_SENSOR_H */
