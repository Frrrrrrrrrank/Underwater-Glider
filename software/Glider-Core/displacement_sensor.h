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

#define DISPLACEMENT_REG_VALUE_FLOAT_H  0x0008U
#define DISPLACEMENT_REG_VALUE_FLOAT_L  0x0009U

#define MODBUS_FUNC_READ_HOLDING_REGS   0x03U

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
} DisplacementSensorDiag;

extern volatile DisplacementSensorData g_displacement_sensor;
extern volatile DisplacementSensorDiag g_displacement_diag;

void DisplacementSensor_Init(UART_HandleTypeDef *huart);
void DisplacementSensor_Poll(uint32_t now_ms);
void Displacement_StartTask(void *argument);

#endif /* DISPLACEMENT_SENSOR_H */
