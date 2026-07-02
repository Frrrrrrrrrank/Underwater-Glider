#ifndef PRESSURE_SENSOR_DRIVER_H
#define PRESSURE_SENSOR_DRIVER_H

#include "main.h"
#include <stdint.h>

#define PRESSURE_SENSOR_RX_RING_SIZE 512U
#define PRESSURE_SENSOR_MODBUS_REQUEST_PERIOD_MS 500U
#define PRESSURE_SENSOR_MODBUS_RESPONSE_TIMEOUT_MS 120U
#define PRESSURE_SENSOR_MODBUS_DEFAULT_ADDR 250U

typedef struct
{
  uint32_t initialized;
  uint32_t slave_address;
  uint32_t last_error;
  uint32_t uart_error_code;

  uint32_t tx_in_flight;
  uint32_t response_waiting;

  uint32_t rx_irq_count;
  uint32_t rx_overflow_count;

  uint32_t request_count;
  uint32_t response_ok_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t exception_count;
  uint32_t last_exception_code;

  uint32_t last_request_tick;
  uint32_t last_response_tick;
} PressureSensorDiag;

typedef struct
{
  uint32_t last_irq_byte;
  uint32_t ring_available_bytes;

  uint32_t pressure_valid;
  float pressure_bar;
  uint32_t temperature_valid;
  float temperature_c;
  uint32_t last_update_tick;

  uint8_t last_tx_data[16];
  uint32_t last_response_len;
  uint8_t last_response_data[16];
} PressureSensorReadout;

typedef struct
{
  PressureSensorDiag diag;
  PressureSensorReadout readout;
} PressureSensorState;

extern volatile PressureSensorState g_pressure_sensor_state;

#define g_pressure_sensor_diag g_pressure_sensor_state.diag
#define g_pressure_sensor_readout g_pressure_sensor_state.readout

void PressureSensorDriver_Init(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnError(UART_HandleTypeDef *huart);
void PressureSensorDriver_Poll(uint32_t now_ms);

uint16_t PressureSensorDriver_Read(uint8_t *dst, uint16_t max_len);
uint16_t PressureSensorDriver_Available(void);
void PressureSensorDriver_ClearRxBuffer(void);

#endif /* PRESSURE_SENSOR_DRIVER_H */
