/*
 * displacement_sensor.h
 * 
 * MPS-XS-400mm Linear Displacement Sensor Driver
 * MODBUS-RTU Protocol over RS485 (UART3)
 * 
 * Sensor ID: 2 (addressable on RS485 bus)
 * Communication: UART3 115200 baud, 8N1
 * Control: GPIO DE pin for RS485 direction
 */

#ifndef DISPLACEMENT_SENSOR_H
#define DISPLACEMENT_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

#define DISPLACEMENT_SENSOR_ID          0x02      /* Modbus slave address */
#define DISPLACEMENT_SENSOR_BAUD_RATE   115200
#define DISPLACEMENT_SENSOR_TIMEOUT_MS  100       /* RX timeout */
#define DISPLACEMENT_SENSOR_UPDATE_MS   100       /* Read period */

/* MODBUS Register Addresses (from datasheet) */
#define DISPLACEMENT_REG_VALUE_FLOAT_H  0x0008    /* Sensor value float high word */
#define DISPLACEMENT_REG_VALUE_FLOAT_L  0x0009    /* Sensor value float low word */

/* MODBUS Function Codes */
#define MODBUS_FUNC_READ_HOLDING_REGS   0x03

/* ============================================================================
 * Sensor Data Structure (Shared with Application)
 * ============================================================================ */

typedef struct
{
  /* Core sensor data */
  float displacement_mm;             /* Current displacement reading in millimeters */
  uint8_t sensor_id;                 /* Sensor address (from response, should be 0x02) */
  uint32_t valid;                    /* 1 if data is valid, 0 otherwise */
  
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
} DisplacementSensorDiag;

/* Global sensor data - accessible from application code */
extern volatile DisplacementSensorData g_displacement_sensor;
extern volatile DisplacementSensorDiag g_displacement_diag;

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/* Driver task entry point (called by main.c wrapper StartUsart3Task) */
void Displacement_StartTask(void *argument);

#endif /* DISPLACEMENT_SENSOR_H */

