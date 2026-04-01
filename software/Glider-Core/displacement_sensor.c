/*
 * displacement_sensor.c (I-V-485 MPS-XS-400mm)
 *
 * MODBUS-RTU over RS485 (UART3)
 * Request frame aligned with vendor tool:
 *   02 03 00 00 00 10 44 35
 */

#include "displacement_sensor.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

volatile DisplacementSensorData g_displacement_sensor = {
  0.0f,
  0x02,
  0U,
};

volatile DisplacementSensorDiag g_displacement_diag = {
  0U, 0U, 0U, 0U, 0U, 0U, 0U, {0}, 0U, {0}, 0U, 0U
};

typedef struct
{
  uint32_t initialized;
  uint32_t comm_ok;
  uint32_t last_error;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint8_t rx_buffer[256];
  uint16_t rx_data_len;
} DisplacementSensorInternal;

static DisplacementSensorInternal sensor_internal = {
  0U, 0U, 0U, 0U, 0U, 0U, 0U, {0}, 0U
};

/* Standard Modbus CRC16 (poly 0xA001, init 0xFFFF). */
static uint16_t Displacement_CRC16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/* Build request exactly as vendor tool: 02 03 00 00 00 10 44 35 */
static uint16_t Displacement_BuildReadFrame(uint8_t *frame)
{
  uint16_t crc;

  frame[0] = DISPLACEMENT_SENSOR_ID;   /* 0x02 */
  frame[1] = MODBUS_FUNC_READ_HOLDING_REGS; /* 0x03 */
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = 0x00;
  frame[5] = 0x10;

  crc = Displacement_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);       /* CRC low */
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);/* CRC high */

  return 8U;
}

/* Parse response of 16 registers read:
 * [addr][func][bytecnt=32][data 32 bytes][crcLo][crcHi]
 * Extract displacement from reg 0x0008/0x0009 in this block.
 */
static uint8_t Displacement_ParseResponse(const uint8_t *frame, uint16_t frame_len)
{
  uint16_t recv_crc;
  uint16_t calc_crc;
  uint16_t reg8_high;
  uint16_t reg9_low;
  uint32_t bits;
  float displacement;

  if (frame == NULL || frame_len < 37U)
  {
    sensor_internal.last_error = 1U;
    return 0U;
  }

  if (frame[0] != DISPLACEMENT_SENSOR_ID)
  {
    sensor_internal.last_error = 2U;
    return 0U;
  }

  if (frame[1] != MODBUS_FUNC_READ_HOLDING_REGS)
  {
    sensor_internal.last_error = 3U;
    return 0U;
  }

  if (frame[2] != 32U)
  {
    sensor_internal.last_error = 4U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)frame[36] << 8) | frame[35]);
  calc_crc = Displacement_CRC16(frame, 35U);
  if (recv_crc != calc_crc)
  {
    sensor_internal.crc_error_count++;
    sensor_internal.last_error = 5U;
    return 0U;
  }

  /* data starts at frame[3], reg index 8 => +16 bytes */
  reg8_high = (uint16_t)(((uint16_t)frame[19] << 8) | frame[20]);
  reg9_low  = (uint16_t)(((uint16_t)frame[21] << 8) | frame[22]);
  bits = ((uint32_t)reg8_high << 16) | reg9_low;
  memcpy(&displacement, &bits, sizeof(float));

  g_displacement_sensor.displacement_mm = displacement;
  g_displacement_sensor.sensor_id = frame[0];
  g_displacement_sensor.valid = 1U;

  sensor_internal.last_error = 0U;
  sensor_internal.comm_ok = 1U;
  sensor_internal.rx_count++;
  return 1U;
}

static void Displacement_SyncDiag(void)
{
  uint16_t copy_len;

  g_displacement_diag.last_error = sensor_internal.last_error;
  g_displacement_diag.comm_ok = sensor_internal.comm_ok;
  g_displacement_diag.tx_count = sensor_internal.tx_count;
  g_displacement_diag.rx_count = sensor_internal.rx_count;
  g_displacement_diag.timeout_count = sensor_internal.timeout_count;
  g_displacement_diag.crc_error_count = sensor_internal.crc_error_count;
  g_displacement_diag.last_rx_len = sensor_internal.rx_data_len;
  g_displacement_diag.uart_rx_state = (uint32_t)huart3.RxState;
  g_displacement_diag.uart_error_code = huart3.ErrorCode;

  copy_len = sensor_internal.rx_data_len;
  if (copy_len > (uint16_t)sizeof(g_displacement_diag.last_rx_frame))
  {
    copy_len = (uint16_t)sizeof(g_displacement_diag.last_rx_frame);
  }
  if (copy_len > 0U)
  {
    memcpy((void *)g_displacement_diag.last_rx_frame, sensor_internal.rx_buffer, copy_len);
  }
}

void Displacement_StartTask(void *argument)
{
  uint8_t tx_frame[8] = {0};
  uint8_t rx_frame[40] = {0};
  uint16_t tx_len;
  uint32_t last_send_tick = 0U;

  (void)argument;
  sensor_internal.initialized = 1U;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    if ((now - last_send_tick) >= DISPLACEMENT_SENSOR_UPDATE_MS)
    {
      last_send_tick = now;
      tx_len = Displacement_BuildReadFrame(tx_frame);
      g_displacement_diag.last_tx_len = tx_len;
      memcpy((void *)g_displacement_diag.last_tx_frame, tx_frame, tx_len);

      if (HAL_UART_Transmit(&huart3, tx_frame, tx_len, 50U) == HAL_OK)
      {
        sensor_internal.tx_count++;

        /* Bus turnaround for RS485 */
        osDelay(2U);

        if (HAL_UART_Receive(&huart3, rx_frame, 37U, DISPLACEMENT_SENSOR_TIMEOUT_MS) == HAL_OK)
        {
          sensor_internal.rx_data_len = 37U;
          memcpy(sensor_internal.rx_buffer, rx_frame, 37U);
          (void)Displacement_ParseResponse(sensor_internal.rx_buffer, sensor_internal.rx_data_len);
        }
        else
        {
          sensor_internal.timeout_count++;
          sensor_internal.comm_ok = 0U;
          sensor_internal.last_error = 6U;
          g_displacement_sensor.valid = 0U;
        }
      }
      else
      {
        sensor_internal.comm_ok = 0U;
        sensor_internal.last_error = 7U;
        g_displacement_sensor.valid = 0U;
      }
    }

    Displacement_SyncDiag();
    osDelay(2U);
  }
}
