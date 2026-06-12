/*
 * displacement_sensor.c
 *
 * MPS-XS-400mm MODBUS-RTU driver on USART3 / RS485_Bus3.
 */

#include "displacement_sensor.h"
#include "cmsis_os.h"

#include <string.h>

volatile DisplacementSensorData g_displacement_sensor = {
  0.0f,
  DISPLACEMENT_SENSOR_ID,
  0U
};

volatile DisplacementSensorDiag g_displacement_diag = {
  0U, 0U, 0U, 0U, 0U, 0U, 0U, {0}, 0U, {0}, 0U, 0U, 0U
};

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
} DisplacementSensorInternal;

static DisplacementSensorInternal s_sensor = {
  NULL, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, {0}, 0U
};

static uint16_t Displacement_CRC16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
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

static uint16_t Displacement_BuildReadFrame(uint8_t *frame)
{
  uint16_t crc;

  frame[0] = DISPLACEMENT_SENSOR_ID;
  frame[1] = MODBUS_FUNC_READ_HOLDING_REGS;
  frame[2] = 0x00U;
  frame[3] = 0x00U;
  frame[4] = 0x00U;
  frame[5] = 0x10U;

  crc = Displacement_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

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
    s_sensor.last_error = 1U;
    return 0U;
  }

  if (frame[0] != DISPLACEMENT_SENSOR_ID)
  {
    s_sensor.last_error = 2U;
    return 0U;
  }

  if (frame[1] != MODBUS_FUNC_READ_HOLDING_REGS)
  {
    s_sensor.last_error = 3U;
    return 0U;
  }

  if (frame[2] != 32U)
  {
    s_sensor.last_error = 4U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)frame[36] << 8) | frame[35]);
  calc_crc = Displacement_CRC16(frame, 35U);
  if (recv_crc != calc_crc)
  {
    s_sensor.crc_error_count++;
    s_sensor.last_error = 5U;
    return 0U;
  }

  reg8_high = (uint16_t)(((uint16_t)frame[19] << 8) | frame[20]);
  reg9_low  = (uint16_t)(((uint16_t)frame[21] << 8) | frame[22]);
  bits = ((uint32_t)reg8_high << 16) | reg9_low;
  memcpy(&displacement, &bits, sizeof(float));

  g_displacement_sensor.displacement_mm = displacement;
  g_displacement_sensor.sensor_id = frame[0];
  g_displacement_sensor.valid = 1U;

  s_sensor.last_error = 0U;
  s_sensor.comm_ok = 1U;
  s_sensor.rx_count++;
  g_displacement_diag.last_seen_tick = HAL_GetTick();
  return 1U;
}

static void Displacement_ClearUartRx(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
  {
    (void)huart->Instance->RDR;
  }

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_READY;
}

static void Displacement_SyncDiag(void)
{
  uint16_t copy_len;

  g_displacement_diag.last_error = s_sensor.last_error;
  g_displacement_diag.comm_ok = s_sensor.comm_ok;
  g_displacement_diag.tx_count = s_sensor.tx_count;
  g_displacement_diag.rx_count = s_sensor.rx_count;
  g_displacement_diag.timeout_count = s_sensor.timeout_count;
  g_displacement_diag.crc_error_count = s_sensor.crc_error_count;
  g_displacement_diag.last_rx_len = s_sensor.rx_data_len;

  if (s_sensor.huart != NULL)
  {
    g_displacement_diag.uart_rx_state = (uint32_t)s_sensor.huart->RxState;
    g_displacement_diag.uart_error_code = s_sensor.huart->ErrorCode;
  }

  copy_len = s_sensor.rx_data_len;
  if (copy_len > (uint16_t)sizeof(g_displacement_diag.last_rx_frame))
  {
    copy_len = (uint16_t)sizeof(g_displacement_diag.last_rx_frame);
  }
  if (copy_len > 0U)
  {
    memcpy((void *)g_displacement_diag.last_rx_frame, s_sensor.rx_buffer, copy_len);
  }
}

void DisplacementSensor_Init(UART_HandleTypeDef *huart)
{
  memset(&s_sensor, 0, sizeof(s_sensor));
  memset((void *)&g_displacement_sensor, 0, sizeof(g_displacement_sensor));
  memset((void *)&g_displacement_diag, 0, sizeof(g_displacement_diag));

  s_sensor.huart = huart;
  s_sensor.initialized = (huart != NULL) ? 1U : 0U;
  s_sensor.startup_tick = HAL_GetTick();
  g_displacement_sensor.sensor_id = DISPLACEMENT_SENSOR_ID;
}

void DisplacementSensor_Poll(uint32_t now_ms)
{
  uint8_t tx_frame[8] = {0};
  uint8_t rx_frame[40] = {0};
  uint16_t tx_len;

  if (s_sensor.initialized == 0U || s_sensor.huart == NULL)
  {
    return;
  }

  if ((now_ms - s_sensor.startup_tick) < DISPLACEMENT_SENSOR_STARTUP_DELAY_MS)
  {
    Displacement_SyncDiag();
    return;
  }

  if ((now_ms - s_sensor.last_poll_tick) < DISPLACEMENT_SENSOR_UPDATE_MS)
  {
    Displacement_SyncDiag();
    return;
  }
  s_sensor.last_poll_tick = now_ms;

  tx_len = Displacement_BuildReadFrame(tx_frame);
  g_displacement_diag.last_tx_len = tx_len;
  memcpy((void *)g_displacement_diag.last_tx_frame, tx_frame, tx_len);

  Displacement_ClearUartRx(s_sensor.huart);

  if (HAL_UART_Transmit(s_sensor.huart, tx_frame, tx_len, 50U) == HAL_OK)
  {
    s_sensor.tx_count++;
    osDelay(2U);

    if (HAL_UART_Receive(s_sensor.huart, rx_frame, 37U, DISPLACEMENT_SENSOR_TIMEOUT_MS) == HAL_OK)
    {
      s_sensor.rx_data_len = 37U;
      memcpy(s_sensor.rx_buffer, rx_frame, 37U);
      (void)Displacement_ParseResponse(s_sensor.rx_buffer, s_sensor.rx_data_len);
    }
    else
    {
      s_sensor.timeout_count++;
      s_sensor.comm_ok = 0U;
      s_sensor.last_error = 6U;
      g_displacement_sensor.valid = 0U;
    }
  }
  else
  {
    s_sensor.comm_ok = 0U;
    s_sensor.last_error = 7U;
    g_displacement_sensor.valid = 0U;
  }

  Displacement_SyncDiag();
}

void Displacement_StartTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    DisplacementSensor_Poll(osKernelGetTickCount());
    osDelay(2U);
  }
}
