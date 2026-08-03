/*
 * displacement_sensor.c
 *
 * MPS-XS-400mm MODBUS-RTU driver on USART3 / RS485_Bus3.
 */

#include "displacement_sensor.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define DISPLACEMENT_RX_EVENT_COMPLETE  (1UL << 0)
#define DISPLACEMENT_RX_EVENT_ERROR     (1UL << 1)

volatile DisplacementSensorData g_displacement_sensor = {
  0.0f,
  DISPLACEMENT_SENSOR_ID,
  0U
};

volatile DisplacementSensorDiag g_displacement_diag = {
  0
};

static DisplacementSensorInternal s_sensor = {
  0
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

  if (frame == NULL || frame_len < DISPLACEMENT_RESPONSE_FRAME_LEN)
  {
    s_sensor.last_error = DISPLACEMENT_ERROR_FRAME_SHORT;
    return 0U;
  }

  if (frame[0] != DISPLACEMENT_SENSOR_ID)
  {
    s_sensor.last_error = DISPLACEMENT_ERROR_SENSOR_ID;
    return 0U;
  }

  if (frame[1] != MODBUS_FUNC_READ_HOLDING_REGS)
  {
    s_sensor.last_error = DISPLACEMENT_ERROR_FUNCTION;
    return 0U;
  }

  if (frame[2] != 32U)
  {
    s_sensor.last_error = DISPLACEMENT_ERROR_BYTE_COUNT;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)frame[36] << 8) | frame[35]);
  calc_crc = Displacement_CRC16(frame, 35U);
  if (recv_crc != calc_crc)
  {
    s_sensor.crc_error_count++;
    s_sensor.last_error = DISPLACEMENT_ERROR_CRC;
    return 0U;
  }

  reg8_high = (uint16_t)(((uint16_t)frame[19] << 8) | frame[20]);
  reg9_low  = (uint16_t)(((uint16_t)frame[21] << 8) | frame[22]);
  bits = ((uint32_t)reg8_high << 16) | reg9_low;
  memcpy(&displacement, &bits, sizeof(float));

  if (!isfinite(displacement))
  {
    s_sensor.last_error = DISPLACEMENT_ERROR_VALUE_INVALID;
    return 0U;
  }

  g_displacement_sensor.displacement_mm = displacement;
  g_displacement_sensor.sensor_id = frame[0];
  g_displacement_sensor.valid = 1U;

  s_sensor.last_error = DISPLACEMENT_ERROR_NONE;
  s_sensor.last_uart_error = HAL_UART_ERROR_NONE;
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
}

/*
 * ISR 只发布事件，Usart3Task 在这里一次性取走。
 * 临界区很短，只保护一个 32 位标志，避免完成/错误事件被任务清除时丢失。
 */
static uint32_t Displacement_TakeRxEvents(void)
{
  uint32_t events;

  taskENTER_CRITICAL();
  events = s_sensor.rx_events;
  s_sensor.rx_events = 0U;
  taskEXIT_CRITICAL();
  return events;
}

static uint16_t Displacement_GetReceivedLength(void)
{
  uint16_t remaining;

  if (s_sensor.huart == NULL)
  {
    return 0U;
  }

  remaining = s_sensor.huart->RxXferCount;
  if (remaining > DISPLACEMENT_RESPONSE_FRAME_LEN)
  {
    return 0U;
  }
  return (uint16_t)(DISPLACEMENT_RESPONSE_FRAME_LEN - remaining);
}

/* 任务上下文取消当前事务；不会在 ISR 中解析或重新挂接。 */
static void Displacement_CancelReceive(void)
{
  taskENTER_CRITICAL();
  if (s_sensor.huart != NULL && s_sensor.rx_pending != 0U)
  {
    s_sensor.rx_data_len = Displacement_GetReceivedLength();
    (void)HAL_UART_AbortReceive(s_sensor.huart);
  }
  s_sensor.rx_pending = 0U;
  s_sensor.rx_events = 0U;
  taskEXIT_CRITICAL();
}

/*
 * 在屏蔽 USART3 IRQ 的短临界区内再次检查事件，避免第 37 字节
 * 恰好到达时同时被任务判成超时。
 */
static uint8_t Displacement_AbortIfTimedOut(uint32_t now_ms)
{
  uint8_t timed_out = 0U;

  taskENTER_CRITICAL();
  if (s_sensor.rx_pending != 0U &&
      s_sensor.rx_events == 0U &&
      (now_ms - s_sensor.request_tick) >= DISPLACEMENT_SENSOR_TIMEOUT_MS)
  {
    s_sensor.rx_data_len = Displacement_GetReceivedLength();
    (void)HAL_UART_AbortReceive(s_sensor.huart);
    s_sensor.rx_pending = 0U;
    timed_out = 1U;
  }
  taskEXIT_CRITICAL();
  return timed_out;
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
  }
  g_displacement_diag.uart_error_code = s_sensor.last_uart_error;
  g_displacement_diag.rx_pending = s_sensor.rx_pending;
  g_displacement_diag.rx_complete_count = s_sensor.rx_complete_count;
  g_displacement_diag.uart_error_count = s_sensor.uart_error_count;
  g_displacement_diag.rx_arm_fail_count = s_sensor.rx_arm_fail_count;
  g_displacement_diag.request_tick = s_sensor.request_tick;

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
  uint8_t tx_frame[DISPLACEMENT_REQUEST_FRAME_LEN] = {0};
  uint16_t tx_len;
  uint32_t rx_events;
  HAL_StatusTypeDef status;

  if (s_sensor.initialized == 0U || s_sensor.huart == NULL)
  {
    return;
  }

  /* 同一 IRQ 中可能先完成、后报错，因此错误事件必须优先。 */
  rx_events = Displacement_TakeRxEvents();
  if ((rx_events & DISPLACEMENT_RX_EVENT_ERROR) != 0U)
  {
    Displacement_CancelReceive();
    s_sensor.comm_ok = 0U;
    s_sensor.last_error = DISPLACEMENT_ERROR_UART;
    g_displacement_sensor.valid = 0U;
    /* Modbus 无事务号；错误后留一周期静默，丢弃可能迟到的旧响应。 */
    s_sensor.last_poll_tick = now_ms;
    Displacement_SyncDiag();
    return;
  }
  else if ((rx_events & DISPLACEMENT_RX_EVENT_COMPLETE) != 0U)
  {
    s_sensor.rx_pending = 0U;
    s_sensor.rx_data_len = DISPLACEMENT_RESPONSE_FRAME_LEN;
    if (Displacement_ParseResponse(
            s_sensor.rx_buffer, s_sensor.rx_data_len) == 0U)
    {
      /* 完整长度不等于有效数据；格式或 CRC 错误也必须停泵。 */
      s_sensor.comm_ok = 0U;
      g_displacement_sensor.valid = 0U;
    }
  }

  if (Displacement_AbortIfTimedOut(now_ms) != 0U)
  {
    s_sensor.timeout_count++;
    s_sensor.comm_ok = 0U;
    s_sensor.last_error = DISPLACEMENT_ERROR_TIMEOUT;
    g_displacement_sensor.valid = 0U;
    /* 不在同一轮立即重发，避免把旧请求的迟到响应当成新响应。 */
    s_sensor.last_poll_tick = now_ms;
    Displacement_SyncDiag();
    return;
  }

  if (s_sensor.rx_pending != 0U)
  {
    Displacement_SyncDiag();
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
  memset(s_sensor.rx_buffer, 0, sizeof(s_sensor.rx_buffer));
  s_sensor.rx_data_len = 0U;
  s_sensor.rx_events = 0U;
  s_sensor.rx_pending = 1U;

  
 
   //先挂接 37B 中断接收，再发送 8B 请求，避免漏掉传感器立即返回的首字节
   //rx_buffer 是模块静态缓冲，函数返回后仍然有效
  status = HAL_UART_Receive_IT(
      s_sensor.huart,
      s_sensor.rx_buffer,
      DISPLACEMENT_RESPONSE_FRAME_LEN);
  if (status != HAL_OK)
  {
    s_sensor.rx_arm_fail_count++;
    Displacement_CancelReceive();
    s_sensor.comm_ok = 0U;
    s_sensor.last_error = DISPLACEMENT_ERROR_RX_ARM;
    g_displacement_sensor.valid = 0U;
    Displacement_SyncDiag();
    return;
  }

  s_sensor.request_tick = HAL_GetTick();
  if (HAL_UART_Transmit(s_sensor.huart, tx_frame, tx_len, 50U) == HAL_OK)
  {
    s_sensor.tx_count++;
  }
  else
  {
    Displacement_CancelReceive();
    s_sensor.comm_ok = 0U;
    s_sensor.last_error = DISPLACEMENT_ERROR_TRANSMIT;
    g_displacement_sensor.valid = 0U;
  }

  Displacement_SyncDiag();
}

void DisplacementSensor_OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_sensor.huart == NULL ||
      huart->Instance != s_sensor.huart->Instance ||
      s_sensor.rx_pending == 0U)
  {
    return;
  }

  s_sensor.rx_complete_count++;
  __DMB();
  s_sensor.rx_events |= DISPLACEMENT_RX_EVENT_COMPLETE;
}

void DisplacementSensor_OnError(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_sensor.huart == NULL ||
      huart->Instance != s_sensor.huart->Instance ||
      s_sensor.rx_pending == 0U)
  {
    return;
  }

  s_sensor.last_uart_error = huart->ErrorCode;
  s_sensor.uart_error_count++;
  s_sensor.rx_events |= DISPLACEMENT_RX_EVENT_ERROR;
}

void Displacement_StartTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    DisplacementSensor_Poll(HAL_GetTick());
    /* 这里只是任务轮询节拍，不是发送请求后的等待。 */
    osDelay(10U);
  }
}
