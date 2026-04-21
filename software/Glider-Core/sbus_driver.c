#include "sbus_driver.h"

#include <string.h>

#define SBUS_START_BYTE 0x0FU
#define SBUS_FLAGS_INDEX 23U

static UART_HandleTypeDef *s_sbus_uart = NULL;
static volatile uint8_t s_sbus_rx_byte = 0U;
static volatile uint8_t s_sbus_frame_buf[SBUS_FRAME_LEN];
static volatile uint32_t s_sbus_frame_index = 0U;

volatile SbusState g_sbus_state = {0};

static void SbusDriver_ResetParser(void)
{
  s_sbus_frame_index = 0U;
}

static void SbusDriver_CopyFrameToReadout(void)
{
  uint32_t i;

  for (i = 0U; i < SBUS_FRAME_LEN; i++)
  {
    g_sbus_readout.last_frame[i] = s_sbus_frame_buf[i];
  }
}

static void SbusDriver_UpdateControlInputs(void)
{
  g_sbus_readout.controls.right_stick_x = g_sbus_readout.channels[SBUS_CH_RIGHT_STICK_X];
  g_sbus_readout.controls.right_stick_y = g_sbus_readout.channels[SBUS_CH_RIGHT_STICK_Y];
  g_sbus_readout.controls.left_stick_y = g_sbus_readout.channels[SBUS_CH_LEFT_STICK_Y];
  g_sbus_readout.controls.left_stick_x = g_sbus_readout.channels[SBUS_CH_LEFT_STICK_X];
  g_sbus_readout.controls.switch_left_outer = g_sbus_readout.channels[SBUS_CH_SWITCH_LEFT_OUTER];
  g_sbus_readout.controls.switch_left_mid_1 = g_sbus_readout.channels[SBUS_CH_SWITCH_LEFT_MID_1];
  g_sbus_readout.controls.switch_left_mid_2 = g_sbus_readout.channels[SBUS_CH_SWITCH_LEFT_MID_2];
  g_sbus_readout.controls.switch_right_outer = g_sbus_readout.channels[SBUS_CH_SWITCH_RIGHT_OUTER];
  g_sbus_readout.controls.wheel_left = g_sbus_readout.channels[SBUS_CH_WHEEL_LEFT];
  g_sbus_readout.controls.wheel_right = g_sbus_readout.channels[SBUS_CH_WHEEL_RIGHT];
}

static void SbusDriver_DecodeFrame(void)
{
  const volatile uint8_t *f = s_sbus_frame_buf;
  uint8_t flags;

  if (f[0] != SBUS_START_BYTE)
  {
    g_sbus_diag.frame_bad_count++;
    g_sbus_diag.last_error = 7U;
    return;
  }

  g_sbus_readout.channels[0] = (uint16_t)((((uint16_t)f[1]) | (((uint16_t)f[2]) << 8)) & 0x07FFU);
  g_sbus_readout.channels[1] = (uint16_t)((((uint16_t)(f[2] >> 3)) | (((uint16_t)f[3]) << 5)) & 0x07FFU);
  g_sbus_readout.channels[2] = (uint16_t)((((uint16_t)(f[3] >> 6)) | (((uint16_t)f[4]) << 2) | (((uint16_t)f[5]) << 10)) & 0x07FFU);
  g_sbus_readout.channels[3] = (uint16_t)((((uint16_t)(f[5] >> 1)) | (((uint16_t)f[6]) << 7)) & 0x07FFU);
  g_sbus_readout.channels[4] = (uint16_t)((((uint16_t)(f[6] >> 4)) | (((uint16_t)f[7]) << 4)) & 0x07FFU);
  g_sbus_readout.channels[5] = (uint16_t)((((uint16_t)(f[7] >> 7)) | (((uint16_t)f[8]) << 1) | (((uint16_t)f[9]) << 9)) & 0x07FFU);
  g_sbus_readout.channels[6] = (uint16_t)((((uint16_t)(f[9] >> 2)) | (((uint16_t)f[10]) << 6)) & 0x07FFU);
  g_sbus_readout.channels[7] = (uint16_t)((((uint16_t)(f[10] >> 5)) | (((uint16_t)f[11]) << 3)) & 0x07FFU);
  g_sbus_readout.channels[8] = (uint16_t)((((uint16_t)f[12]) | (((uint16_t)f[13]) << 8)) & 0x07FFU);
  g_sbus_readout.channels[9] = (uint16_t)((((uint16_t)(f[13] >> 3)) | (((uint16_t)f[14]) << 5)) & 0x07FFU);
  g_sbus_readout.channels[10] = (uint16_t)((((uint16_t)(f[14] >> 6)) | (((uint16_t)f[15]) << 2) | (((uint16_t)f[16]) << 10)) & 0x07FFU);
  g_sbus_readout.channels[11] = (uint16_t)((((uint16_t)(f[16] >> 1)) | (((uint16_t)f[17]) << 7)) & 0x07FFU);
  g_sbus_readout.channels[12] = (uint16_t)((((uint16_t)(f[17] >> 4)) | (((uint16_t)f[18]) << 4)) & 0x07FFU);
  g_sbus_readout.channels[13] = (uint16_t)((((uint16_t)(f[18] >> 7)) | (((uint16_t)f[19]) << 1) | (((uint16_t)f[20]) << 9)) & 0x07FFU);
  g_sbus_readout.channels[14] = (uint16_t)((((uint16_t)(f[20] >> 2)) | (((uint16_t)f[21]) << 6)) & 0x07FFU);
  g_sbus_readout.channels[15] = (uint16_t)((((uint16_t)(f[21] >> 5)) | (((uint16_t)f[22]) << 3)) & 0x07FFU);

  flags = f[SBUS_FLAGS_INDEX];
  g_sbus_readout.channels[16] = ((flags & 0x01U) != 0U) ? SBUS_CHANNEL_VALUE_MAX : 0U;
  g_sbus_readout.channels[17] = ((flags & 0x02U) != 0U) ? SBUS_CHANNEL_VALUE_MAX : 0U;
  g_sbus_readout.frame_lost = ((flags & 0x04U) != 0U) ? 1U : 0U;
  g_sbus_readout.failsafe = ((flags & 0x08U) != 0U) ? 1U : 0U;
  SbusDriver_UpdateControlInputs();

  g_sbus_readout.frame_update_count++;
  g_sbus_readout.last_update_tick = HAL_GetTick();

  SbusDriver_CopyFrameToReadout();

  g_sbus_diag.frame_ok_count++;
  g_sbus_diag.last_error = 0U;
}

static void SbusDriver_ProcessByte(uint8_t b)
{
  if (s_sbus_frame_index == 0U)
  {
    if (b != SBUS_START_BYTE)
    {
      g_sbus_diag.start_byte_miss_count++;
      return;
    }

    s_sbus_frame_buf[0] = b;
    s_sbus_frame_index = 1U;
    return;
  }

  if (s_sbus_frame_index >= SBUS_FRAME_LEN)
  {
    g_sbus_diag.frame_bad_count++;
    g_sbus_diag.last_error = 8U;
    s_sbus_frame_index = 0U;
    return;
  }

  s_sbus_frame_buf[s_sbus_frame_index] = b;
  s_sbus_frame_index++;

  if (s_sbus_frame_index == SBUS_FRAME_LEN)
  {
    SbusDriver_DecodeFrame();
    s_sbus_frame_index = 0U;
  }
}

static void SbusDriver_RearmReceive(uint32_t fail_error_code)
{
  HAL_StatusTypeDef status;

  if (s_sbus_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(s_sbus_uart, (uint8_t *)&s_sbus_rx_byte, 1U);
  if (status == HAL_OK)
  {
    g_sbus_diag.rx_rearm_ok_count++;
    return;
  }

  if (status == HAL_BUSY)
  {
    g_sbus_diag.rx_rearm_busy_count++;
    s_sbus_uart->RxState = HAL_UART_STATE_READY;
    s_sbus_uart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UART_Receive_IT(s_sbus_uart, (uint8_t *)&s_sbus_rx_byte, 1U);
    if (status == HAL_OK)
    {
      g_sbus_diag.rx_rearm_ok_count++;
      return;
    }
  }

  g_sbus_diag.rx_rearm_fail_count++;
  g_sbus_diag.last_error = fail_error_code;
}

void SbusDriver_Init(UART_HandleTypeDef *huart)
{
  s_sbus_uart = huart;
  s_sbus_rx_byte = 0U;
  SbusDriver_ResetParser();

  (void)memset((void *)&g_sbus_state, 0, sizeof(g_sbus_state));

  if (s_sbus_uart == NULL)
  {
    g_sbus_diag.last_error = 1U;
    return;
  }

  SbusDriver_RearmReceive(2U);
  if (g_sbus_diag.last_error == 0U)
  {
    g_sbus_diag.initialized = 1U;
  }
}

void SbusDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_sbus_uart == NULL)
  {
    return;
  }

  if (huart->Instance != s_sbus_uart->Instance)
  {
    return;
  }

  g_sbus_diag.rx_irq_count++;
  g_sbus_diag.last_irq_tick = HAL_GetTick();

  SbusDriver_ProcessByte(s_sbus_rx_byte);
  SbusDriver_RearmReceive(3U);
}

void SbusDriver_OnError(UART_HandleTypeDef *huart)
{
  uint32_t isr;
  uint8_t has_salvaged_byte = 0U;
  uint8_t salvaged_byte = 0U;

  if (huart == NULL || s_sbus_uart == NULL)
  {
    return;
  }

  if (huart->Instance != s_sbus_uart->Instance)
  {
    return;
  }

  g_sbus_diag.uart_error_code = huart->ErrorCode;
  g_sbus_diag.last_error = 4U;

  /* Some SBUS links still place a data byte into RDR when FE/NE is flagged.
   * Salvage that byte first to avoid stalling frame assembly at error bursts. */
  isr = huart->Instance->ISR;
  if ((isr & USART_ISR_RXNE_RXFNE) != 0U)
  {
    salvaged_byte = (uint8_t)(huart->Instance->RDR & 0xFFU);
    has_salvaged_byte = 1U;
  }
  else
  {
    (void)(huart->Instance->RDR);
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  huart->RxState = HAL_UART_STATE_READY;

  if (has_salvaged_byte != 0U)
  {
    g_sbus_diag.rx_irq_count++;
    g_sbus_diag.last_irq_tick = HAL_GetTick();
    SbusDriver_ProcessByte(salvaged_byte);
  }

  SbusDriver_RearmReceive(5U);
}

uint32_t SbusDriver_IsFrameFresh(uint32_t now_ms, uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    timeout_ms = SBUS_FRAME_FRESH_DEFAULT_MS;
  }

  if (g_sbus_readout.frame_update_count == 0U)
  {
    return 0U;
  }

  return ((uint32_t)(now_ms - g_sbus_readout.last_update_tick) <= timeout_ms) ? 1U : 0U;
}

uint32_t SbusDriver_GetChannels(uint16_t *dst, uint32_t max_count,
                                uint32_t *frame_lost, uint32_t *failsafe,
                                uint32_t *last_update_tick)
{
  uint32_t i;
  uint32_t count;
  uint32_t primask;

  if (dst == NULL || max_count == 0U)
  {
    return 0U;
  }

  count = (max_count < SBUS_CHANNEL_COUNT) ? max_count : SBUS_CHANNEL_COUNT;

  primask = __get_PRIMASK();
  __disable_irq();

  for (i = 0U; i < count; i++)
  {
    dst[i] = g_sbus_readout.channels[i];
  }

  if (frame_lost != NULL)
  {
    *frame_lost = g_sbus_readout.frame_lost;
  }

  if (failsafe != NULL)
  {
    *failsafe = g_sbus_readout.failsafe;
  }

  if (last_update_tick != NULL)
  {
    *last_update_tick = g_sbus_readout.last_update_tick;
  }

  __set_PRIMASK((uint32_t)primask);

  return count;
}
