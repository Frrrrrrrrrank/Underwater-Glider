#include "pressure_sensor_driver.h"

#include <string.h>

#define PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING 0x03U
#define PRESSURE_SENSOR_MODBUS_REQUEST_LEN 8U
#define PRESSURE_SENSOR_MODBUS_MAX_RESPONSE_LEN 16U
#define PRESSURE_SENSOR_MODBUS_VALUE_BYTE_COUNT 4U
#define PRESSURE_SENSOR_MODBUS_VALUE_REG_COUNT 0x0002U
#define PRESSURE_SENSOR_MODBUS_PRESSURE_START_REG 0x0002U
#define PRESSURE_SENSOR_MODBUS_TEMPERATURE_START_REG 0x0008U

#define PRESSURE_SENSOR_REQ_PRESSURE 0U
#define PRESSURE_SENSOR_REQ_TEMPERATURE 1U

static UART_HandleTypeDef *s_pressure_sensor_uart = NULL;
static volatile uint8_t s_pressure_sensor_rx_ring[PRESSURE_SENSOR_RX_RING_SIZE];
static volatile uint16_t s_pressure_sensor_rx_head = 0U;
static volatile uint16_t s_pressure_sensor_rx_tail = 0U;
static volatile uint8_t s_pressure_sensor_rx_byte = 0U;

static volatile uint8_t s_modbus_request[PRESSURE_SENSOR_MODBUS_REQUEST_LEN];
static volatile uint8_t s_response_frame[PRESSURE_SENSOR_MODBUS_MAX_RESPONSE_LEN];
static volatile uint16_t s_response_index = 0U;
static volatile uint16_t s_response_expected_len = 0U;

static volatile uint8_t s_tx_in_flight = 0U;
static volatile uint8_t s_waiting_response = 0U;
static volatile uint32_t s_last_request_tick = 0U;
static volatile uint32_t s_response_start_tick = 0U;

static volatile uint8_t s_next_request_kind = PRESSURE_SENSOR_REQ_PRESSURE;
static volatile uint8_t s_active_request_kind = PRESSURE_SENSOR_REQ_PRESSURE;

volatile PressureSensorState g_pressure_sensor_state = {0};

static uint16_t PressureSensorDriver_ModbusCrc16(const uint8_t *data, uint16_t len)
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

static void PressureSensorDriver_UpdateRingAvailable(void)
{
  if (s_pressure_sensor_rx_head >= s_pressure_sensor_rx_tail)
  {
    g_pressure_sensor_readout.ring_available_bytes = (uint32_t)(s_pressure_sensor_rx_head - s_pressure_sensor_rx_tail);
  }
  else
  {
    g_pressure_sensor_readout.ring_available_bytes =
        (uint32_t)(PRESSURE_SENSOR_RX_RING_SIZE - s_pressure_sensor_rx_tail + s_pressure_sensor_rx_head);
  }
}

static void PressureSensorDriver_ResetResponseParser(void)
{
  s_response_index = 0U;
  s_response_expected_len = 0U;
}

static void PressureSensorDriver_StopWaitingResponse(void)
{
  s_waiting_response = 0U;
  g_pressure_sensor_diag.response_waiting = 0U;
  PressureSensorDriver_ResetResponseParser();
}

static void PressureSensorDriver_CopyTxFrameToState(void)
{
  uint32_t i;

  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_tx_data); i++)
  {
    if (i < PRESSURE_SENSOR_MODBUS_REQUEST_LEN)
    {
      g_pressure_sensor_readout.last_tx_data[i] = s_modbus_request[i];
    }
    else
    {
      g_pressure_sensor_readout.last_tx_data[i] = 0U;
    }
  }
}

static void PressureSensorDriver_CopyResponseFrameToState(uint16_t len)
{
  uint32_t i;

  g_pressure_sensor_readout.last_response_len = (uint32_t)len;
  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_response_data); i++)
  {
    if (i < (uint32_t)len)
    {
      g_pressure_sensor_readout.last_response_data[i] = s_response_frame[i];
    }
    else
    {
      g_pressure_sensor_readout.last_response_data[i] = 0U;
    }
  }
}

static void PressureSensorDriver_BuildReadRequest(uint8_t addr, uint16_t start_reg)
{
  uint16_t crc;

  s_modbus_request[0] = addr;
  s_modbus_request[1] = PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING;
  s_modbus_request[2] = (uint8_t)((start_reg >> 8) & 0xFFU);
  s_modbus_request[3] = (uint8_t)(start_reg & 0xFFU);
  s_modbus_request[4] = (uint8_t)((PRESSURE_SENSOR_MODBUS_VALUE_REG_COUNT >> 8) & 0xFFU);
  s_modbus_request[5] = (uint8_t)(PRESSURE_SENSOR_MODBUS_VALUE_REG_COUNT & 0xFFU);

  crc = PressureSensorDriver_ModbusCrc16((const uint8_t *)s_modbus_request, 6U);
  s_modbus_request[6] = (uint8_t)(crc & 0xFFU);
  s_modbus_request[7] = (uint8_t)((crc >> 8) & 0xFFU);

  PressureSensorDriver_CopyTxFrameToState();
}

static uint8_t PressureSensorDriver_DecodeFloat(const volatile uint8_t *src, float *out_value)
{
  uint32_t bits;

  if (src == NULL || out_value == NULL)
  {
    return 0U;
  }

  bits = ((uint32_t)src[0] << 24) |
         ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) |
         ((uint32_t)src[3]);
  (void)memcpy(out_value, &bits, sizeof(float));

  return 1U;
}

static void PressureSensorDriver_CommitMeasurement(float value)
{
  if (s_active_request_kind == PRESSURE_SENSOR_REQ_PRESSURE)
  {
    g_pressure_sensor_readout.pressure_bar = value;
    g_pressure_sensor_readout.pressure_valid = 1U;
  }
  else
  {
    g_pressure_sensor_readout.temperature_c = value;
    g_pressure_sensor_readout.temperature_valid = 1U;
  }

  g_pressure_sensor_readout.last_update_tick = HAL_GetTick();
}

static void PressureSensorDriver_HandleResponseFrame(uint16_t len)
{
  uint16_t recv_crc;
  uint16_t calc_crc;
  uint8_t func;
  float value = 0.0f;

  if (len < 5U || len > PRESSURE_SENSOR_MODBUS_MAX_RESPONSE_LEN)
  {
    g_pressure_sensor_diag.last_error = 10U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  PressureSensorDriver_CopyResponseFrameToState(len);

  recv_crc = (uint16_t)(((uint16_t)s_response_frame[len - 1U] << 8) | s_response_frame[len - 2U]);
  calc_crc = PressureSensorDriver_ModbusCrc16((const uint8_t *)s_response_frame, (uint16_t)(len - 2U));
  g_pressure_sensor_diag.last_response_tick = HAL_GetTick();

  if (recv_crc != calc_crc)
  {
    g_pressure_sensor_diag.crc_error_count++;
    g_pressure_sensor_diag.last_error = 8U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  func = s_response_frame[1];
  if (func == (uint8_t)(PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING | 0x80U))
  {
    g_pressure_sensor_diag.exception_count++;
    g_pressure_sensor_diag.last_exception_code = s_response_frame[2];
    g_pressure_sensor_diag.last_error = 9U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  if (func != PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING || s_response_frame[2] != PRESSURE_SENSOR_MODBUS_VALUE_BYTE_COUNT)
  {
    g_pressure_sensor_diag.last_error = 10U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  if (PressureSensorDriver_DecodeFloat(&s_response_frame[3], &value) == 0U)
  {
    g_pressure_sensor_diag.last_error = 10U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  PressureSensorDriver_CommitMeasurement(value);
  g_pressure_sensor_diag.response_ok_count++;
  g_pressure_sensor_diag.last_error = 0U;
  PressureSensorDriver_StopWaitingResponse();
}

static void PressureSensorDriver_ProcessResponseByte(uint8_t data)
{
  uint8_t configured_addr;

  if (s_waiting_response == 0U)
  {
    return;
  }

  configured_addr = (uint8_t)g_pressure_sensor_diag.slave_address;
  if (configured_addr == 0U)
  {
    configured_addr = PRESSURE_SENSOR_MODBUS_DEFAULT_ADDR;
    g_pressure_sensor_diag.slave_address = configured_addr;
  }

  if (s_response_index == 0U)
  {
    if (data != configured_addr)
    {
      return;
    }

    s_response_frame[s_response_index] = data;
    s_response_index = 1U;
    return;
  }

  if (s_response_index >= PRESSURE_SENSOR_MODBUS_MAX_RESPONSE_LEN)
  {
    g_pressure_sensor_diag.last_error = 11U;
    PressureSensorDriver_StopWaitingResponse();
    return;
  }

  s_response_frame[s_response_index] = data;
  s_response_index++;

  if (s_response_index == 2U)
  {
    uint8_t func = s_response_frame[1];

    if (func == PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING)
    {
      return;
    }

    if (func == (uint8_t)(PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING | 0x80U))
    {
      s_response_expected_len = 5U;
      return;
    }

    s_response_index = 0U;
    return;
  }

  if (s_response_index == 3U && s_response_frame[1] == PRESSURE_SENSOR_MODBUS_FUNC_READ_HOLDING)
  {
    uint8_t byte_count = s_response_frame[2];

    s_response_expected_len = (uint16_t)(3U + byte_count + 2U);
    if (byte_count != PRESSURE_SENSOR_MODBUS_VALUE_BYTE_COUNT ||
        s_response_expected_len > PRESSURE_SENSOR_MODBUS_MAX_RESPONSE_LEN)
    {
      g_pressure_sensor_diag.last_error = 12U;
      PressureSensorDriver_StopWaitingResponse();
    }
    return;
  }

  if (s_response_expected_len != 0U && s_response_index >= s_response_expected_len)
  {
    PressureSensorDriver_HandleResponseFrame(s_response_expected_len);
  }
}

static void PressureSensorDriver_RingPush(uint8_t data)
{
  uint16_t next = (uint16_t)((s_pressure_sensor_rx_head + 1U) % PRESSURE_SENSOR_RX_RING_SIZE);

  if (next == s_pressure_sensor_rx_tail)
  {
    g_pressure_sensor_diag.rx_overflow_count++;
    s_pressure_sensor_rx_tail = (uint16_t)((s_pressure_sensor_rx_tail + 1U) % PRESSURE_SENSOR_RX_RING_SIZE);
  }

  s_pressure_sensor_rx_ring[s_pressure_sensor_rx_head] = data;
  s_pressure_sensor_rx_head = next;
  g_pressure_sensor_readout.last_irq_byte = (uint32_t)data;
  PressureSensorDriver_UpdateRingAvailable();
}

void PressureSensorDriver_Init(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef status;
  uint32_t i;

  s_pressure_sensor_uart = huart;
  s_pressure_sensor_rx_head = 0U;
  s_pressure_sensor_rx_tail = 0U;
  s_pressure_sensor_rx_byte = 0U;

  s_tx_in_flight = 0U;
  s_waiting_response = 0U;
  s_last_request_tick = 0U;
  s_response_start_tick = 0U;
  s_next_request_kind = PRESSURE_SENSOR_REQ_PRESSURE;
  s_active_request_kind = PRESSURE_SENSOR_REQ_PRESSURE;
  PressureSensorDriver_ResetResponseParser();

  g_pressure_sensor_diag.initialized = 0U;
  g_pressure_sensor_diag.slave_address = PRESSURE_SENSOR_MODBUS_DEFAULT_ADDR;
  g_pressure_sensor_diag.last_error = 0U;
  g_pressure_sensor_diag.uart_error_code = 0U;
  g_pressure_sensor_diag.tx_in_flight = 0U;
  g_pressure_sensor_diag.response_waiting = 0U;
  g_pressure_sensor_diag.rx_irq_count = 0U;
  g_pressure_sensor_diag.rx_overflow_count = 0U;
  g_pressure_sensor_diag.request_count = 0U;
  g_pressure_sensor_diag.response_ok_count = 0U;
  g_pressure_sensor_diag.timeout_count = 0U;
  g_pressure_sensor_diag.crc_error_count = 0U;
  g_pressure_sensor_diag.exception_count = 0U;
  g_pressure_sensor_diag.last_exception_code = 0U;
  g_pressure_sensor_diag.last_request_tick = 0U;
  g_pressure_sensor_diag.last_response_tick = 0U;

  g_pressure_sensor_readout.last_irq_byte = 0U;
  g_pressure_sensor_readout.ring_available_bytes = 0U;
  g_pressure_sensor_readout.pressure_valid = 0U;
  g_pressure_sensor_readout.pressure_bar = 0.0f;
  g_pressure_sensor_readout.temperature_valid = 0U;
  g_pressure_sensor_readout.temperature_c = 0.0f;
  g_pressure_sensor_readout.last_update_tick = 0U;
  g_pressure_sensor_readout.last_response_len = 0U;

  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_tx_data); i++)
  {
    g_pressure_sensor_readout.last_tx_data[i] = 0U;
  }
  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_response_data); i++)
  {
    g_pressure_sensor_readout.last_response_data[i] = 0U;
  }

  if (s_pressure_sensor_uart == NULL)
  {
    g_pressure_sensor_diag.last_error = 1U;
    return;
  }

  status = HAL_UART_Receive_IT(s_pressure_sensor_uart, (uint8_t *)&s_pressure_sensor_rx_byte, 1U);
  if (status == HAL_OK)
  {
    g_pressure_sensor_diag.initialized = 1U;
  }
  else
  {
    g_pressure_sensor_diag.last_error = 2U;
  }
}

void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef status;

  if (huart == NULL || s_pressure_sensor_uart == NULL)
  {
    return;
  }

  if (huart->Instance != s_pressure_sensor_uart->Instance)
  {
    return;
  }

  g_pressure_sensor_diag.rx_irq_count++;
  PressureSensorDriver_RingPush(s_pressure_sensor_rx_byte);
  PressureSensorDriver_ProcessResponseByte(s_pressure_sensor_rx_byte);

  status = HAL_UART_Receive_IT(s_pressure_sensor_uart, (uint8_t *)&s_pressure_sensor_rx_byte, 1U);
  if (status != HAL_OK && status != HAL_BUSY)
  {
    g_pressure_sensor_diag.last_error = 3U;
  }
}

void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_pressure_sensor_uart == NULL)
  {
    return;
  }

  if (huart->Instance != s_pressure_sensor_uart->Instance)
  {
    return;
  }

  s_tx_in_flight = 0U;
  g_pressure_sensor_diag.tx_in_flight = 0U;

  s_waiting_response = 1U;
  g_pressure_sensor_diag.response_waiting = 1U;
  s_response_start_tick = HAL_GetTick();
  PressureSensorDriver_ResetResponseParser();
}

void PressureSensorDriver_OnError(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef status;

  if (huart == NULL || s_pressure_sensor_uart == NULL)
  {
    return;
  }

  if (huart->Instance != s_pressure_sensor_uart->Instance)
  {
    return;
  }

  g_pressure_sensor_diag.uart_error_code = huart->ErrorCode;
  g_pressure_sensor_diag.last_error = 4U;

  s_tx_in_flight = 0U;
  g_pressure_sensor_diag.tx_in_flight = 0U;
  PressureSensorDriver_StopWaitingResponse();

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  (void)(huart->Instance->RDR);
  huart->RxState = HAL_UART_STATE_READY;

  status = HAL_UART_Receive_IT(s_pressure_sensor_uart, (uint8_t *)&s_pressure_sensor_rx_byte, 1U);
  if (status != HAL_OK && status != HAL_BUSY)
  {
    g_pressure_sensor_diag.last_error = 4U;
  }
}

void PressureSensorDriver_Poll(uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  uint8_t addr;
  uint16_t start_reg;

  if (s_pressure_sensor_uart == NULL || g_pressure_sensor_diag.initialized == 0U)
  {
    return;
  }

  if (s_tx_in_flight != 0U)
  {
    if ((uint32_t)(now_ms - s_last_request_tick) >= PRESSURE_SENSOR_MODBUS_RESPONSE_TIMEOUT_MS)
    {
      s_tx_in_flight = 0U;
      g_pressure_sensor_diag.tx_in_flight = 0U;
      g_pressure_sensor_diag.timeout_count++;
      g_pressure_sensor_diag.last_error = 13U;
    }
    return;
  }

  if (s_waiting_response != 0U)
  {
    if ((uint32_t)(now_ms - s_response_start_tick) >= PRESSURE_SENSOR_MODBUS_RESPONSE_TIMEOUT_MS)
    {
      g_pressure_sensor_diag.timeout_count++;
      g_pressure_sensor_diag.last_error = 14U;
      PressureSensorDriver_StopWaitingResponse();
    }
    return;
  }

  if ((uint32_t)(now_ms - s_last_request_tick) < PRESSURE_SENSOR_MODBUS_REQUEST_PERIOD_MS)
  {
    return;
  }

  addr = (uint8_t)g_pressure_sensor_diag.slave_address;
  if (addr == 0U)
  {
    addr = PRESSURE_SENSOR_MODBUS_DEFAULT_ADDR;
    g_pressure_sensor_diag.slave_address = addr;
  }

  if (s_next_request_kind == PRESSURE_SENSOR_REQ_PRESSURE)
  {
    start_reg = PRESSURE_SENSOR_MODBUS_PRESSURE_START_REG;
  }
  else
  {
    start_reg = PRESSURE_SENSOR_MODBUS_TEMPERATURE_START_REG;
  }

  PressureSensorDriver_BuildReadRequest(addr, start_reg);
  g_pressure_sensor_diag.request_count++;

  status = HAL_UART_Transmit_IT(s_pressure_sensor_uart, (const uint8_t *)s_modbus_request, PRESSURE_SENSOR_MODBUS_REQUEST_LEN);
  if (status == HAL_OK)
  {
    s_tx_in_flight = 1U;
    g_pressure_sensor_diag.tx_in_flight = 1U;

    s_last_request_tick = now_ms;
    g_pressure_sensor_diag.last_request_tick = now_ms;

    s_active_request_kind = s_next_request_kind;
    if (s_next_request_kind == PRESSURE_SENSOR_REQ_PRESSURE)
    {
      s_next_request_kind = PRESSURE_SENSOR_REQ_TEMPERATURE;
    }
    else
    {
      s_next_request_kind = PRESSURE_SENSOR_REQ_PRESSURE;
    }
  }
  else if (status != HAL_BUSY)
  {
    g_pressure_sensor_diag.last_error = 15U;
  }
}

uint16_t PressureSensorDriver_Available(void)
{
  uint16_t head = s_pressure_sensor_rx_head;
  uint16_t tail = s_pressure_sensor_rx_tail;

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }

  return (uint16_t)(PRESSURE_SENSOR_RX_RING_SIZE - tail + head);
}

uint16_t PressureSensorDriver_Read(uint8_t *dst, uint16_t max_len)
{
  uint16_t count = 0U;

  if (dst == NULL || max_len == 0U)
  {
    return 0U;
  }

  while (count < max_len)
  {
    uint16_t tail = s_pressure_sensor_rx_tail;

    if (tail == s_pressure_sensor_rx_head)
    {
      break;
    }

    dst[count] = s_pressure_sensor_rx_ring[tail];
    s_pressure_sensor_rx_tail = (uint16_t)((tail + 1U) % PRESSURE_SENSOR_RX_RING_SIZE);
    count++;
  }

  PressureSensorDriver_UpdateRingAvailable();
  return count;
}

void PressureSensorDriver_ClearRxBuffer(void)
{
  s_pressure_sensor_rx_tail = s_pressure_sensor_rx_head;
  g_pressure_sensor_readout.ring_available_bytes = 0U;
}
