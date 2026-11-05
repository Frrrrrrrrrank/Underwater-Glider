#include "pressure_sensor_driver.h"

#include <string.h>

#if PRESSURE_SENSOR_TYPE == PRESSURE_SENSOR_TYPE_MODBUS_RS485

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
  const uint32_t now_ms = HAL_GetTick();

  if (s_active_request_kind == PRESSURE_SENSOR_REQ_PRESSURE)
  {
    g_pressure_sensor_readout.pressure_bar = value;
    g_pressure_sensor_readout.pressure_valid = 1U;
    g_pressure_sensor_readout.pressure_update_tick = now_ms;
    g_pressure_sensor_readout.pressure_update_seq++;
  }
  else
  {
    g_pressure_sensor_readout.temperature_c = value;
    g_pressure_sensor_readout.temperature_valid = 1U;
    g_pressure_sensor_readout.temperature_update_tick = now_ms;
    g_pressure_sensor_readout.temperature_update_seq++;
  }

  g_pressure_sensor_readout.last_update_tick = now_ms;
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
  g_pressure_sensor_diag.sensor_type = PRESSURE_SENSOR_TYPE_MODBUS_RS485;
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
  g_pressure_sensor_readout.pressure_update_tick = 0U;
  g_pressure_sensor_readout.pressure_update_seq = 0U;
  g_pressure_sensor_readout.temperature_valid = 0U;
  g_pressure_sensor_readout.temperature_c = 0.0f;
  g_pressure_sensor_readout.temperature_update_tick = 0U;
  g_pressure_sensor_readout.temperature_update_seq = 0U;
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

#elif PRESSURE_SENSOR_TYPE == PRESSURE_SENSOR_TYPE_KELLER_I2C

#define KELLER_I2C_MEASUREMENT_COMMAND      0xACU
#define KELLER_I2C_SCALING_FIRST_ADDRESS    0x13U
#define KELLER_I2C_SCALING_LAST_ADDRESS     0x16U
#define KELLER_I2C_MEMORY_WAIT_MS            1U
#define KELLER_I2C_CONVERSION_WAIT_MS        8U
#define KELLER_I2C_TRANSACTION_TIMEOUT_MS    3U
#define KELLER_I2C_REQUEST_PERIOD_MS       500U
#define KELLER_STATUS_BUSY_MASK             0x20U
#define KELLER_STATUS_MEMORY_ERROR_MASK     0x04U
#define KELLER_I2C_SCAN_FIRST_ADDR          0x08U
#define KELLER_I2C_SCAN_LAST_ADDR           0x77U

typedef enum
{
  KELLER_STATE_DIRECT_READ_TEST = 0,
  KELLER_STATE_ADDRESS_PROBE,
  KELLER_STATE_SCALING_WRITE,
  KELLER_STATE_SCALING_WAIT,
  KELLER_STATE_SCALING_READ,
  KELLER_STATE_MEASUREMENT_IDLE,
  KELLER_STATE_MEASUREMENT_WAIT,
  KELLER_STATE_MEASUREMENT_READ
} KellerDriverState;

extern I2C_HandleTypeDef hi2c2;

static I2C_HandleTypeDef *s_pressure_sensor_i2c = NULL;
static KellerDriverState s_keller_state = KELLER_STATE_SCALING_WRITE;
static uint8_t s_keller_address = PRESSURE_SENSOR_KELLER_I2C_ADDRESS;
static uint8_t s_scan_address = KELLER_I2C_SCAN_FIRST_ADDR;
static uint8_t s_scaling_address = KELLER_I2C_SCALING_FIRST_ADDRESS;
static uint16_t s_scaling_words[4];
static uint32_t s_state_tick = 0U;
static uint32_t s_last_measurement_request_tick = 0U;

volatile PressureSensorState g_pressure_sensor_state = {0};

static void PressureSensorDriver_ClearFrameBuffers(void)
{
  uint32_t i;

  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_tx_data); i++)
  {
    g_pressure_sensor_readout.last_tx_data[i] = 0U;
  }
  for (i = 0U; i < (uint32_t)sizeof(g_pressure_sensor_readout.last_response_data); i++)
  {
    g_pressure_sensor_readout.last_response_data[i] = 0U;
  }
}

static uint16_t PressureSensorDriver_KellerHalAddress(void)
{
  return (uint16_t)(s_keller_address << 1U);
}

static void PressureSensorDriver_RecordI2cError(uint32_t error)
{
  g_pressure_sensor_diag.last_error = error;
  if (s_pressure_sensor_i2c != NULL)
  {
    g_pressure_sensor_diag.i2c_error_code = HAL_I2C_GetError(s_pressure_sensor_i2c);
  }
}

static HAL_StatusTypeDef PressureSensorDriver_KellerWriteCommand(uint8_t command)
{
  HAL_StatusTypeDef status;

  g_pressure_sensor_readout.last_tx_data[0] = command;
  g_pressure_sensor_diag.request_count++;
  g_pressure_sensor_diag.last_request_tick = HAL_GetTick();
  status = HAL_I2C_Master_Transmit(s_pressure_sensor_i2c,
                                  PressureSensorDriver_KellerHalAddress(),
                                  &command,
                                  1U,
                                  KELLER_I2C_TRANSACTION_TIMEOUT_MS);
  g_pressure_sensor_diag.last_hal_status = (uint32_t)status;
  return status;
}

static HAL_StatusTypeDef PressureSensorDriver_KellerRead(uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;
  uint16_t i;

  status = HAL_I2C_Master_Receive(s_pressure_sensor_i2c,
                                 PressureSensorDriver_KellerHalAddress(),
                                 data,
                                 len,
                                 KELLER_I2C_TRANSACTION_TIMEOUT_MS);
  g_pressure_sensor_diag.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    return status;
  }

  g_pressure_sensor_readout.last_response_len = len;
  for (i = 0U; i < (uint16_t)sizeof(g_pressure_sensor_readout.last_response_data); i++)
  {
    g_pressure_sensor_readout.last_response_data[i] = (i < len) ? data[i] : 0U;
  }
  g_pressure_sensor_diag.last_response_tick = HAL_GetTick();
  return HAL_OK;
}

static float PressureSensorDriver_WordsToFloat(uint16_t high_word, uint16_t low_word)
{
  uint32_t bits = ((uint32_t)high_word << 16U) | (uint32_t)low_word;
  float value;

  (void)memcpy(&value, &bits, sizeof(value));
  return value;
}

static void PressureSensorDriver_CommitScaling(void)
{
  float pressure_min = PressureSensorDriver_WordsToFloat(s_scaling_words[0], s_scaling_words[1]);
  float pressure_max = PressureSensorDriver_WordsToFloat(s_scaling_words[2], s_scaling_words[3]);

  if (pressure_max > pressure_min)
  {
    g_pressure_sensor_readout.pressure_min_bar = pressure_min;
    g_pressure_sensor_readout.pressure_max_bar = pressure_max;
    g_pressure_sensor_diag.scaling_valid = 1U;
    g_pressure_sensor_diag.last_error = 0U;
  }
  else
  {
    g_pressure_sensor_diag.scaling_valid = 0U;
    g_pressure_sensor_diag.last_error = 23U;
  }
}

static void PressureSensorDriver_CommitMeasurement(const uint8_t data[5], uint32_t now_ms)
{
  uint16_t pressure_raw = (uint16_t)(((uint16_t)data[1] << 8U) | data[2]);
  uint16_t temperature_raw = (uint16_t)(((uint16_t)data[3] << 8U) | data[4]);
  int32_t temperature_code = (int32_t)(temperature_raw >> 4U);
  float pressure_span;

  g_pressure_sensor_diag.status_byte = data[0];
  g_pressure_sensor_diag.pressure_raw = pressure_raw;
  g_pressure_sensor_diag.temperature_raw = temperature_raw;

  if ((data[0] & KELLER_STATUS_BUSY_MASK) != 0U)
  {
    g_pressure_sensor_diag.last_error = 24U;
    return;
  }

  if ((data[0] & KELLER_STATUS_MEMORY_ERROR_MASK) != 0U)
  {
    g_pressure_sensor_diag.last_error = 25U;
  }

  if (g_pressure_sensor_diag.scaling_valid != 0U)
  {
    pressure_span = g_pressure_sensor_readout.pressure_max_bar -
                    g_pressure_sensor_readout.pressure_min_bar;
    g_pressure_sensor_readout.pressure_bar =
        (((float)pressure_raw - 16384.0f) * pressure_span / 32768.0f) +
        g_pressure_sensor_readout.pressure_min_bar;
    g_pressure_sensor_readout.pressure_valid = 1U;
    g_pressure_sensor_readout.pressure_update_tick = now_ms;
    g_pressure_sensor_readout.pressure_update_seq++;
  }

  g_pressure_sensor_readout.temperature_c =
      ((float)(temperature_code - 24) * 0.05f) - 50.0f;
  g_pressure_sensor_readout.temperature_valid = 1U;
  g_pressure_sensor_readout.temperature_update_tick = now_ms;
  g_pressure_sensor_readout.temperature_update_seq++;
  g_pressure_sensor_readout.last_update_tick = now_ms;
  g_pressure_sensor_diag.response_ok_count++;
  if ((data[0] & KELLER_STATUS_MEMORY_ERROR_MASK) == 0U)
  {
    g_pressure_sensor_diag.last_error = 0U;
  }
}

void PressureSensorDriver_Init(UART_HandleTypeDef *huart)
{
  (void)huart;
  (void)memset((void *)&g_pressure_sensor_state, 0, sizeof(g_pressure_sensor_state));
  (void)memset(s_scaling_words, 0, sizeof(s_scaling_words));

  s_pressure_sensor_i2c = &hi2c2;
  s_keller_state = (PRESSURE_SENSOR_KELLER_SCAN_ADDRESS != 0U) ?
      KELLER_STATE_ADDRESS_PROBE : KELLER_STATE_MEASUREMENT_IDLE;
  s_keller_address = PRESSURE_SENSOR_KELLER_I2C_ADDRESS;
  s_scan_address = KELLER_I2C_SCAN_FIRST_ADDR;
  s_scaling_address = KELLER_I2C_SCALING_FIRST_ADDRESS;
  s_state_tick = 0U;
  s_last_measurement_request_tick = 0U;

  g_pressure_sensor_diag.sensor_type = PRESSURE_SENSOR_TYPE_KELLER_I2C;
  g_pressure_sensor_diag.slave_address = s_keller_address;
  g_pressure_sensor_diag.scaling_valid = 1U;
  g_pressure_sensor_diag.initialized = (s_pressure_sensor_i2c != NULL) ? 1U : 0U;
  g_pressure_sensor_readout.pressure_min_bar = PRESSURE_SENSOR_KELLER_PRESSURE_MIN_BAR;
  g_pressure_sensor_readout.pressure_max_bar = PRESSURE_SENSOR_KELLER_PRESSURE_MAX_BAR;
  PressureSensorDriver_ClearFrameBuffers();
}

void PressureSensorDriver_Poll(uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  uint8_t data[5];

  if (g_pressure_sensor_diag.initialized == 0U || s_pressure_sensor_i2c == NULL)
  {
    return;
  }

  switch (s_keller_state)
  {
    case KELLER_STATE_ADDRESS_PROBE:
      s_keller_address = s_scan_address;
      g_pressure_sensor_diag.probe_address = s_scan_address;
      g_pressure_sensor_diag.probe_count++;
      g_pressure_sensor_readout.last_tx_data[0] = (uint8_t)(s_scan_address << 1U);
      g_pressure_sensor_readout.last_tx_data[1] = 0U;
      status = HAL_I2C_IsDeviceReady(s_pressure_sensor_i2c,
                                     PressureSensorDriver_KellerHalAddress(),
                                     1U,
                                     KELLER_I2C_TRANSACTION_TIMEOUT_MS);
      g_pressure_sensor_diag.last_hal_status = (uint32_t)status;
      if (status == HAL_OK)
      {
        g_pressure_sensor_diag.address_ack_count++;
        g_pressure_sensor_diag.slave_address = s_keller_address;
        g_pressure_sensor_diag.i2c_error_code = 0U;
        g_pressure_sensor_diag.last_error = 0U;
        s_scaling_address = KELLER_I2C_SCALING_FIRST_ADDRESS;
        s_keller_state = KELLER_STATE_SCALING_WRITE;
      }
      else if (s_scan_address < KELLER_I2C_SCAN_LAST_ADDR)
      {
        s_scan_address++;
        PressureSensorDriver_RecordI2cError(26U);
      }
      else
      {
        s_scan_address = KELLER_I2C_SCAN_FIRST_ADDR;
        g_pressure_sensor_diag.slave_address = 0U;
        PressureSensorDriver_RecordI2cError(27U);
      }
      break;

    case KELLER_STATE_SCALING_WRITE:
      g_pressure_sensor_diag.probe_address = s_keller_address;
      g_pressure_sensor_diag.probe_count++;
      status = PressureSensorDriver_KellerWriteCommand(s_scaling_address);
      if (status == HAL_OK)
      {
        g_pressure_sensor_diag.address_ack_count++;
        g_pressure_sensor_diag.i2c_error_code = 0U;
        s_state_tick = now_ms;
        s_keller_state = KELLER_STATE_SCALING_WAIT;
      }
      else
      {
        PressureSensorDriver_RecordI2cError(20U);
      }
      break;

    case KELLER_STATE_SCALING_WAIT:
      if ((uint32_t)(now_ms - s_state_tick) >= KELLER_I2C_MEMORY_WAIT_MS)
      {
        s_keller_state = KELLER_STATE_SCALING_READ;
      }
      break;

    case KELLER_STATE_SCALING_READ:
      status = PressureSensorDriver_KellerRead(data, 3U);
      if (status != HAL_OK)
      {
        PressureSensorDriver_RecordI2cError(21U);
        s_keller_state = KELLER_STATE_SCALING_WRITE;
        break;
      }

      g_pressure_sensor_diag.status_byte = data[0];
      s_scaling_words[s_scaling_address - KELLER_I2C_SCALING_FIRST_ADDRESS] =
          (uint16_t)(((uint16_t)data[1] << 8U) | data[2]);

      if (s_scaling_address < KELLER_I2C_SCALING_LAST_ADDRESS)
      {
        s_scaling_address++;
        s_keller_state = KELLER_STATE_SCALING_WRITE;
      }
      else
      {
        PressureSensorDriver_CommitScaling();
        s_last_measurement_request_tick = now_ms;
        s_keller_state = KELLER_STATE_MEASUREMENT_IDLE;
      }
      break;

    case KELLER_STATE_MEASUREMENT_IDLE:
      if ((uint32_t)(now_ms - s_last_measurement_request_tick) < KELLER_I2C_REQUEST_PERIOD_MS)
      {
        break;
      }

      status = PressureSensorDriver_KellerWriteCommand(KELLER_I2C_MEASUREMENT_COMMAND);
      if (status == HAL_OK)
      {
        g_pressure_sensor_diag.address_ack_count++;
        g_pressure_sensor_diag.i2c_error_code = 0U;
        s_last_measurement_request_tick = now_ms;
        s_state_tick = now_ms;
        s_keller_state = KELLER_STATE_MEASUREMENT_WAIT;
      }
      else
      {
        PressureSensorDriver_RecordI2cError(20U);
      }
      break;

    case KELLER_STATE_MEASUREMENT_WAIT:
      if ((uint32_t)(now_ms - s_state_tick) >= KELLER_I2C_CONVERSION_WAIT_MS)
      {
        s_keller_state = KELLER_STATE_MEASUREMENT_READ;
      }
      break;

    case KELLER_STATE_MEASUREMENT_READ:
      status = PressureSensorDriver_KellerRead(data, 5U);
      if (status == HAL_OK)
      {
        PressureSensorDriver_CommitMeasurement(data, now_ms);
      }
      else
      {
        g_pressure_sensor_diag.timeout_count++;
        PressureSensorDriver_RecordI2cError(22U);
      }
      s_keller_state = KELLER_STATE_MEASUREMENT_IDLE;
      break;

    default:
      s_keller_state = KELLER_STATE_MEASUREMENT_IDLE;
      break;
  }

  g_pressure_sensor_diag.driver_state = (uint32_t)s_keller_state;
}

void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void PressureSensorDriver_OnError(UART_HandleTypeDef *huart)
{
  (void)huart;
}

uint16_t PressureSensorDriver_Available(void)
{
  return 0U;
}

uint16_t PressureSensorDriver_Read(uint8_t *dst, uint16_t max_len)
{
  (void)dst;
  (void)max_len;
  return 0U;
}

void PressureSensorDriver_ClearRxBuffer(void)
{
}


#elif PRESSURE_SENSOR_TYPE == PRESSURE_SENSOR_TYPE_MS5837_30BA

#include "FreeRTOS.h"
#include "task.h"

/* TE MS5837-30BA datasheet Rev C6 (02/2025), commands and compensation:
 * https://www.mouser.com/datasheet/2/418/10/ENG_DS_MS5837_30BA_C6-1955841.pdf
 * Keep 64-bit signed intermediates: D1*SENS exceeds 32 bits.
 * No HAL_Delay or conversion spin waits; only short bounded I2C transfers. */
#define MS5837_RESET 0x1EU
#define MS5837_PROM_BASE 0xA0U
#define MS5837_D1_OSR8192 0x4AU
#define MS5837_D2_OSR8192 0x5AU
#define MS5837_ADC_READ 0x00U

extern I2C_HandleTypeDef hi2c2;
volatile PressureSensorState g_pressure_sensor_state = {0};
static uint16_t s_ms_prom[8];
static uint8_t s_ms_prom_index;
static uint32_t s_ms_state;
static uint32_t s_ms_state_tick;
static uint32_t s_ms_d1;
static uint32_t s_ms_d1_tick;
static uint8_t s_ms_started;

static void PressureSensorDriver_MsState(uint32_t state)
{
  s_ms_state = state;
  s_ms_state_tick = HAL_GetTick();
  g_pressure_sensor_diag.driver_state = state;
}

static void PressureSensorDriver_MsFail(uint32_t error)
{
  g_pressure_sensor_diag.last_error = error;
  g_pressure_sensor_diag.initialized = 0U;
  g_pressure_sensor_diag.scaling_valid = 0U;
  g_pressure_sensor_diag.response_waiting = 0U;
  g_pressure_sensor_readout.pressure_valid = 0U;
  g_pressure_sensor_readout.temperature_valid = 0U;
  PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_RETRY);
}

static HAL_StatusTypeDef PressureSensorDriver_MsTransfer(uint8_t *data, uint16_t len,
                                                       uint8_t read)
{
  HAL_StatusTypeDef status;
  const uint16_t address = (uint16_t)(PRESSURE_SENSOR_MS5837_I2C_ADDRESS << 1U);

  /* Avoid the H7 HAL's separate 25 ms BUSY wait on a stuck bus. No bus reset
   * here: a future second I2C2 client must not have its transaction aborted. */
  if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY ||
      __HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET)
  {
    status = HAL_BUSY;
  }
  else if (read != 0U)
  {
    status = HAL_I2C_Master_Receive(&hi2c2, address, data, len,
                                  PRESSURE_SENSOR_MS5837_I2C_TIMEOUT_MS);
  }
  else
  {
    g_pressure_sensor_readout.last_tx_data[0] = data[0];
    status = HAL_I2C_Master_Transmit(&hi2c2, address, data, len,
                                   PRESSURE_SENSOR_MS5837_I2C_TIMEOUT_MS);
  }
  g_pressure_sensor_diag.last_hal_status = (uint32_t)status;
  g_pressure_sensor_diag.i2c_error_code = HAL_I2C_GetError(&hi2c2);
  if (status != HAL_OK)
  {
    g_pressure_sensor_diag.i2c_fail_count++;
    if (status == HAL_TIMEOUT)
    {
      g_pressure_sensor_diag.timeout_count++;
    }
    PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_I2C);
  }
  else
  {
    g_pressure_sensor_diag.address_ack_count++;
    if (read != 0U)
    {
      uint16_t i;
      g_pressure_sensor_readout.last_response_len = len;
      for (i = 0U; i < sizeof(g_pressure_sensor_readout.last_response_data); ++i)
      {
        g_pressure_sensor_readout.last_response_data[i] = (i < len) ? data[i] : 0U;
      }
    }
  }
  return status;
}

static HAL_StatusTypeDef PressureSensorDriver_MsCommand(uint8_t command)
{
  return PressureSensorDriver_MsTransfer(&command, 1U, 0U);
}

static uint8_t PressureSensorDriver_MsRead(uint8_t command, uint8_t *data, uint16_t len)
{
  if (PressureSensorDriver_MsCommand(command) != HAL_OK)
  {
    return 0U;
  }
  return (PressureSensorDriver_MsTransfer(data, len, 1U) == HAL_OK) ? 1U : 0U;
}

static uint8_t PressureSensorDriver_MsCrc4(const uint16_t prom[8])
{
  uint16_t remainder = 0U;
  uint8_t byte_index;
  for (byte_index = 0U; byte_index < 16U; ++byte_index)
  {
    uint16_t word = (byte_index < 14U) ? prom[byte_index / 2U] : 0U;
    uint8_t bit;
    if (byte_index < 2U) word &= 0x0FFFU;
    remainder ^= (byte_index & 1U) ? (word & 0xFFU) : (word >> 8U);
    for (bit = 0U; bit < 8U; ++bit)
    {
      remainder = (uint16_t)((remainder << 1U) ^
                            ((remainder & 0x8000U) ? 0x3000U : 0U));
    }
  }
  return (uint8_t)((remainder >> 12U) & 0x0FU);
}

static uint8_t PressureSensorDriver_MsAdc(uint32_t *value)
{
  uint8_t bytes[3];
  if (PressureSensorDriver_MsRead(MS5837_ADC_READ, bytes, 3U) == 0U) return 0U;
  *value = ((uint32_t)bytes[0] << 16U) | ((uint32_t)bytes[1] << 8U) | bytes[2];
  if (*value == 0U || *value == 0xFFFFFFU)
  {
    PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_ADC);
    return 0U;
  }
  return 1U;
}

static uint8_t PressureSensorDriver_MsCompensate(uint32_t d1, uint32_t d2,
                                               float *pressure, float *temperature)
{
  int64_t dt = (int64_t)d2 - (int64_t)s_ms_prom[5] * 256;
  int64_t temp = 2000 + dt * s_ms_prom[6] / 8388608;
  int64_t off = (int64_t)s_ms_prom[2] * 65536 + dt * s_ms_prom[4] / 128;
  int64_t sens = (int64_t)s_ms_prom[1] * 32768 + dt * s_ms_prom[3] / 256;
  int64_t delta = temp - 2000;
  int64_t ti, offi, sensi;
  int64_t p;
  if (temp < 2000)
  {
    ti = 3 * dt * dt / 8589934592LL;
    offi = 3 * delta * delta / 2;
    sensi = 5 * delta * delta / 8;
    if (temp < -1500)
    {
      delta = temp + 1500;
      offi += 7 * delta * delta;
      sensi += 4 * delta * delta;
    }
  }
  else
  {
    ti = 2 * dt * dt / 137438953472LL;
    offi = delta * delta / 16;
    sensi = 0;
  }
  temp -= ti;
  p = ((int64_t)d1 * (sens - sensi) / 2097152 - (off - offi)) / 8192;
  /* Datasheet P is in 0.1 mbar; 10000 such units = 1 bar. */
  *pressure = (float)p / 10000.0f;
  *temperature = (float)temp / 100.0f;
  /* Reject impossible outputs instead of clamping them into valid samples. */
  return (p >= 0 && p <= 300000 && temp >= -2000 && temp <= 8500) ? 1U : 0U;
}

static uint8_t PressureSensorDriver_MsStartD1(void)
{
  g_pressure_sensor_diag.request_count++;
  g_pressure_sensor_diag.last_request_tick = HAL_GetTick();
  if (PressureSensorDriver_MsCommand(MS5837_D1_OSR8192) != HAL_OK) return 0U;
  s_ms_d1_tick = HAL_GetTick();
  g_pressure_sensor_diag.response_waiting = 1U;
  PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_D1_WAIT);
  return 1U;
}

void PressureSensorDriver_Init(UART_HandleTypeDef *huart)
{
  (void)huart; /* Preserve main.c call site; do not arm USART1 in I2C mode. */
  (void)memset((void *)&g_pressure_sensor_state, 0, sizeof(g_pressure_sensor_state));
  (void)memset(s_ms_prom, 0, sizeof(s_ms_prom));
  s_ms_prom_index = 0U;
  s_ms_d1 = 0U;
  s_ms_d1_tick = 0U;
  s_ms_started = 1U;
  g_pressure_sensor_diag.sensor_type = PRESSURE_SENSOR_TYPE_MS5837_30BA;
  g_pressure_sensor_diag.slave_address = PRESSURE_SENSOR_MS5837_I2C_ADDRESS;
  g_pressure_sensor_readout.pressure_max_bar = 30.0f;
  PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_RESET);
}

void PressureSensorDriver_Poll(uint32_t now_ms)
{
  uint32_t now;
  uint8_t data[2];
  /* The caller's cached tick may predate this task's last I2C operation.
   * Use the HAL clock consistently, including wrap-safe elapsed checks. */
  (void)now_ms;
  if (s_ms_started == 0U) return;
  now = HAL_GetTick();
  if (g_pressure_sensor_readout.pressure_valid != 0U &&
      (uint32_t)(now - g_pressure_sensor_readout.pressure_update_tick) >=
          PRESSURE_SENSOR_MS5837_STALE_MS)
  {
    PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_STALE);
    return;
  }
  switch (s_ms_state)
  {
    case PRESSURE_SENSOR_MS5837_STATE_RETRY:
      if ((uint32_t)(now - s_ms_state_tick) < PRESSURE_SENSOR_MS5837_RETRY_MS) return;
      PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_RESET);
      /* fall through */
    case PRESSURE_SENSOR_MS5837_STATE_RESET:
      if (PressureSensorDriver_MsCommand(MS5837_RESET) == HAL_OK)
      {
        g_pressure_sensor_diag.reset_count++;
        s_ms_prom_index = 0U;
        (void)memset(s_ms_prom, 0, sizeof(s_ms_prom));
        PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_RESET_WAIT);
      }
      break;

    case PRESSURE_SENSOR_MS5837_STATE_RESET_WAIT:
      if ((uint32_t)(now - s_ms_state_tick) < 10U) return;
      PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_PROM);
      /* fall through */
    case PRESSURE_SENSOR_MS5837_STATE_PROM:
      if (PressureSensorDriver_MsRead((uint8_t)(MS5837_PROM_BASE + 2U * s_ms_prom_index),
                                     data, 2U) == 0U) return;
      s_ms_prom[s_ms_prom_index] = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
      g_pressure_sensor_diag.prom[s_ms_prom_index] = s_ms_prom[s_ms_prom_index];
      s_ms_prom_index++;
      if (s_ms_prom_index == 7U)
      {
        uint8_t i;
        g_pressure_sensor_diag.prom_crc_expected = s_ms_prom[0] >> 12U;
        g_pressure_sensor_diag.prom_crc_computed = PressureSensorDriver_MsCrc4(s_ms_prom);
        for (i = 1U; i < 7U; ++i)
        {
          if (s_ms_prom[i] == 0U || s_ms_prom[i] == 0xFFFFU)
          {
            PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_PROM_DATA);
            return;
          }
        }
        if (g_pressure_sensor_diag.prom_crc_expected != g_pressure_sensor_diag.prom_crc_computed)
        {
          g_pressure_sensor_diag.crc_error_count++;
          PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_PROM_CRC);
          return;
        }
        g_pressure_sensor_diag.initialized = 1U;
        g_pressure_sensor_diag.scaling_valid = 1U;
        g_pressure_sensor_diag.last_error = 0U;
        PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_D1_START);
      }
      break;

    case PRESSURE_SENSOR_MS5837_STATE_D1_START:
      (void)PressureSensorDriver_MsStartD1();
      break;

    case PRESSURE_SENSOR_MS5837_STATE_D1_WAIT:
      if ((uint32_t)(now - s_ms_state_tick) < PRESSURE_SENSOR_MS5837_CONVERSION_MS) return;
      if ((uint32_t)(now - s_ms_d1_tick) >= PRESSURE_SENSOR_MS5837_STALE_MS)
      {
        PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_STALE);
        return;
      }
      if (PressureSensorDriver_MsAdc(&s_ms_d1) == 0U) return;
      g_pressure_sensor_diag.pressure_raw = s_ms_d1;
      if (PressureSensorDriver_MsCommand(MS5837_D2_OSR8192) == HAL_OK)
        PressureSensorDriver_MsState(PRESSURE_SENSOR_MS5837_STATE_D2_WAIT);
      break;

    case PRESSURE_SENSOR_MS5837_STATE_D2_WAIT:
    {
      uint32_t d2, sample_tick;
      float pressure, temperature;
      if ((uint32_t)(now - s_ms_state_tick) < PRESSURE_SENSOR_MS5837_CONVERSION_MS) return;
      if ((uint32_t)(now - s_ms_d1_tick) >= PRESSURE_SENSOR_MS5837_STALE_MS)
      {
        PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_STALE);
        return;
      }
      if (PressureSensorDriver_MsAdc(&d2) == 0U) return;
      g_pressure_sensor_diag.temperature_raw = d2;
      if (PressureSensorDriver_MsCompensate(s_ms_d1, d2, &pressure, &temperature) == 0U)
      {
        PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_RANGE);
        return;
      }
      sample_tick = HAL_GetTick();
      /* Publish a complete sample atomically to the higher-priority EKF reader. */
      taskENTER_CRITICAL();
      g_pressure_sensor_readout.pressure_bar = pressure;
      g_pressure_sensor_readout.temperature_c = temperature;
      g_pressure_sensor_readout.pressure_update_tick = s_ms_d1_tick;
      g_pressure_sensor_readout.temperature_update_tick = s_ms_state_tick;
      g_pressure_sensor_readout.last_update_tick = sample_tick;
      g_pressure_sensor_readout.pressure_update_seq++;
      g_pressure_sensor_readout.temperature_update_seq++;
      g_pressure_sensor_readout.pressure_valid = 1U;
      g_pressure_sensor_readout.temperature_valid = 1U;
      taskEXIT_CRITICAL();
      g_pressure_sensor_diag.response_ok_count++;
      g_pressure_sensor_diag.last_response_tick = sample_tick;
      g_pressure_sensor_diag.last_error = 0U;
      g_pressure_sensor_diag.response_waiting = 0U;
      (void)PressureSensorDriver_MsStartD1();
      break;
    }
    default:
      PressureSensorDriver_MsFail(PRESSURE_SENSOR_MS5837_ERROR_I2C);
      break;
  }
}

void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *huart) { (void)huart; }
void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *huart) { (void)huart; }
void PressureSensorDriver_OnError(UART_HandleTypeDef *huart) { (void)huart; }
uint16_t PressureSensorDriver_Available(void) { return 0U; }
uint16_t PressureSensorDriver_Read(uint8_t *dst, uint16_t max_len)
{
  (void)dst;
  (void)max_len;
  return 0U;
}
void PressureSensorDriver_ClearRxBuffer(void) {}

#endif
