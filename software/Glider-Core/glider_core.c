#include "glider_core.h"

#include "cmsis_os.h"
#include "glider_c01_proto.h"

#include <string.h>

#define GLIDER_RX_RING_SIZE 512U
#define GLIDER_RX_FRAME_MAX_SIZE GLIDER_C01_MAX_FRAME_SIZE

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_ring[GLIDER_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint8_t s_rx_byte = 0;

volatile GliderCoreDiag g_glider_core_diag = {0};

#define s_rx_overflow_count g_glider_core_diag.rx_overflow_count
#define s_rx_line_count g_glider_core_diag.rx_line_count
#define s_tx_attempt_count g_glider_core_diag.tx_attempt_count
#define s_tx_ok_count g_glider_core_diag.tx_ok_count
#define s_tx_fail_count g_glider_core_diag.tx_fail_count
#define s_tx_last_hal_status g_glider_core_diag.tx_last_hal_status
#define s_tx_last_uart_error g_glider_core_diag.tx_last_uart_error
#define s_rx_frame_count g_glider_core_diag.rx_frame_count
#define s_rx_frame_checksum_error_count g_glider_core_diag.rx_frame_checksum_error_count
#define s_rx_last_function_code g_glider_core_diag.rx_last_function_code
#define s_rx_remote_ack_count g_glider_core_diag.rx_remote_ack_count
#define s_rx_remote_data_count g_glider_core_diag.rx_remote_data_count
#define s_rx_last_decoded_status g_glider_core_diag.rx_last_decoded_status
#define s_rx_last_decoded_data g_glider_core_diag.rx_last_decoded_data

static uint8_t s_rx_frame_buffer[GLIDER_RX_FRAME_MAX_SIZE];
static size_t s_rx_frame_len = 0;
static size_t s_rx_expected_len = 0;

static const uint8_t s_c01_address = 0x01U;

static void diag_copy_bytes(volatile uint8_t *dst, size_t dst_len, const uint8_t *src, size_t src_len)
{
  size_t i = 0U;
  size_t copy_len = 0U;

  if (dst == NULL || src == NULL || dst_len == 0U)
  {
    return;
  }

  copy_len = (src_len < dst_len) ? src_len : dst_len;
  for (i = 0U; i < copy_len; i++)
  {
    dst[i] = src[i];
  }
  for (; i < dst_len; i++)
  {
    dst[i] = 0U;
  }
}

static void decode_remote_frame(const GliderC01Frame *decoded)
{
  if (decoded == NULL)
  {
    return;
  }

  s_rx_last_decoded_status = decoded->status;

  if (decoded->payload_len >= 2U)
  {
    s_rx_last_decoded_data = decoded->payload[1];
  }
  else
  {
    s_rx_last_decoded_data = 0U;
  }

  if (decoded->function == (uint8_t)GLIDER_C01_FUNC_RX_REMOTE_ACK)
  {
    s_rx_remote_ack_count++;
  }
  else if (decoded->function == (uint8_t)GLIDER_C01_FUNC_RX_REMOTE)
  {
    s_rx_remote_data_count++;
  }
}

static HAL_StatusTypeDef send_bytes(const uint8_t *data, size_t len)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  if (s_uart == NULL || data == NULL || len == 0U)
  {
    s_tx_fail_count++;
    s_tx_last_hal_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  s_tx_attempt_count++;
  g_glider_core_diag.tx_last_encoded_len = (len <= GLIDER_C01_MAX_FRAME_SIZE) ? (uint32_t)len : GLIDER_C01_MAX_FRAME_SIZE;
  diag_copy_bytes(g_glider_core_diag.tx_last_encoded_frame,
                  GLIDER_C01_MAX_FRAME_SIZE,
                  data,
                  len);
  ret = HAL_UART_Transmit(s_uart, (uint8_t *)data, (uint16_t)len, 200U);
  s_tx_last_hal_status = (uint32_t)ret;
  s_tx_last_uart_error = (uint32_t)s_uart->ErrorCode;

  if (ret == HAL_OK)
  {
    s_tx_ok_count++;
  }
  else
  {
    s_tx_fail_count++;
  }

  return ret;
}

static bool ring_pop(uint8_t *out)
{
  uint16_t tail = 0;

  if (out == NULL)
  {
    return false;
  }

  if (s_rx_tail == s_rx_head)
  {
    return false;
  }

  tail = s_rx_tail;
  *out = s_rx_ring[tail];
  s_rx_tail = (uint16_t)((tail + 1U) % GLIDER_RX_RING_SIZE);
  return true;
}

static void ring_push(const uint8_t data)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % GLIDER_RX_RING_SIZE);

  if (next == s_rx_tail)
  {
    s_rx_overflow_count++;
    return;
  }

  s_rx_ring[s_rx_head] = data;
  s_rx_head = next;
}

static void handle_rx_frame(const uint8_t *frame, size_t frame_len)
{
  GliderC01Frame decoded;

  if (!GliderC01_TryParseFrame(frame, frame_len, &decoded))
  {
    return;
  }

  s_rx_frame_count++;
  s_rx_line_count = s_rx_frame_count;
  s_rx_last_function_code = decoded.function;
  g_glider_core_diag.rx_last_encoded_len = (frame_len <= GLIDER_C01_MAX_FRAME_SIZE) ? (uint32_t)frame_len : GLIDER_C01_MAX_FRAME_SIZE;
  diag_copy_bytes(g_glider_core_diag.rx_last_encoded_frame,
                  GLIDER_C01_MAX_FRAME_SIZE,
                  frame,
                  frame_len);

  if (!decoded.checksum_valid)
  {
    s_rx_frame_checksum_error_count++;
    return;
  }

  if (decoded.function == (uint8_t)GLIDER_C01_FUNC_RX_REMOTE_ACK ||
      decoded.function == (uint8_t)GLIDER_C01_FUNC_RX_REMOTE)
  {
    decode_remote_frame(&decoded);
  }

  g_glider_core_diag.rx_last_decoded_payload_len =
      (decoded.payload_len <= (GLIDER_C01_MAX_FRAME_SIZE - 5U)) ? (uint32_t)decoded.payload_len : (GLIDER_C01_MAX_FRAME_SIZE - 5U);
  diag_copy_bytes(g_glider_core_diag.rx_last_decoded_payload,
                  (GLIDER_C01_MAX_FRAME_SIZE - 5U),
                  decoded.payload,
                  decoded.payload_len);

  GliderCore_OnBinaryFrame(frame, frame_len);
}

static bool is_supported_function(uint8_t function)
{
  switch (function)
  {
    case (uint8_t)GLIDER_C01_FUNC_RX_REMOTE_ACK:
    case (uint8_t)GLIDER_C01_FUNC_RX_REMOTE:
    case (uint8_t)GLIDER_C01_FUNC_RX_POSITION:
    case (uint8_t)GLIDER_C01_FUNC_RX_TRIM:
    case (uint8_t)GLIDER_C01_FUNC_RX_MULTI:
      return true;
    default:
      return false;
  }
}

static void process_rx_byte(const uint8_t byte)
{
  if (s_rx_frame_len == 0U)
  {
    if (byte == 0x00U || byte == 0xFFU)
    {
      return;
    }
    s_rx_frame_buffer[s_rx_frame_len++] = byte;
    s_rx_expected_len = 0U;
    return;
  }

  if (s_rx_frame_len >= GLIDER_RX_FRAME_MAX_SIZE)
  {
    s_rx_frame_len = 0U;
    s_rx_expected_len = 0U;
    return;
  }

  s_rx_frame_buffer[s_rx_frame_len++] = byte;

  if (s_rx_frame_len == 2U)
  {
    if (!is_supported_function(s_rx_frame_buffer[1]))
    {
      s_rx_frame_len = 0U;
      s_rx_expected_len = 0U;
    }
    return;
  }

  if (s_rx_frame_len == 3U)
  {
    s_rx_expected_len = s_rx_frame_buffer[2];
    if (s_rx_expected_len < 5U || s_rx_expected_len > GLIDER_RX_FRAME_MAX_SIZE)
    {
      s_rx_frame_len = 0U;
      s_rx_expected_len = 0U;
    }
    return;
  }

  if (s_rx_expected_len > 0U && s_rx_frame_len >= s_rx_expected_len)
  {
    handle_rx_frame(s_rx_frame_buffer, s_rx_expected_len);
    s_rx_frame_len = 0U;
    s_rx_expected_len = 0U;
  }
}

void GliderCore_Init(UART_HandleTypeDef *uart_handle)
{
  s_uart = uart_handle;
  s_rx_head = 0;
  s_rx_tail = 0;
  s_rx_frame_len = 0;
  s_rx_expected_len = 0;
  memset((void *)&g_glider_core_diag, 0, sizeof(g_glider_core_diag));
  g_glider_core_diag.device_role = GLIDER_C01_DEVICE_ROLE;
#if GLIDER_C01_DEVICE_ROLE == GLIDER_C01_ROLE_MASTER
  g_glider_core_diag.tx_enabled = 1U;
#else
  g_glider_core_diag.tx_enabled = 0U;
#endif

  if (s_uart != NULL)
  {
    (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  }
}

void GliderCore_RunRxTask(void)
{
  uint8_t byte = 0;

  for (;;)
  {
    while (ring_pop(&byte))
    {
      process_rx_byte(byte);
    }

    osDelay(2U);
  }
}

void GliderCore_RunTxTask(void)
{
#if GLIDER_C01_DEVICE_ROLE == GLIDER_C01_ROLE_MASTER
  uint8_t tx_frame[GLIDER_C01_MAX_FRAME_SIZE];
  size_t tx_len = 0U;
  uint8_t remote_data = 0x00U;
#endif

  for (;;)
  {
#if GLIDER_C01_DEVICE_ROLE == GLIDER_C01_ROLE_MASTER
    if (GliderC01_BuildRemoteCommand(s_c01_address, remote_data, tx_frame, sizeof(tx_frame), &tx_len))
    {
      (void)send_bytes(tx_frame, tx_len);
    }

    remote_data++;
    osDelay(1300U);
#else
    osDelay(20U);
#endif
  }
}

HAL_StatusTypeDef GliderCore_SendText(const char *text)
{
  size_t len = 0;

  if (s_uart == NULL || text == NULL)
  {
    s_tx_fail_count++;
    s_tx_last_hal_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  len = strlen(text);
  if (len == 0U)
  {
    return HAL_OK;
  }

  return send_bytes((const uint8_t *)text, len);
}

uint32_t GliderCore_GetRxLineCount(void)
{
  return s_rx_line_count;
}

uint32_t GliderCore_GetRxOverflowCount(void)
{
  return s_rx_overflow_count;
}

uint32_t GliderCore_GetTxAttemptCount(void)
{
  return s_tx_attempt_count;
}

uint32_t GliderCore_GetTxOkCount(void)
{
  return s_tx_ok_count;
}

uint32_t GliderCore_GetTxFailCount(void)
{
  return s_tx_fail_count;
}

uint32_t GliderCore_GetLastTxHalStatus(void)
{
  return s_tx_last_hal_status;
}

uint32_t GliderCore_GetLastUartErrorCode(void)
{
  return s_tx_last_uart_error;
}

uint32_t GliderCore_GetRxFrameCount(void)
{
  return s_rx_frame_count;
}

uint32_t GliderCore_GetRxFrameChecksumErrorCount(void)
{
  return s_rx_frame_checksum_error_count;
}

uint32_t GliderCore_GetLastRxFunctionCode(void)
{
  return s_rx_last_function_code;
}

uint32_t GliderCore_GetRxRemoteAckCount(void)
{
  return s_rx_remote_ack_count;
}

uint32_t GliderCore_GetRxRemoteDataCount(void)
{
  return s_rx_remote_data_count;
}

uint32_t GliderCore_GetRxLastDecodedStatus(void)
{
  return s_rx_last_decoded_status;
}

uint32_t GliderCore_GetRxLastDecodedData(void)
{
  return s_rx_last_decoded_data;
}

const volatile GliderCoreDiag *GliderCore_GetDiag(void)
{
  return &g_glider_core_diag;
}

__weak void GliderCore_OnTextFrame(const char *line)
{
  (void)line;
}

__weak void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len)
{
  (void)payload;
  (void)payload_len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (s_uart != NULL && huart->Instance == s_uart->Instance)
  {
    g_glider_core_diag.rx_byte_count++;
    ring_push(s_rx_byte);
    (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  }
}
