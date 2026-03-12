#include "glider_core.h"

#include "cmsis_os.h"
#include "glider_rdss_codec.h"

#include <string.h>

#define GLIDER_RX_RING_SIZE 512U
#define GLIDER_RX_LINE_SIZE 192U

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_ring[GLIDER_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint8_t s_rx_byte = 0;

static volatile uint32_t s_rx_overflow_count = 0;
static volatile uint32_t s_rx_line_count = 0;
static volatile uint32_t s_tx_attempt_count = 0;
static volatile uint32_t s_tx_ok_count = 0;
static volatile uint32_t s_tx_fail_count = 0;
static volatile uint32_t s_tx_last_hal_status = 0;
static volatile uint32_t s_tx_last_uart_error = 0;

static char s_line_buffer[GLIDER_RX_LINE_SIZE];
static size_t s_line_len = 0;
static GliderRdssDecodedFrame s_decoded_frame;

static const char *s_boot_test_cmds[] = {
  "AT\r\n",
  "ATI\r\n",
  "AT+CSQ\r\n"
};

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

static void process_line(const char *line)
{
  if (!GliderRdssCodec_TryDecodeLine(line, &s_decoded_frame))
  {
    return;
  }

  s_rx_line_count++;
  GliderCore_OnDecodedFrame(&s_decoded_frame);

  if (s_decoded_frame.type == GLIDER_RDSS_FRAME_HEX_BINARY)
  {
    GliderCore_OnBinaryFrame(s_decoded_frame.payload, s_decoded_frame.payload_len);
    return;
  }

  GliderCore_OnTextFrame(s_decoded_frame.text);
}

static void process_rx_byte(const uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    if (s_line_len > 0)
    {
      s_line_buffer[s_line_len] = '\0';
      process_line(s_line_buffer);
      s_line_len = 0;
    }
    return;
  }

  if (s_line_len >= (GLIDER_RX_LINE_SIZE - 1U))
  {
    s_line_len = 0;
  }

  s_line_buffer[s_line_len++] = (char)byte;
}

void GliderCore_Init(UART_HandleTypeDef *uart_handle)
{
  s_uart = uart_handle;
  s_rx_head = 0;
  s_rx_tail = 0;
  s_line_len = 0;
  s_rx_overflow_count = 0;
  s_rx_line_count = 0;
  s_tx_attempt_count = 0;
  s_tx_ok_count = 0;
  s_tx_fail_count = 0;
  s_tx_last_hal_status = 0;
  s_tx_last_uart_error = 0;

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
  size_t idx = 0;

  for (idx = 0; idx < (sizeof(s_boot_test_cmds) / sizeof(s_boot_test_cmds[0])); idx++)
  {
    (void)GliderCore_SendText(s_boot_test_cmds[idx]);
    osDelay(500U);
  }

  for (;;)
  {
    (void)GliderCore_SendText("AT+TESTRD\r\n");
    osDelay(5000U);
  }
}

HAL_StatusTypeDef GliderCore_SendText(const char *text)
{
  size_t len = 0;
  HAL_StatusTypeDef ret = HAL_ERROR;

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

  s_tx_attempt_count++;
  ret = HAL_UART_Transmit(s_uart, (uint8_t *)text, (uint16_t)len, 200U);
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

__weak void GliderCore_OnTextFrame(const char *line)
{
  (void)line;
}

__weak void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len)
{
  (void)payload;
  (void)payload_len;
}

__weak void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame)
{
  (void)frame;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (s_uart != NULL && huart->Instance == s_uart->Instance)
  {
    ring_push(s_rx_byte);
    (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  }
}
