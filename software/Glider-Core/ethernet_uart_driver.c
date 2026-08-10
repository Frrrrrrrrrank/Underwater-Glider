#include "ethernet_uart_driver.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

volatile EthernetUartState g_ethernet_uart_state = {0};

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_byte = 0U;
static volatile uint8_t s_rx_ring[ETHERNET_UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint32_t s_tx_busy = 0U;

typedef struct
{
  uint16_t length;
  uint8_t is_rdss;
  char data[ETHERNET_UART_TX_LINE_MAX];
} EthernetUartTxQueueItem;

static EthernetUartTxQueueItem s_tx_queue[ETHERNET_UART_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_queue_head = 0U;
static volatile uint8_t s_tx_queue_tail = 0U;
static volatile uint8_t s_tx_from_queue = 0U;
static osMutexId_t s_tx_queue_mutex = NULL;

static void EthernetUartDriver_RearmReceive(void);
static void EthernetUartDriver_RingPush(uint8_t byte);
static bool EthernetUartDriver_RingPop(uint8_t *byte);
static void EthernetUartDriver_ProcessReceivedData(void);
static void EthernetUartDriver_ProcessLine(const char *line);
static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length);
static bool EthernetUartDriver_QueueText(const char *text, uint8_t is_rdss);
static void EthernetUartDriver_TrySendQueued(void);

static void EthernetUartDriver_UpdateQueueDepth(void)
{
  g_ethernet_uart_diag.tx_queue_depth =
      (s_tx_queue_head >= s_tx_queue_tail)
          ? (uint32_t)(s_tx_queue_head - s_tx_queue_tail)
          : (uint32_t)(ETHERNET_UART_TX_QUEUE_DEPTH - s_tx_queue_tail + s_tx_queue_head);
}

static bool EthernetUartDriver_QueueText(const char *text, uint8_t is_rdss)
{
  size_t length;
  uint8_t next;
  EthernetUartTxQueueItem *item;

  if (text == NULL)
  {
    return false;
  }

  length = strlen(text);
  if (length == 0U || length >= ETHERNET_UART_TX_LINE_MAX)
  {
    return false;
  }

  if (s_tx_queue_mutex == NULL || osMutexAcquire(s_tx_queue_mutex, 0U) != osOK)
  {
    return false;
  }

  next = (uint8_t)(s_tx_queue_head + 1U);
  if (next >= ETHERNET_UART_TX_QUEUE_DEPTH)
  {
    next = 0U;
  }
  if (next == s_tx_queue_tail)
  {
    (void)osMutexRelease(s_tx_queue_mutex);
    return false;
  }

  item = &s_tx_queue[s_tx_queue_head];
  memcpy(item->data, text, length + 1U);
  item->length = (uint16_t)length;
  item->is_rdss = is_rdss;
  s_tx_queue_head = next;
  EthernetUartDriver_UpdateQueueDepth();
  (void)osMutexRelease(s_tx_queue_mutex);
  return true;
}

static void EthernetUartDriver_TrySendQueued(void)
{
  EthernetUartTxQueueItem *item;
  HAL_StatusTypeDef status;

  if (s_tx_busy != 0U || s_tx_queue_head == s_tx_queue_tail)
  {
    return;
  }

  item = &s_tx_queue[s_tx_queue_tail];
  s_tx_from_queue = 1U;
  status = EthernetUartDriver_SendRaw((const uint8_t *)item->data, item->length);
  if (status != HAL_OK)
  {
    s_tx_from_queue = 0U;
    if (status != HAL_BUSY)
    {
      if (item->is_rdss != 0U)
      {
        g_ethernet_uart_diag.rdss_forward_drop_count++;
      }
      s_tx_queue_tail = (uint8_t)(s_tx_queue_tail + 1U);
      if (s_tx_queue_tail >= ETHERNET_UART_TX_QUEUE_DEPTH)
      {
        s_tx_queue_tail = 0U;
      }
      EthernetUartDriver_UpdateQueueDepth();
    }
  }
}

static void EthernetUartDriver_RearmReceive(void)
{
  HAL_StatusTypeDef status;

  if (s_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  if (status == HAL_OK)
  {
    g_ethernet_uart_diag.rx_rearm_ok_count++;
    return;
  }

  if (status == HAL_BUSY)
  {
    g_ethernet_uart_diag.rx_rearm_busy_count++;
    s_uart->RxState = HAL_UART_STATE_READY;
    s_uart->ErrorCode = HAL_UART_ERROR_NONE;
    status = HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
    if (status == HAL_OK)
    {
      g_ethernet_uart_diag.rx_rearm_ok_count++;
      return;
    }
  }

  g_ethernet_uart_diag.rx_rearm_fail_count++;
  g_ethernet_uart_diag.last_error = 1U;
}

static void EthernetUartDriver_RingPush(uint8_t byte)
{
  uint16_t next = (uint16_t)(s_rx_head + 1U);
  if (next >= ETHERNET_UART_RX_RING_SIZE)
  {
    next = 0U;
  }

  if (next == s_rx_tail)
  {
    g_ethernet_uart_diag.rx_ring_overflow_count++;
    return;
  }

  s_rx_ring[s_rx_head] = byte;
  s_rx_head = next;
  g_ethernet_uart_diag.rx_byte_count++;
  g_ethernet_uart_diag.ring_available_bytes =
      (s_rx_head >= s_rx_tail)
          ? (uint32_t)(s_rx_head - s_rx_tail)
          : (uint32_t)(ETHERNET_UART_RX_RING_SIZE - s_rx_tail + s_rx_head);
}

static bool EthernetUartDriver_RingPop(uint8_t *byte)
{
  if (s_rx_head == s_rx_tail)
  {
    return false;
  }

  *byte = s_rx_ring[s_rx_tail];
  s_rx_tail++;
  if (s_rx_tail >= ETHERNET_UART_RX_RING_SIZE)
  {
    s_rx_tail = 0U;
  }

  g_ethernet_uart_diag.ring_available_bytes =
      (s_rx_head >= s_rx_tail)
          ? (uint32_t)(s_rx_head - s_rx_tail)
          : (uint32_t)(ETHERNET_UART_RX_RING_SIZE - s_rx_tail + s_rx_head);
  return true;
}

static void EthernetUartDriver_ProcessLine(const char *line)
{
  if (line == NULL || line[0] == '\0')
  {
    return;
  }

  size_t len = strlen(line);
  if (len >= ETHERNET_UART_RX_LINE_MAX)
  {
    g_ethernet_uart_diag.rx_line_overflow_count++;
    return;
  }

  memcpy((void *)g_ethernet_uart_readout.last_rx_line, line, len + 1U);
  g_ethernet_uart_readout.last_rx_byte = (uint32_t)line[len - 1U];
  g_ethernet_uart_diag.rx_line_count++;
  g_ethernet_uart_diag.last_rx_tick = HAL_GetTick();

  if (strcmp(line, "RDSS_FORWARD=1") == 0)
  {
    g_ethernet_uart_diag.rdss_forward_enabled = 1U;
    g_ethernet_uart_diag.rdss_forward_command_count++;
    (void)EthernetUartDriver_QueueText("OK RDSS_FORWARD=1\r\n", 0U);
    return;
  }
  if (strcmp(line, "RDSS_FORWARD=0") == 0)
  {
    g_ethernet_uart_diag.rdss_forward_enabled = 0U;
    g_ethernet_uart_diag.rdss_forward_command_count++;
    (void)EthernetUartDriver_QueueText("OK RDSS_FORWARD=0\r\n", 0U);
    return;
  }
  if (strcmp(line, "RDSS_FORWARD?") == 0)
  {
    g_ethernet_uart_diag.rdss_forward_command_count++;
    (void)EthernetUartDriver_QueueText(
        (g_ethernet_uart_diag.rdss_forward_enabled != 0U)
            ? "RDSS_FORWARD=1\r\n"
            : "RDSS_FORWARD=0\r\n",
        0U);
    return;
  }

  if (strstr(line, "CONNECT") != NULL || strstr(line, "OK") != NULL)
  {
    g_ethernet_uart_diag.last_error = 0U;
  }
  else if (strstr(line, "ERROR") != NULL || strstr(line, "FAIL") != NULL)
  {
    g_ethernet_uart_diag.last_error = 2U;
  }
}

static void EthernetUartDriver_ProcessReceivedData(void)
{
  uint8_t byte;
  static char line[ETHERNET_UART_RX_LINE_MAX];
  static size_t line_len = 0U;

  while (EthernetUartDriver_RingPop(&byte))
  {
    if (byte == '\r' || byte == '\n')
    {
      if (line_len != 0U)
      {
        line[line_len] = '\0';
        EthernetUartDriver_ProcessLine(line);
        line_len = 0U;
      }
      continue;
    }

    if (line_len + 1U < sizeof(line))
    {
      line[line_len++] = (char)byte;
    }
    else
    {
      g_ethernet_uart_diag.rx_line_overflow_count++;
      line_len = 0U;
    }
  }
}

static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length)
{
  uint16_t copy_len;

  if (s_uart == NULL || data == NULL || length == 0U || length > ETHERNET_UART_TX_LINE_MAX)
  {
    return HAL_ERROR;
  }

  if (s_tx_busy != 0U)
  {
    g_ethernet_uart_diag.tx_busy_count++;
    return HAL_BUSY;
  }

  copy_len = (length < ETHERNET_UART_TX_LINE_MAX)
                 ? length
                 : (uint16_t)(ETHERNET_UART_TX_LINE_MAX - 1U);
  memcpy((void *)g_ethernet_uart_readout.last_tx_line, data, copy_len);
  g_ethernet_uart_readout.last_tx_line[copy_len] = '\0';

  HAL_StatusTypeDef status = HAL_UART_Transmit_IT(s_uart, data, length);
  if (status == HAL_OK)
  {
    s_tx_busy = 1U;
    g_ethernet_uart_diag.tx_count++;
    g_ethernet_uart_diag.tx_in_flight = 1U;
  }
  else
  {
    g_ethernet_uart_diag.tx_fail_count++;
    g_ethernet_uart_diag.last_hal_status = (uint32_t)status;
  }

  return status;
}

void EthernetUartDriver_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    g_ethernet_uart_diag.last_error = 1U;
    return;
  }

  s_uart = huart;

  (void)memset((void *)&g_ethernet_uart_state, 0, sizeof(g_ethernet_uart_state));
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_byte = 0U;
  s_tx_busy = 0U;
  s_tx_queue_head = 0U;
  s_tx_queue_tail = 0U;
  s_tx_from_queue = 0U;
  s_tx_queue_mutex = NULL;

  EthernetUartDriver_RearmReceive();
  if (g_ethernet_uart_diag.last_error == 0U)
  {
    g_ethernet_uart_diag.initialized = 1U;
  }
}

void EthernetUartDriver_RunTask(void *argument)
{
  (void)argument;

  s_tx_queue_mutex = osMutexNew(NULL);
  if (s_tx_queue_mutex == NULL)
  {
    g_ethernet_uart_diag.last_error = 6U;
  }

  for (;;)
  {
    g_ethernet_uart_diag.task_loop_count++;

    EthernetUartDriver_ProcessReceivedData();
    EthernetUartDriver_TrySendQueued();

    g_ethernet_uart_readout.ring_available_bytes = g_ethernet_uart_diag.ring_available_bytes;
    osDelay(ETHERNET_UART_TASK_PERIOD_MS);
  }
}

void EthernetUartDriver_SendText(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  uint16_t length = (uint16_t)strlen(text);
  if (length == 0U || length > ETHERNET_UART_TX_LINE_MAX)
  {
    return;
  }

  (void)EthernetUartDriver_SendRaw((const uint8_t *)text, length);
}

void EthernetUartDriver_ForwardRdssLine(const char *line)
{
  char message[ETHERNET_UART_TX_LINE_MAX];
  size_t line_len;
  int written;

  if (g_ethernet_uart_diag.rdss_forward_enabled == 0U || line == NULL)
  {
    return;
  }

  line_len = strlen(line);
  while (line_len > 0U && (line[line_len - 1U] == '\r' || line[line_len - 1U] == '\n'))
  {
    line_len--;
  }

  written = snprintf(message, sizeof(message), "RDSS_RX:%.*s\r\n", (int)line_len, line);
  if (written <= 0 || (size_t)written >= sizeof(message) ||
      !EthernetUartDriver_QueueText(message, 1U))
  {
    g_ethernet_uart_diag.rdss_forward_drop_count++;
    return;
  }

  g_ethernet_uart_diag.rdss_forward_enqueued_count++;
}

void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }

  g_ethernet_uart_diag.rx_irq_count++;
  EthernetUartDriver_RingPush(s_rx_byte);
  EthernetUartDriver_RearmReceive();
}

void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }

  s_tx_busy = 0U;
  g_ethernet_uart_diag.tx_complete_count++;
  g_ethernet_uart_diag.tx_in_flight = 0U;
  g_ethernet_uart_diag.last_tx_tick = HAL_GetTick();

  if (s_tx_from_queue != 0U && s_tx_queue_head != s_tx_queue_tail)
  {
    if (s_tx_queue[s_tx_queue_tail].is_rdss != 0U)
    {
      g_ethernet_uart_diag.rdss_forward_sent_count++;
    }
    s_tx_queue_tail = (uint8_t)(s_tx_queue_tail + 1U);
    if (s_tx_queue_tail >= ETHERNET_UART_TX_QUEUE_DEPTH)
    {
      s_tx_queue_tail = 0U;
    }
    EthernetUartDriver_UpdateQueueDepth();
  }
  s_tx_from_queue = 0U;
}

void EthernetUartDriver_OnError(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }

  g_ethernet_uart_diag.uart_error_code = huart->ErrorCode;
  g_ethernet_uart_diag.last_error = 5U;

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  /* Drain receiver data register to clear overrun condition */
  volatile uint32_t dummy = huart->Instance->RDR;
  (void)dummy;

  huart->RxState = HAL_UART_STATE_READY;
  EthernetUartDriver_RearmReceive();
}
