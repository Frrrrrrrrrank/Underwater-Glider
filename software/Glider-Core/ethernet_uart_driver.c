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
static uint32_t s_last_command_tick = 0U;

static void EthernetUartDriver_RearmReceive(void);
static void EthernetUartDriver_RingPush(uint8_t byte);
static bool EthernetUartDriver_RingPop(uint8_t *byte);
static void EthernetUartDriver_ProcessReceivedData(void);
static void EthernetUartDriver_ProcessLine(const char *line);
static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length);

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

  memcpy(g_ethernet_uart_readout.last_rx_line, line, len + 1U);
  g_ethernet_uart_readout.last_rx_byte = (uint32_t)line[len - 1U];
  g_ethernet_uart_diag.rx_line_count++;
  g_ethernet_uart_diag.last_rx_tick = HAL_GetTick();

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
  if (s_uart == NULL || data == NULL || length == 0U || length > ETHERNET_UART_TX_LINE_MAX)
  {
    return HAL_ERROR;
  }

  if (s_tx_busy != 0U)
  {
    g_ethernet_uart_diag.tx_busy_count++;
    return HAL_BUSY;
  }

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
  s_last_command_tick = 0U;

  EthernetUartDriver_RearmReceive();
  if (g_ethernet_uart_diag.last_error == 0U)
  {
    g_ethernet_uart_diag.initialized = 1U;
  }
}

void EthernetUartDriver_RunTask(void *argument)
{
  (void)argument;
  const uint32_t setup_period = ETHERNET_UART_HEARTBEAT_PERIOD_MS;
  const char *setup_commands[] = {
      "AT\r\n",
      "AT+IPR=115200\r\n",
      "AT+NETP=1,0,12345,0\r\n",
      "AT+ENTM\r\n"
  };
  const size_t setup_count = sizeof(setup_commands) / sizeof(setup_commands[0]);
  static size_t setup_index = 0U;

  for (;;)
  {
    const uint32_t now = HAL_GetTick();
    g_ethernet_uart_diag.task_loop_count++;

    EthernetUartDriver_ProcessReceivedData();

    if (setup_index < setup_count)
    {
      if ((now - s_last_command_tick) >= setup_period)
      {
        EthernetUartDriver_SendText(setup_commands[setup_index]);
        s_last_command_tick = now;
        setup_index++;
      }
    }
    else if ((now - g_ethernet_uart_diag.last_tx_tick) >= ETHERNET_UART_HEARTBEAT_PERIOD_MS)
    {
      EthernetUartDriver_SendText("AT\r\n");
      g_ethernet_uart_diag.heartbeat_count++;
      g_ethernet_uart_diag.last_tx_tick = now;
    }

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
