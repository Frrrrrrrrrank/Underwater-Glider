#ifndef ETHERNET_UART_DRIVER_H
#define ETHERNET_UART_DRIVER_H

#include "main.h"
#include <stdint.h>

#define ETHERNET_UART_RX_RING_SIZE 256U
#define ETHERNET_UART_RX_LINE_MAX 96U
#define ETHERNET_UART_TX_LINE_MAX 576U
#define ETHERNET_UART_TX_QUEUE_DEPTH 4U
#define ETHERNET_UART_TASK_PERIOD_MS 20U

typedef struct
{
  uint32_t initialized;
  uint32_t rx_armed;
  uint32_t task_loop_count;

  uint32_t rx_irq_count;
  uint32_t rx_rearm_ok_count;
  uint32_t rx_rearm_busy_count;
  uint32_t rx_rearm_fail_count;
  uint32_t rx_byte_count;
  uint32_t rx_ring_overflow_count;
  uint32_t rx_line_count;
  uint32_t rx_line_overflow_count;

  uint32_t tx_count;
  uint32_t tx_complete_count;
  uint32_t tx_busy_count;
  uint32_t tx_fail_count;
  uint32_t tx_in_flight;

  uint32_t command_ping_count;
  uint32_t command_status_count;
  uint32_t command_help_count;
  uint32_t command_unknown_count;
  uint32_t heartbeat_count;

  uint32_t rdss_forward_enabled;
  uint32_t rdss_forward_command_count;
  uint32_t rdss_forward_enqueued_count;
  uint32_t rdss_forward_sent_count;
  uint32_t rdss_forward_drop_count;
  uint32_t tx_queue_depth;

  uint32_t ring_available_bytes;
  uint32_t last_rx_tick;
  uint32_t last_tx_tick;
  uint32_t last_error;
  uint32_t last_hal_status;
  uint32_t uart_error_code;
} EthernetUartDiag;

typedef struct
{
  uint32_t last_rx_byte;
  uint32_t ring_available_bytes;
  char last_rx_line[ETHERNET_UART_RX_LINE_MAX];
  char last_tx_line[ETHERNET_UART_TX_LINE_MAX];
} EthernetUartReadout;

typedef struct
{
  EthernetUartDiag diag;
  EthernetUartReadout readout;
} EthernetUartState;

extern volatile EthernetUartState g_ethernet_uart_state;

#define g_ethernet_uart_diag g_ethernet_uart_state.diag
#define g_ethernet_uart_readout g_ethernet_uart_state.readout

void EthernetUartDriver_Init(UART_HandleTypeDef *huart);
void EthernetUartDriver_RunTask(void *argument);
void EthernetUartDriver_SendText(const char *text);
void EthernetUartDriver_ForwardRdssLine(const char *line);
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnError(UART_HandleTypeDef *huart);

#endif /* ETHERNET_UART_DRIVER_H */
