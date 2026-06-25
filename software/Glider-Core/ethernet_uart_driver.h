#ifndef ETHERNET_UART_DRIVER_H
#define ETHERNET_UART_DRIVER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define ETHERNET_UART_RX_RING_SIZE 256U
#define ETHERNET_UART_RX_LINE_MAX 96U
#define ETHERNET_UART_TX_LINE_MAX 160U
#define ETHERNET_UART_TASK_PERIOD_MS 20U
#define ETHERNET_UART_HEARTBEAT_PERIOD_MS 1000U

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

typedef struct
{
  uint8_t pitchmotor_enable;
  uint32_t pitchmotor_speed_rpm;
  uint8_t rollmotor_enable;
  uint32_t rollmotor_speed_rpm;
  uint8_t thrustmotor_direction;
  uint8_t thrustmotor_speed_rpm;
  uint8_t thrustmotor1se_speed_rpm;
  uint8_t thrustmotor2se_speed_rpm;
  uint8_t backup_for_thrust;
  uint8_t pump_direction;
  uint8_t pump_on_off;
  uint8_t ask_for_status;
  uint8_t eth_control_flag;

} EthernetUartReadoutPack;

extern volatile EthernetUartState g_ethernet_uart_state;
extern volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
extern volatile uint8_t g_ethernet_rx_message_flag;

#define g_ethernet_uart_diag g_ethernet_uart_state.diag
#define g_ethernet_uart_readout g_ethernet_uart_state.readout

void EthernetUartDriver_Init(UART_HandleTypeDef *huart);
void EthernetUartDriver_RunTask(void *argument);
void EthernetUartDriver_SendText(const char *text);
void EthernetUartDriver_SendSystemSnapshot(void);
void EthernetUartDriver_ClearRxRing(void);
bool EthernetUartDriver_ConvertFront19BytesToPack(EthernetUartReadoutPack *out_pack);
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart);
void EthernetUartDriver_OnError(UART_HandleTypeDef *huart);

#endif /* ETHERNET_UART_DRIVER_H */
