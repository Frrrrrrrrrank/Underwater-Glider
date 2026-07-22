#include "pump_driver.h"
#include "sbus_driver.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "displacement_sensor.h"
#include "ethernet_uart_driver.h"

#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;
static osMutexId_t s_pump_bus_mutex = NULL;
static const osMutexAttr_t s_pump_bus_mutex_attr = {
  .name = "pumpUart3Bus"
};

volatile PumpMotorUart3Diag g_pump_motor_uart3_diag = {0};
volatile PumpDriverStatus g_pump_driver_status = {0};
volatile IoControlPumpDebugInfo g_io_pump_debug = {0};

static void PumpDriver_BusInit(void)
{
  if (s_pump_bus_mutex == NULL)
  {
    s_pump_bus_mutex = osMutexNew(&s_pump_bus_mutex_attr);
  }
}

static uint32_t PumpDriver_BusLock(uint32_t timeout_ms)
{
  if (s_pump_bus_mutex == NULL)
  {
    PumpDriver_BusInit();
  }

  if (s_pump_bus_mutex == NULL)
  {
    return 0U;
  }

  return (osMutexAcquire(s_pump_bus_mutex, timeout_ms) == osOK) ? 1U : 0U;
}

static void PumpDriver_BusUnlock(void)
{
  if (s_pump_bus_mutex != NULL)
  {
    (void)osMutexRelease(s_pump_bus_mutex);
  }
}

static uint32_t PumpDriver_GetTick(void)
{
  return HAL_GetTick();
}

static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
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

static uint16_t PumpDriver_BuildReadHoldingReg(uint8_t node_id, uint16_t reg_addr, uint8_t *frame)
{
  uint16_t crc;

  frame[0] = node_id;
  frame[1] = 0x03U;
  frame[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
  frame[3] = (uint8_t)(reg_addr & 0xFFU);
  frame[4] = 0x00U;
  frame[5] = 0x01U;

  crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

static uint16_t PumpDriver_BuildWriteSingleReg(uint8_t node_id, uint16_t reg_addr, uint16_t value, uint8_t *frame)
{
  uint16_t crc;

  frame[0] = node_id;
  frame[1] = 0x06U;
  frame[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
  frame[3] = (uint8_t)(reg_addr & 0xFFU);
  frame[4] = (uint8_t)((value >> 8) & 0xFFU);
  frame[5] = (uint8_t)(value & 0xFFU);

  crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

static uint8_t PumpDriver_ModbusExchange(const uint8_t *tx_frame,
                                         uint16_t tx_len,
                                         uint8_t *rx_frame,
                                         uint16_t rx_capacity,
                                         uint16_t timeout_ms,
                                         uint16_t *out_rx_len)
{
  uint32_t start_tick;
  uint16_t rx_len = 0U;
  uint8_t rx_byte = 0U;
  HAL_StatusTypeDef tx_status;

  if (tx_frame == NULL || rx_frame == NULL || rx_capacity == 0U || s_huart == NULL)
  {
    g_pump_motor_uart3_diag.last_error = 1U;
    return 0U;
  }

  if (PumpDriver_BusLock((uint32_t)timeout_ms + PUMP_MOTOR_UART3_TX_TIMEOUT_MS + 10U) == 0U)
  {
    g_pump_motor_uart3_diag.last_error = 21U;
    return 0U;
  }

  tx_status = HAL_UART_Transmit(s_huart, (uint8_t *)tx_frame, tx_len, PUMP_MOTOR_UART3_TX_TIMEOUT_MS);
  if (tx_status != HAL_OK)
  {
    g_pump_motor_uart3_diag.last_error = 30U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  g_pump_motor_uart3_diag.tx_count++;
  memset((void *)g_pump_motor_uart3_diag.last_tx_frame, 0, sizeof(g_pump_motor_uart3_diag.last_tx_frame));
  if (tx_len > sizeof(g_pump_motor_uart3_diag.last_tx_frame))
  {
    tx_len = sizeof(g_pump_motor_uart3_diag.last_tx_frame);
  }
  memcpy((void *)g_pump_motor_uart3_diag.last_tx_frame, tx_frame, tx_len);

  start_tick = PumpDriver_GetTick();
  while ((PumpDriver_GetTick() - start_tick) < timeout_ms)
  {
    if (HAL_UART_Receive(s_huart, &rx_byte, 1U, 10U) == HAL_OK)
    {
      if (rx_len < rx_capacity)
      {
        rx_frame[rx_len++] = rx_byte;
      }
    }
    else if (rx_len > 0U)
    {
      break;
    }
  }

  if (rx_len == 0U)
  {
    g_pump_motor_uart3_diag.timeout_count++;
    g_pump_motor_uart3_diag.last_error = 31U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  g_pump_motor_uart3_diag.rx_count++;
  if (rx_len > sizeof(g_pump_motor_uart3_diag.last_rx_frame))
  {
    rx_len = sizeof(g_pump_motor_uart3_diag.last_rx_frame);
  }
  memcpy((void *)g_pump_motor_uart3_diag.last_rx_frame, rx_frame, rx_len);
  if (out_rx_len != NULL)
  {
    *out_rx_len = rx_len;
  }

  PumpDriver_BusUnlock();
  return 1U;
}

static uint8_t PumpDriver_ModbusReadHoldingReg(uint16_t reg_addr, uint16_t *out_value)
{
  uint8_t tx_frame[8];
  uint8_t rx_frame[8];
  uint16_t rx_len = 0U;
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (out_value == NULL)
  {
    g_pump_motor_uart3_diag.last_error = 2U;
    return 0U;
  }

  (void)PumpDriver_BuildReadHoldingReg(g_pump_motor_uart3_diag.node_id, reg_addr, tx_frame);
  if (PumpDriver_ModbusExchange(tx_frame, 8U, rx_frame, sizeof(rx_frame), PUMP_MOTOR_UART3_RX_TIMEOUT_MS, &rx_len) == 0U)
  {
    return 0U;
  }

  if (rx_len == 5U && rx_frame[1] == (uint8_t)(0x03U | 0x80U))
  {
    g_pump_motor_uart3_diag.last_error = rx_frame[2];
    return 0U;
  }

  if (rx_len < 7U)
  {
    g_pump_motor_uart3_diag.last_error = 4U;
    return 0U;
  }

  if (rx_frame[0] != g_pump_motor_uart3_diag.node_id || rx_frame[1] != 0x03U || rx_frame[2] != 2U)
  {
    g_pump_motor_uart3_diag.last_error = 4U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[6] << 8) | rx_frame[5]);
  calc_crc = Modbus_CRC16(rx_frame, 5U);
  if (recv_crc != calc_crc)
  {
    g_pump_motor_uart3_diag.last_error = 5U;
    return 0U;
  }

  *out_value = (uint16_t)(((uint16_t)rx_frame[3] << 8) | rx_frame[4]);
  g_pump_motor_uart3_diag.last_error = 0U;
  return 1U;
}

static uint8_t PumpDriver_ModbusWriteSingleReg(uint16_t reg_addr, uint16_t value)
{
  uint8_t tx_frame[8];
  uint8_t rx_frame[8];
  uint16_t rx_len = 0U;
  uint16_t recv_crc;
  uint16_t calc_crc;

  (void)PumpDriver_BuildWriteSingleReg(g_pump_motor_uart3_diag.node_id, reg_addr, value, tx_frame);
  if (PumpDriver_ModbusExchange(tx_frame, 8U, rx_frame, sizeof(rx_frame), PUMP_MOTOR_UART3_RX_TIMEOUT_MS, &rx_len) == 0U)
  {
    return 0U;
  }

  if (rx_len == 5U && rx_frame[1] == (uint8_t)(0x06U | 0x80U))
  {
    g_pump_motor_uart3_diag.last_error = rx_frame[2];
    return 0U;
  }

  if (rx_len < 8U)
  {
    g_pump_motor_uart3_diag.last_error = 14U;
    return 0U;
  }

  if (rx_frame[0] != g_pump_motor_uart3_diag.node_id || rx_frame[1] != 0x06U)
  {
    g_pump_motor_uart3_diag.last_error = 14U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[7] << 8) | rx_frame[6]);
  calc_crc = Modbus_CRC16(rx_frame, 6U);
  if (recv_crc != calc_crc)
  {
    g_pump_motor_uart3_diag.last_error = 15U;
    return 0U;
  }

  g_pump_motor_uart3_diag.last_error = 0U;
  return 1U;
}

static uint8_t PumpDriver_QueryStatusCode(uint16_t *out_value)
{
  return PumpDriver_ModbusReadHoldingReg(0x0001U, out_value);
}

static uint8_t PumpDriver_QueryAlarmCode(uint16_t *out_value)
{
  return PumpDriver_ModbusReadHoldingReg(0x0000U, out_value);
}

static uint8_t PumpDriver_QuerySpeedRpm(int32_t *out_value)
{
  uint16_t raw_speed = 0U;

  if (out_value == NULL)
  {
    return 0U;
  }

  if (PumpDriver_ModbusReadHoldingReg(0x000AU, &raw_speed) == 0U)
  {
    return 0U;
  }

  *out_value = (int32_t)((uint32_t)raw_speed * 60U / 240U);
  return 1U;
}

void PumpDriver_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  PumpDriver_BusInit();
  memset((void *)&g_pump_motor_uart3_diag, 0, sizeof(g_pump_motor_uart3_diag));
  memset((void *)&g_pump_driver_status, 0, sizeof(g_pump_driver_status));
  g_pump_motor_uart3_diag.initialized = 1U;
  g_pump_motor_uart3_diag.node_id = PUMP_MOTOR_UART3_DEFAULT_NODE_ID;
  g_pump_driver_status.motor.initialized = 1U;
  g_pump_driver_status.motor.node_id = PUMP_MOTOR_UART3_DEFAULT_NODE_ID;
}

uint8_t PumpDriver_Start(void)
{
  return PumpDriver_ModbusWriteSingleReg(0x007CU, 0x0096U);
}

uint8_t PumpDriver_Stop(void)
{
  return PumpDriver_ModbusWriteSingleReg(0x007CU, 0x00D8U);
}

uint8_t PumpDriver_SetSpeed(uint16_t speed_240rps)
{
  return PumpDriver_ModbusWriteSingleReg(0x0030U, speed_240rps);
}

uint8_t PumpDriver_ReadSpeed(int32_t *out_speed_rpm)
{
  if (out_speed_rpm == NULL)
  {
    return 0U;
  }

  if (PumpDriver_QuerySpeedRpm(out_speed_rpm) != 0U)
  {
    g_pump_motor_uart3_diag.feedback_speed_rpm = *out_speed_rpm;
    return 1U;
  }
  return 0U;
}

static void PumpDriver_SyncStatus(uint32_t now_ms)
{
  g_pump_driver_status.displacement.valid = 0U;
  g_pump_driver_status.displacement.sensor_id = 0U;
  g_pump_driver_status.displacement.displacement_mm = 0.0f;
  g_pump_driver_status.displacement.tx_count = 0U;
  g_pump_driver_status.displacement.rx_count = 0U;
  g_pump_driver_status.displacement.timeout_count = 0U;
  g_pump_driver_status.displacement.crc_error_count = 0U;
  g_pump_driver_status.displacement.last_error = 0U;
  g_pump_driver_status.displacement.online = 0U;

  g_pump_driver_status.motor.initialized = g_pump_motor_uart3_diag.initialized;
  g_pump_driver_status.motor.node_id = g_pump_motor_uart3_diag.node_id;
  g_pump_driver_status.motor.valid = g_pump_motor_uart3_diag.valid;
  g_pump_driver_status.motor.status_code = g_pump_motor_uart3_diag.status_code;
  g_pump_driver_status.motor.alarm_code = g_pump_motor_uart3_diag.alarm_code;
  g_pump_driver_status.motor.feedback_speed_rpm = g_pump_motor_uart3_diag.feedback_speed_rpm;
  g_pump_driver_status.motor.tx_count = g_pump_motor_uart3_diag.tx_count;
  g_pump_driver_status.motor.rx_count = g_pump_motor_uart3_diag.rx_count;
  g_pump_driver_status.motor.timeout_count = g_pump_motor_uart3_diag.timeout_count;
  g_pump_driver_status.motor.last_error = g_pump_motor_uart3_diag.last_error;
  g_pump_driver_status.motor.online =
    (g_pump_driver_status.motor.last_seen_tick != 0U &&
     (now_ms - g_pump_driver_status.motor.last_seen_tick) < 500U) ? 1U : 0U;
}

void PumpDriver_Poll(uint32_t now_ms)
{
  uint16_t parsed_value = 0U;

  if (g_pump_motor_uart3_diag.initialized == 0U)
  {
    return;
  }

  if ((now_ms - g_pump_driver_status.motor.last_seen_tick) < PUMP_MOTOR_UART3_POLL_PERIOD_MS &&
      g_pump_driver_status.motor.last_seen_tick != 0U)
  {
    PumpDriver_SyncStatus(now_ms);
    return;
  }

  if (PumpDriver_QueryStatusCode(&parsed_value) != 0U)
  {
    g_pump_motor_uart3_diag.status_code = parsed_value;
    g_pump_motor_uart3_diag.valid = 1U;
    g_pump_driver_status.motor.status_code = parsed_value;
    g_pump_driver_status.motor.valid = 1U;
    g_pump_driver_status.motor.last_seen_tick = now_ms;
  }

  PumpDriver_SyncStatus(now_ms);
}

/* ============================================================================
 * IO Control Pump Driver Functions
 * ============================================================================
 */

typedef struct
{
  uint32_t initialized;
  uint32_t speed_bit0_high;    /* 1 = PC13 speed bit0 driven high/run command */
  uint32_t speed_bit1_low;     /* 1 = PC14 speed bit1 driven low */
  uint32_t stp_high;           /* 1 = PC15 driven high */
  uint32_t direction_reversed; /* 1 = PE4 direction driven low */
} IoControlPumpState;

static IoControlPumpState s_io_pump_state = {0};

void IoControlPump_Init(void)
{
  s_io_pump_state.initialized = 1U;
  s_io_pump_state.speed_bit0_high = 0U;
  s_io_pump_state.speed_bit1_low = 1U;
  s_io_pump_state.stp_high = 1U;
  s_io_pump_state.direction_reversed = 0U;
  
  /* Keep pump stopped by default: B0 low, B1 low, STP high. */
  if (IO_PUMP_SPEED_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_SPEED_PORT, IO_PUMP_SPEED_PIN, GPIO_PIN_RESET);
  }
  if (IO_PUMP_SPEED_B1_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_SPEED_B1_PORT, IO_PUMP_SPEED_B1_PIN, GPIO_PIN_RESET);
  }
  if (IO_PUMP_STP_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_STP_PORT, IO_PUMP_STP_PIN, GPIO_PIN_SET);
  }
  if (IO_PUMP_DIR_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_DIR_PORT, IO_PUMP_DIR_PIN, GPIO_PIN_SET);
  }

  g_io_pump_debug.speed_triggered = 0U;
  g_io_pump_debug.direction_reversed = 0U;
}

void IoControlPump_SetSpeed(uint8_t enabled)
{
  if (s_io_pump_state.initialized == 0U)
  {
    return;
  }
  
  s_io_pump_state.speed_bit0_high = (enabled != 0U) ? 1U : 0U;
  
  if (IO_PUMP_SPEED_PORT != NULL)
  {
    GPIO_PinState pin_state = (s_io_pump_state.speed_bit0_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(IO_PUMP_SPEED_PORT, IO_PUMP_SPEED_PIN, pin_state);
  }

  /* Speed bit1 is not controlled by upper computer yet: keep it driven low. */
  s_io_pump_state.speed_bit1_low = 1U;
  if (IO_PUMP_SPEED_B1_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_SPEED_B1_PORT, IO_PUMP_SPEED_B1_PIN, GPIO_PIN_RESET);
  }

  /* STP must stay high unless a dedicated stop policy is added. */
  s_io_pump_state.stp_high = 1U;
  if (IO_PUMP_STP_PORT != NULL)
  {
    HAL_GPIO_WritePin(IO_PUMP_STP_PORT, IO_PUMP_STP_PIN, GPIO_PIN_SET);
  }
}

void IoControlPump_SetDirection(uint8_t reversed)
{
  if (s_io_pump_state.initialized == 0U)
  {
    return;
  }
  
  s_io_pump_state.direction_reversed = (reversed != 0U) ? 1U : 0U;
  
  if (IO_PUMP_DIR_PORT != NULL)
  {
    GPIO_PinState pin_state = (s_io_pump_state.direction_reversed == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(IO_PUMP_DIR_PORT, IO_PUMP_DIR_PIN, pin_state);
  }
}

void IoControlPump_Poll(uint32_t now_ms)
{
  uint16_t channels[18] = {0U};
  uint32_t frame_lost = 0U;
  uint32_t failsafe = 0U;
  uint32_t last_update_tick = 0U;
  uint32_t channel_count = 0U;
  uint16_t right_outer_switch = 0U;
  uint16_t left_mid2_switch = 0U;
  const uint32_t run_threshold = 1400U;
  const uint32_t mid_low = 800U;         /* Mid position lower bound */
  const uint32_t mid_high = 1200U;       /* Mid position upper bound */
  
  if (s_io_pump_state.initialized == 0U)
  {
    return;
  }
  if (g_ethernet_rx_message_flag == 0U)
  {
  /* Get current SBUS channels */
  channel_count = SbusDriver_GetChannels(channels, 18U, &frame_lost, &failsafe, &last_update_tick);
  
  if (channel_count < 18U || frame_lost != 0U || failsafe != 0U)
  {
    /* No valid RC signal: keep speed bits low/stopped and STP high. */
    IoControlPump_SetSpeed(0U);
    IoControlPump_SetDirection(0U);
    g_io_pump_debug.right_outer_switch = 0U;
    g_io_pump_debug.left_mid2_switch = 0U;
    g_io_pump_debug.speed_triggered = 0U;
    g_io_pump_debug.direction_reversed = 0U;
    return;
  }

  /* Get right outer switch (CH7) - rightmost switch */
  right_outer_switch = channels[7U];  /* SBUS_CH_SWITCH_RIGHT_OUTER */
  
  /* Get left mid-2 switch (CH6) - second from right */
  left_mid2_switch = channels[6U];  /* SBUS_CH_SWITCH_LEFT_MID_2 */

  /* Update debug info with raw channel values */
  g_io_pump_debug.right_outer_switch = right_outer_switch;
  g_io_pump_debug.left_mid2_switch = left_mid2_switch;

  /* Speed control: right outer switch RUN side drives speed bit0 HIGH (PC13). */
  if (right_outer_switch > run_threshold)
  {
    IoControlPump_SetSpeed(1U);
    g_io_pump_debug.speed_triggered = 1U;
  }
  else
  {
    IoControlPump_SetSpeed(0U);
    g_io_pump_debug.speed_triggered = 0U;
  }

  /* Direction control: left-mid-2 switch in MIDDLE position (~1024) drives DIR low (PE4). */
  if (left_mid2_switch > mid_low && left_mid2_switch < mid_high)
  {
    IoControlPump_SetDirection(1U);
    g_io_pump_debug.direction_reversed = 1U;
    if(g_displacement_sensor.displacement_mm < 48.0f)
    {
      /* If currently extended, reverse direction to retract. */
      IoControlPump_SetSpeed(0U);
      g_io_pump_debug.speed_triggered = 0U;
    }
   
  }
  else
  {
    IoControlPump_SetDirection(0U);
    g_io_pump_debug.direction_reversed = 0U;
     if(g_displacement_sensor.displacement_mm > 130.0f)
    {
      /* If currently extended, reverse direction to retract. */
      IoControlPump_SetSpeed(0U);
      g_io_pump_debug.speed_triggered = 0U;
    }
  }
  }
  else
  {
    g_io_pump_debug.right_outer_switch = 0U;
    g_io_pump_debug.left_mid2_switch = 0U;

    if (g_ethernet_uart_readout_pack.pump_on_off == 1U)
  {
    IoControlPump_SetSpeed(1U);
    g_io_pump_debug.speed_triggered = 1U;
  }
  else
  {
    IoControlPump_SetSpeed(0U);
    g_io_pump_debug.speed_triggered = 0U;
  }

  if (g_ethernet_uart_readout_pack.pump_direction == 1U)
  {
    IoControlPump_SetDirection(1U);
    g_io_pump_debug.direction_reversed = 1U;
    if(g_displacement_sensor.displacement_mm < 48.0f)
    {
      /* If currently extended, reverse direction to retract. */
      IoControlPump_SetSpeed(0U);
      g_io_pump_debug.speed_triggered = 0U;
    }
   
  }
  else
  {
    IoControlPump_SetDirection(0U);
    g_io_pump_debug.direction_reversed = 0U;
     if(g_displacement_sensor.displacement_mm > 130.0f)
    {
      /* If currently extended, reverse direction to retract. */
      IoControlPump_SetSpeed(0U);
      g_io_pump_debug.speed_triggered = 0U;
    }
  }
  }
}

