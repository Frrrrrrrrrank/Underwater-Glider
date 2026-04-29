#include "pump_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

#include <string.h>

/* --- Displacement sensor internals (migrated here so pump_driver is the single UART3 owner) --- */

extern UART_HandleTypeDef huart3;

static void PumpDriver_BusInit(void);
static uint32_t PumpDriver_BusLock(uint32_t timeout_ms);
static void PumpDriver_BusUnlock(void);

/* Constants */
#define DISPLACEMENT_SENSOR_ID          0x02U
#define DISPLACEMENT_SENSOR_TIMEOUT_MS  100U
#define DISPLACEMENT_SENSOR_UPDATE_MS   100U
#define MODBUS_FUNC_READ_HOLDING_REGS   0x03U

typedef struct
{
  float displacement_mm;
  uint8_t sensor_id;
  uint32_t valid;
} DisplacementSensorData;

typedef struct
{
  uint32_t last_error;
  uint32_t comm_ok;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint16_t last_rx_len;
  uint8_t last_rx_frame[40];
  uint16_t last_tx_len;
  uint8_t last_tx_frame[8];
  uint32_t uart_rx_state;
  uint32_t uart_error_code;
} DisplacementSensorDiag;

volatile DisplacementSensorData g_displacement_sensor = {0.0f, DISPLACEMENT_SENSOR_ID, 0U};
volatile DisplacementSensorDiag g_displacement_diag = {0U,0U,0U,0U,0U,0U,0U,{0},0U,{0},0U,0U};

typedef struct
{
  uint32_t initialized;
  uint32_t comm_ok;
  uint32_t last_error;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint8_t rx_buffer[256];
  uint16_t rx_data_len;
} DisplacementSensorInternal;

static DisplacementSensorInternal s_disp_internal = {0U,0U,0U,0U,0U,0U,0U,{0},0U};

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

static uint16_t Displacement_BuildReadFrame(uint8_t *frame)
{
  uint16_t crc;

  frame[0] = DISPLACEMENT_SENSOR_ID;
  frame[1] = MODBUS_FUNC_READ_HOLDING_REGS;
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = 0x00;
  frame[5] = 0x10;

  crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

static uint8_t Displacement_ParseResponse(const uint8_t *frame, uint16_t frame_len)
{
  uint16_t recv_crc;
  uint16_t calc_crc;
  uint16_t reg8_high;
  uint16_t reg9_low;
  uint32_t bits;
  float displacement;

  if (frame == NULL || frame_len < 37U)
  {
    s_disp_internal.last_error = 1U;
    return 0U;
  }

  if (frame[0] != DISPLACEMENT_SENSOR_ID)
  {
    s_disp_internal.last_error = 2U;
    return 0U;
  }

  if (frame[1] != MODBUS_FUNC_READ_HOLDING_REGS)
  {
    s_disp_internal.last_error = 3U;
    return 0U;
  }

  if (frame[2] != 32U)
  {
    s_disp_internal.last_error = 4U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)frame[36] << 8) | frame[35]);
  calc_crc = Modbus_CRC16(frame, 35U);
  if (recv_crc != calc_crc)
  {
    s_disp_internal.crc_error_count++;
    s_disp_internal.last_error = 5U;
    return 0U;
  }

  reg8_high = (uint16_t)(((uint16_t)frame[19] << 8) | frame[20]);
  reg9_low  = (uint16_t)(((uint16_t)frame[21] << 8) | frame[22]);
  bits = ((uint32_t)reg8_high << 16) | reg9_low;
  memcpy(&displacement, &bits, sizeof(float));

  g_displacement_sensor.displacement_mm = displacement;
  g_displacement_sensor.sensor_id = frame[0];
  g_displacement_sensor.valid = 1U;
  g_pump_driver_status.displacement.last_seen_tick = osKernelGetTickCount();

  s_disp_internal.last_error = 0U;
  s_disp_internal.comm_ok = 1U;
  s_disp_internal.rx_count++;
  return 1U;
}

static void Displacement_Poll(uint32_t now_ms)
{
  uint8_t tx_frame[8] = {0};
  uint8_t rx_frame[40] = {0};
  uint16_t tx_len;
  static uint32_t last_send_tick = 0U;
  uint8_t bus_locked = 0U;

  if ((now_ms - last_send_tick) < DISPLACEMENT_SENSOR_UPDATE_MS)
  {
    return;
  }
  last_send_tick = now_ms;

  tx_len = Displacement_BuildReadFrame(tx_frame);
  g_displacement_diag.last_tx_len = tx_len;
  memcpy((void *)g_displacement_diag.last_tx_frame, tx_frame, tx_len);

  if (PumpDriver_BusLock(DISPLACEMENT_SENSOR_TIMEOUT_MS + 20U) == 0U)
  {
    s_disp_internal.comm_ok = 0U;
    s_disp_internal.last_error = 8U;
    g_displacement_sensor.valid = 0U;
    return;
  }
  bus_locked = 1U;

  if (HAL_UART_Transmit(&huart3, tx_frame, tx_len, 50U) == HAL_OK)
  {
    s_disp_internal.tx_count++;

    osDelay(2U);

    if (HAL_UART_Receive(&huart3, rx_frame, 37U, DISPLACEMENT_SENSOR_TIMEOUT_MS) == HAL_OK)
    {
      s_disp_internal.rx_data_len = 37U;
      memcpy(s_disp_internal.rx_buffer, rx_frame, 37U);
      (void)Displacement_ParseResponse(s_disp_internal.rx_buffer, s_disp_internal.rx_data_len);
    }
    else
    {
      s_disp_internal.timeout_count++;
      s_disp_internal.comm_ok = 0U;
      s_disp_internal.last_error = 6U;
      g_displacement_sensor.valid = 0U;
    }
  }
  else
  {
    s_disp_internal.comm_ok = 0U;
    s_disp_internal.last_error = 7U;
    g_displacement_sensor.valid = 0U;
  }

  if (bus_locked != 0U)
  {
    PumpDriver_BusUnlock();
  }
}

/* --- end displacement internals --- */

static UART_HandleTypeDef *s_huart = NULL;
volatile PumpMotorUart3Diag g_pump_motor_uart3_diag = {0};
volatile PumpDriverStatus g_pump_driver_status = {0};

/* Inline UART3 bus guard so pump_driver is the single owner of UART3 resources */
static osMutexId_t s_pump_bus_mutex = NULL;
static const osMutexAttr_t s_pump_bus_mutex_attr = {
  .name = "pumpUart3Bus"
};

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

void PumpDriver_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  PumpDriver_BusInit();
  g_pump_motor_uart3_diag.initialized = 1U;
  g_pump_motor_uart3_diag.node_id = PUMP_MOTOR_UART3_DEFAULT_NODE_ID;
  g_pump_driver_status.motor.initialized = 1U;
  g_pump_driver_status.motor.node_id = PUMP_MOTOR_UART3_DEFAULT_NODE_ID;
}

static uint16_t PumpDriver_BuildReadHoldingReg(uint8_t node_id, uint16_t reg_addr, uint8_t *frame)
{
  uint16_t crc;

  frame[0] = node_id;
  frame[1] = MODBUS_FUNC_READ_HOLDING_REGS;
  frame[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
  frame[3] = (uint8_t)(reg_addr & 0xFFU);
  frame[4] = 0x00U;
  frame[5] = 0x01U;

  crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

static uint8_t PumpDriver_ModbusReadHoldingReg(uint16_t reg_addr, uint16_t *out_value)
{
  uint8_t tx_frame[8];
  uint8_t rx_frame[7];
  uint16_t tx_len;
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (s_huart == NULL || out_value == NULL)
  {
    g_pump_motor_uart3_diag.last_error = 1U;
    return 0U;
  }

  tx_len = PumpDriver_BuildReadHoldingReg(g_pump_motor_uart3_diag.node_id, reg_addr, tx_frame);

  if (PumpDriver_BusLock(PUMP_MOTOR_UART3_TX_TIMEOUT_MS + PUMP_MOTOR_UART3_RX_TIMEOUT_MS + 10U) == 0U)
  {
    g_pump_motor_uart3_diag.last_error = 21U;
    return 0U;
  }

  if (HAL_UART_Transmit(s_huart, tx_frame, tx_len, PUMP_MOTOR_UART3_TX_TIMEOUT_MS) != HAL_OK)
  {
    g_pump_motor_uart3_diag.last_error = 2U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  g_pump_motor_uart3_diag.tx_count++;
  memcpy((void *)g_pump_motor_uart3_diag.last_tx_frame, tx_frame, sizeof(tx_frame));

  if (HAL_UART_Receive(s_huart, rx_frame, sizeof(rx_frame), PUMP_MOTOR_UART3_RX_TIMEOUT_MS) != HAL_OK)
  {
    g_pump_motor_uart3_diag.timeout_count++;
    g_pump_motor_uart3_diag.last_error = 3U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  memcpy((void *)g_pump_motor_uart3_diag.last_rx_frame, rx_frame, sizeof(rx_frame));
  g_pump_motor_uart3_diag.rx_count++;

  if (rx_frame[0] != g_pump_motor_uart3_diag.node_id)
  {
    g_pump_motor_uart3_diag.last_error = 4U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  if (rx_frame[1] == (uint8_t)(MODBUS_FUNC_READ_HOLDING_REGS | 0x80U))
  {
    g_pump_motor_uart3_diag.last_error = 6U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  if (rx_frame[1] != MODBUS_FUNC_READ_HOLDING_REGS || rx_frame[2] != 2U)
  {
    g_pump_motor_uart3_diag.last_error = 4U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[6] << 8) | rx_frame[5]);
  calc_crc = Modbus_CRC16(rx_frame, 5U);
  if (recv_crc != calc_crc)
  {
    g_pump_motor_uart3_diag.last_error = 5U;
    PumpDriver_BusUnlock();
    return 0U;
  }

  *out_value = (uint16_t)(((uint16_t)rx_frame[3] << 8) | rx_frame[4]);
  g_pump_motor_uart3_diag.last_error = 0U;

  PumpDriver_BusUnlock();
  return 1U;
}

static void PumpDriver_SyncStatus(uint32_t now_ms)
{
  g_pump_driver_status.displacement.valid = g_displacement_sensor.valid;
  g_pump_driver_status.displacement.sensor_id = g_displacement_sensor.sensor_id;
  g_pump_driver_status.displacement.displacement_mm = g_displacement_sensor.displacement_mm;
  g_pump_driver_status.displacement.tx_count = g_displacement_diag.tx_count;
  g_pump_driver_status.displacement.rx_count = g_displacement_diag.rx_count;
  g_pump_driver_status.displacement.timeout_count = g_displacement_diag.timeout_count;
  g_pump_driver_status.displacement.crc_error_count = g_displacement_diag.crc_error_count;
  g_pump_driver_status.displacement.last_error = g_displacement_diag.last_error;
  g_pump_driver_status.displacement.online =
    (g_pump_driver_status.displacement.last_seen_tick != 0U &&
     (now_ms - g_pump_driver_status.displacement.last_seen_tick) < 500U) ? 1U : 0U;

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
  /* Displacement polling temporarily disabled to isolate RS485 traffic. */
  /* Displacement_Poll(now_ms); */

  /* Then poll new pump motor on UART3 - Modbus RTU read holding regs */
  static uint32_t last_motor_poll = 0U;
  static uint8_t next_cmd = 0U; /* 0=Status, 1=Alarm */
  uint16_t parsed_value = 0U;

  if ((now_ms - last_motor_poll) < PUMP_MOTOR_UART3_POLL_PERIOD_MS)
  {
    PumpDriver_SyncStatus(now_ms);
    return;
  }
  last_motor_poll = now_ms;

  if (next_cmd == 0U)
  {
    if (PumpDriver_ModbusReadHoldingReg(0x0001U, &parsed_value) != 0U)
    {
      g_pump_motor_uart3_diag.status_code = parsed_value;
      g_pump_driver_status.motor.status_code = parsed_value;
      g_pump_motor_uart3_diag.valid = 1U;
      g_pump_driver_status.motor.last_seen_tick = now_ms;
    }
    next_cmd = 1U;
  }
  else
  {
    if (PumpDriver_ModbusReadHoldingReg(0x0000U, &parsed_value) != 0U)
    {
      g_pump_motor_uart3_diag.alarm_code = parsed_value;
      g_pump_driver_status.motor.alarm_code = parsed_value;
      g_pump_driver_status.motor.last_seen_tick = now_ms;
    }
    next_cmd = 0U;
  }

  PumpDriver_SyncStatus(now_ms);
}
