#ifndef PRESSURE_SENSOR_DRIVER_H
#define PRESSURE_SENSOR_DRIVER_H

/* Select the sensor here; all I2C variants use the existing hi2c2.
 * Init keeps its UART argument for source compatibility; I2C variants ignore it. */
#define PRESSURE_SENSOR_TYPE_MODBUS_RS485  1U
#define PRESSURE_SENSOR_TYPE_KELLER_I2C    2U
#define PRESSURE_SENSOR_TYPE_MS5837_30BA   3U
#ifndef PRESSURE_SENSOR_TYPE
#define PRESSURE_SENSOR_TYPE PRESSURE_SENSOR_TYPE_MS5837_30BA
#endif

#if (PRESSURE_SENSOR_TYPE != PRESSURE_SENSOR_TYPE_MODBUS_RS485) && \
    (PRESSURE_SENSOR_TYPE != PRESSURE_SENSOR_TYPE_KELLER_I2C) && \
    (PRESSURE_SENSOR_TYPE != PRESSURE_SENSOR_TYPE_MS5837_30BA)
#error "Unsupported PRESSURE_SENSOR_TYPE"
#endif

/* Retained Keller D-Line configuration from ver1.1. */
#define PRESSURE_SENSOR_KELLER_I2C_ADDRESS 0x40U
#define PRESSURE_SENSOR_KELLER_SCAN_ADDRESS 0U
#define PRESSURE_SENSOR_KELLER_PRESSURE_MIN_BAR 0.0f
#define PRESSURE_SENSOR_KELLER_PRESSURE_MAX_BAR 3.0f

/* MS5837-30BA only (NOT the 02BA): fixed 7-bit address, shifted in HAL calls.
 * OSR 8192: datasheet maximum conversion time 18.08 ms, rounded up to 20 ms.
 * Poll every 100 ms yields one pressure/temperature pair about every 200 ms.
 * pressure_bar is ABSOLUTE pressure, about 1 bar in air. Depth zeroing is
 * performed by the existing EKF bridge; no atmospheric offset is subtracted here.
 * PB10=SCL, PB11=SDA; sensor power and bus pull-ups must use 3.3 V. */
#define PRESSURE_SENSOR_MS5837_I2C_ADDRESS 0x76U
#define PRESSURE_SENSOR_MS5837_CONVERSION_MS 20U
#define PRESSURE_SENSOR_MS5837_I2C_TIMEOUT_MS 3U
#define PRESSURE_SENSOR_MS5837_RETRY_MS 1000U
#define PRESSURE_SENSOR_MS5837_STALE_MS 1000U

/* MS5837 last_error codes; driver_state uses the states below. */
#define PRESSURE_SENSOR_MS5837_ERROR_I2C 30U
#define PRESSURE_SENSOR_MS5837_ERROR_PROM_CRC 31U
#define PRESSURE_SENSOR_MS5837_ERROR_PROM_DATA 32U
#define PRESSURE_SENSOR_MS5837_ERROR_ADC 33U
#define PRESSURE_SENSOR_MS5837_ERROR_RANGE 34U
#define PRESSURE_SENSOR_MS5837_ERROR_STALE 35U
#define PRESSURE_SENSOR_MS5837_STATE_RESET 0U
#define PRESSURE_SENSOR_MS5837_STATE_RESET_WAIT 1U
#define PRESSURE_SENSOR_MS5837_STATE_PROM 2U
#define PRESSURE_SENSOR_MS5837_STATE_D1_START 3U
#define PRESSURE_SENSOR_MS5837_STATE_D1_WAIT 4U
#define PRESSURE_SENSOR_MS5837_STATE_D2_WAIT 5U
#define PRESSURE_SENSOR_MS5837_STATE_RETRY 6U

#include "main.h"
#include <stdint.h>

#define PRESSURE_SENSOR_RX_RING_SIZE 512U
/* 调用 PressureSensorDriver_Poll 的任务为 osDelay(100)，Modbus 请求约 10Hz。
 * 当前驱动交替请求压力和温度，因此两类有效新样本各约 5Hz。pressure_update_seq
 * 与 temperature_update_seq 必须分开：温度响应不能被 EKF 误判成新压力。
 * 若要提高深度外环频率，应单独调整请求调度，不能只缩短本门限。 */
#define PRESSURE_SENSOR_MODBUS_REQUEST_PERIOD_MS 90U
/* 响应超时必须 < 请求周期；Keller @115200 实际响应仅几 ms，60ms 留 10× 余量 */
#define PRESSURE_SENSOR_MODBUS_RESPONSE_TIMEOUT_MS 60U
#define PRESSURE_SENSOR_MODBUS_DEFAULT_ADDR 250U

typedef struct
{
  uint32_t initialized;
  uint32_t slave_address;
  uint32_t last_error;
  uint32_t uart_error_code;

  uint32_t tx_in_flight;
  uint32_t response_waiting;

  uint32_t rx_irq_count;
  uint32_t rx_overflow_count;

  uint32_t request_count;
  uint32_t response_ok_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t exception_count;
  uint32_t last_exception_code;

  uint32_t last_request_tick;
  uint32_t last_response_tick;

  uint32_t sensor_type;
  uint32_t i2c_error_code;
  uint32_t last_hal_status;
  uint32_t driver_state;
  uint32_t status_byte;
  uint32_t pressure_raw;
  uint32_t temperature_raw;
  uint32_t scaling_valid;
  uint32_t probe_address;
  uint32_t probe_count;
  uint32_t address_ack_count;
  uint32_t i2c_fail_count;
  uint32_t reset_count;
  uint32_t prom_crc_expected;
  uint32_t prom_crc_computed;
  uint16_t prom[7];
} PressureSensorDiag;

typedef struct
{
  uint32_t last_irq_byte;
  uint32_t ring_available_bytes;

  uint32_t pressure_valid;
  float pressure_bar;
  uint32_t pressure_update_tick;
  uint32_t pressure_update_seq;
  uint32_t temperature_valid;
  float temperature_c;
  uint32_t temperature_update_tick;
  uint32_t temperature_update_seq;
  /* 任意一种测量更新的时刻，仅保留作通用诊断，闭环不得用它判新压力。 */
  uint32_t last_update_tick;

  uint8_t last_tx_data[16];
  uint32_t last_response_len;
  uint8_t last_response_data[16];
  float pressure_min_bar;
  float pressure_max_bar;
} PressureSensorReadout;

typedef struct
{
  PressureSensorDiag diag;
  PressureSensorReadout readout;
} PressureSensorState;

extern volatile PressureSensorState g_pressure_sensor_state;

#define g_pressure_sensor_diag g_pressure_sensor_state.diag
#define g_pressure_sensor_readout g_pressure_sensor_state.readout

void PressureSensorDriver_Init(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *huart);
void PressureSensorDriver_OnError(UART_HandleTypeDef *huart);
void PressureSensorDriver_Poll(uint32_t now_ms);

uint16_t PressureSensorDriver_Read(uint8_t *dst, uint16_t max_len);
uint16_t PressureSensorDriver_Available(void);
void PressureSensorDriver_ClearRxBuffer(void);

#endif /* PRESSURE_SENSOR_DRIVER_H */
