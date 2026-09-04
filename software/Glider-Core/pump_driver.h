#ifndef PUMP_DRIVER_H
#define PUMP_DRIVER_H

#include "stm32h7xx_hal.h"

typedef struct
{
  uint8_t initialized;
  uint8_t node_id;
  uint8_t valid;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint8_t last_rx_frame[64];
  uint8_t last_tx_frame[16];
  uint8_t last_error;
  uint16_t status_code;
  uint16_t alarm_code;
  int32_t feedback_speed_rpm;
} PumpMotorUart3Diag;

typedef struct
{
  uint8_t online;
  uint8_t valid;
  uint8_t sensor_id;
  float displacement_mm;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t last_error;
  uint32_t last_seen_tick;
} PumpDisplacementStatus;

typedef struct
{
  uint8_t online;
  uint8_t initialized;
  uint8_t node_id;
  uint8_t valid;
  int32_t feedback_speed_rpm;
  uint32_t tx_count;
  uint32_t rx_count;
  uint16_t status_code;
  uint16_t alarm_code;
  uint32_t timeout_count;
  uint32_t last_error;
  uint32_t last_seen_tick;
} PumpMotorStatus;

typedef struct
{
  PumpDisplacementStatus displacement;
  PumpMotorStatus motor;
} PumpDriverStatus;

#define PUMP_MOTOR_UART3_DEFAULT_NODE_ID 0x01U
#define PUMP_MOTOR_UART3_TX_TIMEOUT_MS 50U
#define PUMP_MOTOR_UART3_RX_TIMEOUT_MS 200U
#define PUMP_MOTOR_UART3_POLL_PERIOD_MS 1000U

/* IO Control pins */
#define IO_PUMP_SPEED_PORT GPIOC
#define IO_PUMP_SPEED_PIN GPIO_PIN_13
#define IO_PUMP_SPEED_B1_PORT GPIOC
#define IO_PUMP_SPEED_B1_PIN GPIO_PIN_14
#define IO_PUMP_STP_PORT GPIOC
#define IO_PUMP_STP_PIN GPIO_PIN_15
#define IO_PUMP_DIR_PORT GPIOE
#define IO_PUMP_DIR_PIN GPIO_PIN_4

#define IO_PUMP_COMMAND_TIMEOUT_MS 100U
#define IO_PUMP_DIRECTION_GUARD_MS 100U
#define IO_PUMP_DISPLACEMENT_MIN_MM 50.0f //泵限位值
#define IO_PUMP_DISPLACEMENT_MAX_MM 130.0f
#define IO_PUMP_DISPLACEMENT_DEADZONE_MM 2.0f //  dead zone，不操作

typedef enum
{
  IO_PUMP_SOURCE_OFF = 0U,
  IO_PUMP_SOURCE_SBUS = 1U,
  IO_PUMP_SOURCE_DIRECT_ETH = 2U,
  IO_PUMP_SOURCE_PID_ETH = 3U
} IoControlPumpSource;

typedef enum
{
  IO_PUMP_INTERLOCK_NONE = 0U,
  IO_PUMP_INTERLOCK_COMMAND_TIMEOUT = 1U,
  IO_PUMP_INTERLOCK_DISPLACEMENT_INVALID = 2U,
  IO_PUMP_INTERLOCK_MIN_LIMIT = 3U,
  IO_PUMP_INTERLOCK_MAX_LIMIT = 4U,
  IO_PUMP_INTERLOCK_DIRECTION_GUARD = 5U
} IoControlPumpInterlock;

typedef struct
{
  IoControlPumpSource source;
  uint8_t enabled;
  uint8_t direction_reversed;
  uint32_t update_tick_ms;
  uint32_t sequence;
} IoControlPumpCommand;

/* IO Control debug info */
typedef struct
{
  uint16_t right_outer_switch;    /* CH7 raw value */
  uint16_t left_mid2_switch;      /* CH6 raw value */
  uint8_t speed_triggered;        /* 1 when speed control active */
  uint8_t direction_reversed;     /* 1 when direction reversed */
  uint8_t command_source;
  uint8_t command_valid;
  uint8_t interlock_reason;
  uint8_t direction_guard_active;
  uint32_t command_age_ms;
} IoControlPumpDebugInfo;

extern volatile IoControlPumpDebugInfo g_io_pump_debug;

extern volatile PumpMotorUart3Diag g_pump_motor_uart3_diag;
extern volatile PumpDriverStatus g_pump_driver_status;

void PumpDriver_Init(UART_HandleTypeDef *huart);
void PumpDriver_Poll(uint32_t now_ms);

/* 高级控制接口 */
uint8_t PumpDriver_Start(void);
uint8_t PumpDriver_Stop(void);
uint8_t PumpDriver_SetSpeed(uint16_t speed_240rps);  /* 单位: 1/240 RPS */
uint8_t PumpDriver_ReadSpeed(int32_t *out_speed_rpm); /* 输出单位: RPM */

/* IO 控制接口 */
void IoControlPump_Init(void);
/* ServoTask 提交解析后的唯一命令；本函数不写 GPIO。 */
uint8_t IoControlPump_SubmitCommand(
    IoControlPumpSource source,
    uint8_t enabled,
    uint8_t direction_reversed,
    uint32_t now_ms);
/* PumpTask 唯一调用者：执行超时、位移限位和换向保护后写 GPIO。 */
void IoControlPump_Poll(void);

#endif
