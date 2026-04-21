#ifndef SBUS_DRIVER_H
#define SBUS_DRIVER_H

#include "main.h"
#include <stdint.h>

#define SBUS_FRAME_LEN 25U
#define SBUS_CHANNEL_COUNT 18U
#define SBUS_CHANNEL_VALUE_MAX 2047U
#define SBUS_VALUE_MIN_OBSERVED 240U
#define SBUS_VALUE_CENTER_OBSERVED 1024U
#define SBUS_VALUE_MAX_OBSERVED 1807U
#define SBUS_FRAME_FRESH_DEFAULT_MS 100U

typedef enum
{
  SBUS_CH_RIGHT_STICK_X = 0U,
  SBUS_CH_RIGHT_STICK_Y = 1U,
  SBUS_CH_LEFT_STICK_Y = 2U,
  SBUS_CH_LEFT_STICK_X = 3U,
  SBUS_CH_SWITCH_LEFT_OUTER = 4U,
  SBUS_CH_SWITCH_LEFT_MID_1 = 5U,
  SBUS_CH_SWITCH_LEFT_MID_2 = 6U,
  SBUS_CH_SWITCH_RIGHT_OUTER = 7U,
  SBUS_CH_WHEEL_LEFT = 8U,
  SBUS_CH_WHEEL_RIGHT = 9U
} SbusChannelIndex;

typedef struct
{
  uint16_t right_stick_x;
  uint16_t right_stick_y;
  uint16_t left_stick_y;
  uint16_t left_stick_x;
  uint16_t switch_left_outer;
  uint16_t switch_left_mid_1;
  uint16_t switch_left_mid_2;
  uint16_t switch_right_outer;
  uint16_t wheel_left;
  uint16_t wheel_right;
} SbusControlInputs;

typedef struct
{
  uint32_t initialized;
  uint32_t last_error;
  uint32_t uart_error_code;

  uint32_t rx_irq_count;
  uint32_t rx_rearm_ok_count;
  uint32_t rx_rearm_busy_count;
  uint32_t rx_rearm_fail_count;

  uint32_t start_byte_miss_count;
  uint32_t frame_ok_count;
  uint32_t frame_bad_count;

  uint32_t last_irq_tick;
} SbusDiag;

typedef struct
{
  uint16_t channels[SBUS_CHANNEL_COUNT];
  SbusControlInputs controls;
  uint32_t frame_lost;
  uint32_t failsafe;

  uint32_t frame_update_count;
  uint32_t last_update_tick;

  uint8_t last_frame[SBUS_FRAME_LEN];
} SbusReadout;

typedef struct
{
  SbusDiag diag;
  SbusReadout readout;
} SbusState;

extern volatile SbusState g_sbus_state;

#define g_sbus_diag g_sbus_state.diag
#define g_sbus_readout g_sbus_state.readout

void SbusDriver_Init(UART_HandleTypeDef *huart);
void SbusDriver_OnRxComplete(UART_HandleTypeDef *huart);
void SbusDriver_OnError(UART_HandleTypeDef *huart);

uint32_t SbusDriver_IsFrameFresh(uint32_t now_ms, uint32_t timeout_ms);
uint32_t SbusDriver_GetChannels(uint16_t *dst, uint32_t max_count,
                                uint32_t *frame_lost, uint32_t *failsafe,
                                uint32_t *last_update_tick);

#endif /* SBUS_DRIVER_H */
