#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BATTERY_SAMPLE_PERIOD_MS (100U)
#define BATTERY_STALE_MS         (500U)

typedef enum
{
  BATTERY_STATE_INIT = 0,
  BATTERY_STATE_NORMAL = 1,
  BATTERY_STATE_WARN = 2,
  BATTERY_STATE_CRITICAL = 3,
  BATTERY_STATE_INVALID = 4
} BatteryMonitorState;

typedef struct
{
  uint32_t adc_raw;             /* 一批八次转换的平均原码。 */
  uint32_t battery_mv;          /* 已按分压及标定参数换算的电池毫伏。 */
  uint32_t filtered_mv;         /* 告警判断使用的滤波毫伏。 */
  uint32_t last_sample_ms;      /* 最后一批成功采样的 HAL 毫秒时间。 */
  uint32_t sample_count;        /* 成功批次数；达到上限后饱和。 */
  uint32_t error_count;         /* 失败批次及校准失败计数。 */
  uint32_t last_hal_status;     /* 最近一次采样或校准的 HAL 返回值。 */
  BatteryMonitorState state;
  uint8_t valid;               /* 电压有效；INIT 仍可能在等待低压持续时间。 */
} BatteryState;

/* 仅供 Ozone 观察；业务读取必须使用整体快照接口。 */
extern volatile BatteryState g_battery_state;

/* 在调度器启动前、MX_ADC1_Init 之后校验句柄并初始化；硬件校准由首次 Poll 执行。 */
HAL_StatusTypeDef BatteryMonitor_Init(ADC_HandleTypeDef *hadc);

/* 仅由电池任务调用，独占 ADC1；不得从中断或其他任务并发调用。 */
void BatteryMonitor_Poll(void);

/* 任务上下文读取；超过 500ms 的电压在返回快照中标为无效。 */
bool BatteryMonitor_CopyState(BatteryState *out_state);

#ifdef __cplusplus
}
#endif

#endif
