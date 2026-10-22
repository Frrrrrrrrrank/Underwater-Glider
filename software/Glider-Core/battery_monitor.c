#include "battery_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include <string.h>

/* 默认参考电压只是标称值；实机需用万用表标定增益和偏移。 */
#ifndef BATTERY_VREF_MV
#define BATTERY_VREF_MV          (3300U)
#endif
#ifndef BATTERY_GAIN_PPM
#define BATTERY_GAIN_PPM         (1000000U)
#endif
#ifndef BATTERY_OFFSET_MV
#define BATTERY_OFFSET_MV        (0)
#endif
#define BATTERY_R_TOP_OHM        (100000U)
#define BATTERY_R_BOTTOM_OHM     (4700U)
#define BATTERY_ADC_FULL_SCALE   (65535U)
#define BATTERY_BATCH_SIZE       (8U)
#define BATTERY_POLL_TIMEOUT_MS  (2U)
#define BATTERY_CALIB_RETRY_MS   (1000U)
#define BATTERY_WARN_MV          (22000U)
#define BATTERY_CRITICAL_MV      (21000U)
#define BATTERY_WARN_CLEAR_MV    (22500U)
#define BATTERY_CRIT_CLEAR_MV    (21500U)
#define BATTERY_WARN_HOLD_MS     (2000U)
#define BATTERY_CRIT_HOLD_MS     (1000U)
#define BATTERY_CLEAR_HOLD_MS    (5000U)

typedef struct
{
  uint32_t since_ms;
  uint8_t active;
} BatteryTimer;

volatile BatteryState g_battery_state;
static BatteryState s_state;
static ADC_HandleTypeDef *s_adc;
static BatteryMonitorState s_alarm_state;
static BatteryTimer s_warn_timer;
static BatteryTimer s_critical_timer;
static BatteryTimer s_clear_timer;
static uint32_t s_last_calibration_ms;
static uint32_t s_filtered_q8;
static uint8_t s_calibrated;
static uint8_t s_calibration_attempted;
static uint8_t s_filter_ready;

static uint32_t Battery_Increment(uint32_t value)
{
  return (value == UINT32_MAX) ? value : value + 1U;
}

static void Battery_Publish(void)
{
  /* 临界区只复制快照；ADC 等待、校准和换算均在外面执行。 */
  taskENTER_CRITICAL();
  g_battery_state = s_state;
  taskEXIT_CRITICAL();
}

static void Battery_ResetEvidence(void)
{
  memset(&s_warn_timer, 0, sizeof(s_warn_timer));
  memset(&s_critical_timer, 0, sizeof(s_critical_timer));
  memset(&s_clear_timer, 0, sizeof(s_clear_timer));
  s_filter_ready = 0U;
  /* 已确认的低压不能被采样故障清掉；正常状态须重新由有效样本建立。 */
  if (s_alarm_state < BATTERY_STATE_WARN)
  {
    s_alarm_state = BATTERY_STATE_INIT;
  }
}

static void Battery_RecordError(HAL_StatusTypeDef status)
{
  Battery_ResetEvidence();
  s_state.valid = 0U;
  s_state.state = BATTERY_STATE_INVALID;
  s_state.last_hal_status = (uint32_t)status;
  s_state.error_count = Battery_Increment(s_state.error_count);
}

static bool Battery_Held(BatteryTimer *timer, bool condition,
                         uint32_t now_ms, uint32_t duration_ms)
{
  if (!condition)
  {
    timer->active = 0U;
    return false;
  }
  if (timer->active == 0U)
  {
    timer->active = 1U;
    timer->since_ms = now_ms;
  }
  /* 无符号差值覆盖 HAL 毫秒计数回绕，不以时间戳 0 充当未启动标志。 */
  return (uint32_t)(now_ms - timer->since_ms) >= duration_ms;
}

static uint32_t Battery_RawToMv(uint32_t raw)
{
  const uint64_t denominator = (uint64_t)BATTERY_ADC_FULL_SCALE * BATTERY_R_BOTTOM_OHM;
  uint64_t numerator = (uint64_t)raw * BATTERY_VREF_MV *
                       (BATTERY_R_TOP_OHM + BATTERY_R_BOTTOM_OHM);
  uint64_t mv = (numerator + denominator / 2U) / denominator;
  int64_t corrected = (int64_t)((mv * BATTERY_GAIN_PPM + 500000U) / 1000000U) +
                      (int64_t)BATTERY_OFFSET_MV;
  if (corrected < 0)
  {
    return 0U;
  }
  /* 保证后续 Q8 滤波乘法也不会溢出。默认硬件量程不足 74V。 */
  if (corrected > (int64_t)(UINT32_MAX >> 8))
  {
    return UINT32_MAX >> 8;
  }
  return (uint32_t)corrected;
}

static void Battery_UpdateAlarm(uint32_t mv, uint32_t now_ms)
{
  bool warning = Battery_Held(&s_warn_timer, mv <= BATTERY_WARN_MV,
                              now_ms, BATTERY_WARN_HOLD_MS);
  bool critical = Battery_Held(&s_critical_timer, mv <= BATTERY_CRITICAL_MV,
                               now_ms, BATTERY_CRIT_HOLD_MS);

  if (s_alarm_state == BATTERY_STATE_CRITICAL)
  {
    if (Battery_Held(&s_clear_timer, mv >= BATTERY_CRIT_CLEAR_MV,
                     now_ms, BATTERY_CLEAR_HOLD_MS))
    {
      s_alarm_state = BATTERY_STATE_WARN;
      s_clear_timer.active = 0U;
      /* 即使电压直接恢复很高，也必须先降为预警，再另计五秒才能正常。 */
      (void)Battery_Held(&s_clear_timer, mv >= BATTERY_WARN_CLEAR_MV,
                         now_ms, BATTERY_CLEAR_HOLD_MS);
    }
  }
  else if (critical)
  {
    s_alarm_state = BATTERY_STATE_CRITICAL;
    s_clear_timer.active = 0U;
  }
  else if (s_alarm_state == BATTERY_STATE_WARN)
  {
    if (Battery_Held(&s_clear_timer, mv >= BATTERY_WARN_CLEAR_MV,
                     now_ms, BATTERY_CLEAR_HOLD_MS))
    {
      s_alarm_state = BATTERY_STATE_NORMAL;
      s_clear_timer.active = 0U;
    }
  }
  else if (warning)
  {
    s_alarm_state = BATTERY_STATE_WARN;
    s_clear_timer.active = 0U;
  }
  else if (s_alarm_state == BATTERY_STATE_INIT && mv > BATTERY_WARN_MV)
  {
    s_alarm_state = BATTERY_STATE_NORMAL;
  }
  s_state.state = s_alarm_state;
}

HAL_StatusTypeDef BatteryMonitor_Init(ADC_HandleTypeDef *hadc)
{
  HAL_StatusTypeDef status = HAL_OK;
  memset(&s_state, 0, sizeof(s_state));
  s_adc = NULL;
  s_alarm_state = BATTERY_STATE_INIT;
  s_calibrated = 0U;
  s_calibration_attempted = 0U;
  Battery_ResetEvidence();
  s_last_calibration_ms = 0U;
  if (hadc == NULL || hadc->Instance != ADC1)
  {
    status = HAL_ERROR;
  }
  else
  {
    s_adc = hadc;
  }
  if (status != HAL_OK)
  {
    Battery_RecordError(status);
  }
  /* 初始化发生在调度器启动前，不调用 FreeRTOS 临界区。 */
  g_battery_state = s_state;
  return status;
}

void BatteryMonitor_Poll(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t sum = 0U;
  HAL_StatusTypeDef status;

  if (s_adc == NULL)
  {
    return;
  }
  if (s_calibrated == 0U)
  {
    if (s_calibration_attempted != 0U &&
        (uint32_t)(now_ms - s_last_calibration_ms) < BATTERY_CALIB_RETRY_MS)
    {
      return;
    }
    /* HAL 校准使用较长的内部循环超时，并非采样等待的 2ms。
     * 放在低优先级任务中保持控制/通信可抢占，失败后从返回时刻再等一秒。 */
    s_calibration_attempted = 1U;
    status = HAL_ADCEx_Calibration_Start(s_adc, ADC_CALIB_OFFSET_LINEARITY,
                                        ADC_SINGLE_ENDED);
    s_last_calibration_ms = HAL_GetTick();
    if (status != HAL_OK)
    {
      Battery_RecordError(status);
      Battery_Publish();
      return;
    }
    s_calibrated = 1U;
  }

  for (uint32_t index = 0U; index < BATTERY_BATCH_SIZE; ++index)
  {
    uint32_t raw = 0U;
    HAL_StatusTypeDef stop_status;
    status = HAL_ADC_Start(s_adc);
    if (status == HAL_OK)
    {
      status = HAL_ADC_PollForConversion(s_adc, BATTERY_POLL_TIMEOUT_MS);
      if (status == HAL_OK)
      {
        raw = HAL_ADC_GetValue(s_adc);
        /* 满量程无法区分真实电压与钳位/接线异常，作为测量故障处理。 */
        if (raw >= BATTERY_ADC_FULL_SCALE)
        {
          status = HAL_ERROR;
        }
      }
    }
    stop_status = HAL_ADC_Stop(s_adc);
    if (status == HAL_OK)
    {
      status = stop_status;
    }
    if (status != HAL_OK)
    {
      Battery_RecordError(status);
      Battery_Publish();
      return;
    }
    sum += raw;
  }

  now_ms = HAL_GetTick();
  if (s_state.sample_count != 0U &&
      (uint32_t)(now_ms - s_state.last_sample_ms) > BATTERY_STALE_MS)
  {
    Battery_ResetEvidence();
  }
  s_state.adc_raw = (sum + BATTERY_BATCH_SIZE / 2U) / BATTERY_BATCH_SIZE;
  s_state.battery_mv = Battery_RawToMv(s_state.adc_raw);
  if (s_filter_ready == 0U)
  {
    s_filtered_q8 = s_state.battery_mv << 8;
    s_filter_ready = 1U;
  }
  else
  {
    s_filtered_q8 = (uint32_t)(((uint64_t)s_filtered_q8 * 3U +
                                ((uint64_t)s_state.battery_mv << 8) + 2U) / 4U);
  }
  s_state.filtered_mv = (s_filtered_q8 + 128U) >> 8;
  s_state.last_sample_ms = now_ms;
  s_state.sample_count = Battery_Increment(s_state.sample_count);
  s_state.last_hal_status = HAL_OK;
  s_state.valid = 1U;
  Battery_UpdateAlarm(s_state.filtered_mv, now_ms);
  Battery_Publish();
}

bool BatteryMonitor_CopyState(BatteryState *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *out_state = g_battery_state;
  taskEXIT_CRITICAL();
  if (out_state->valid != 0U &&
      (uint32_t)(HAL_GetTick() - out_state->last_sample_ms) > BATTERY_STALE_MS)
  {
    out_state->valid = 0U;
    out_state->state = BATTERY_STATE_INVALID;
  }
  return true;
}
