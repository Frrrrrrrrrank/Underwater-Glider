#include "rudder_driver.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"

/* 定时器参数不匹配时拒绝启动，避免把计数值误当成微秒发送给舵机。 */
#define RUDDER_COUNTER_HZ 1000000U

_Static_assert(RUDDER_DIRECTION == 1 || RUDDER_DIRECTION == -1,
               "RUDDER_DIRECTION must be +1 or -1");
_Static_assert(RUDDER_LIMIT_CDEG > 0 && RUDDER_LIMIT_CDEG <= RUDDER_FULL_SCALE_CDEG,
               "Rudder mechanical limit exceeds nominal servo travel");
_Static_assert((int32_t)RUDDER_CENTER_PULSE_US + RUDDER_CENTER_TRIM_US >= (int32_t)RUDDER_MIN_PULSE_US &&
               (int32_t)RUDDER_CENTER_PULSE_US + RUDDER_CENTER_TRIM_US <= (int32_t)RUDDER_MAX_PULSE_US,
               "Rudder center trim exceeds pulse limits");

volatile RudderState g_rudder_state = {
  .applied_pulse_us = RUDDER_CENTER_PULSE_US + RUDDER_CENTER_TRIM_US
};

volatile RudderDebugCommand g_rudder_debug = {0};

static TIM_HandleTypeDef *s_rudder_tim = NULL;

static uint32_t Rudder_TimerClockHz(void)
{
  uint32_t hclk = HAL_RCC_GetHCLKFreq();
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t apb1_prescaler = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;

  if (hclk == 0U || pclk1 == 0U)
  {
    return 0U;
  }

  /* H743：TIMPRE=0 时 APB 分频 1/2 使用 HCLK，其余使用 2*PCLK1；
     TIMPRE=1 时分频 1/2/4 使用 HCLK，其余使用 4*PCLK1。 */
  if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U)
  {
    return (apb1_prescaler <= 5U) ? hclk : (pclk1 * 4U);
  }
  return (apb1_prescaler <= 4U) ? hclk : (pclk1 * 2U);
}

static uint16_t Rudder_AngleToPulse(int16_t angle_cdeg, uint8_t *limited)
{
  int32_t scaled = (int32_t)angle_cdeg * RUDDER_DIRECTION * 500;
  int32_t pulse;

  /* 正负方向对称四舍五入；500 us 对应名义 45 度轴角。 */
  scaled += (scaled >= 0) ? (RUDDER_FULL_SCALE_CDEG / 2) : -(RUDDER_FULL_SCALE_CDEG / 2);
  pulse = (int32_t)RUDDER_CENTER_PULSE_US + RUDDER_CENTER_TRIM_US +
          scaled / RUDDER_FULL_SCALE_CDEG;
  if (pulse < (int32_t)RUDDER_MIN_PULSE_US)
  {
    pulse = (int32_t)RUDDER_MIN_PULSE_US;
    *limited = 1U;
  }
  else if (pulse > (int32_t)RUDDER_MAX_PULSE_US)
  {
    pulse = (int32_t)RUDDER_MAX_PULSE_US;
    *limited = 1U;
  }
  return (uint16_t)pulse;
}

HAL_StatusTypeDef RudderDriver_Init(TIM_HandleTypeDef *htim)
{
  HAL_StatusTypeDef status;
  uint32_t timer_clock;
  uint32_t prescaler;

  if (s_rudder_tim != NULL)
  {
    return HAL_BUSY;
  }
  if (htim == NULL || htim->Instance != TIM4)
  {
    return HAL_ERROR;
  }

  timer_clock = Rudder_TimerClockHz();
  g_rudder_state.timer_clock_hz = timer_clock;
  prescaler = htim->Instance->PSC;
  if (timer_clock == 0U || prescaler > 65535U ||
      timer_clock % RUDDER_COUNTER_HZ != 0U ||
      timer_clock / RUDDER_COUNTER_HZ != prescaler + 1U ||
      htim->Instance->ARR != RUDDER_PERIOD_US - 1U ||
      (htim->Instance->CR1 & (TIM_CR1_CMS | TIM_CR1_DIR | TIM_CR1_CEN)) != 0U)
  {
    return HAL_ERROR;
  }

  /* CH4 的 CCR 使用预装载，运行期间改目标只在周期边界生效，避免截短脉冲。
     初始化先写中位并产生更新事件，再启用通道，第一帧就是完整中位脉冲。 */
  SET_BIT(htim->Instance->CCMR2, TIM_CCMR2_OC4PE);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, g_rudder_state.applied_pulse_us);
  __HAL_TIM_SET_COUNTER(htim, 0U);
  htim->Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
  status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 此时尚未启动调度器，不调用 taskENTER_CRITICAL，避免改变启动期屏蔽状态。 */
  s_rudder_tim = htim;
  g_rudder_state.pwm_started = 1U;
  g_rudder_state.source = RUDDER_SOURCE_CENTER;
  return HAL_OK;
}

static void Rudder_ApplyCommand(int16_t angle_cdeg, RudderSource source)
{
  RudderState next;
  int16_t applied;

  if (s_rudder_tim == NULL)
  {
    return;
  }

  next = g_rudder_state;
  next.source = source;
  next.requested_angle_cdeg = angle_cdeg;
  next.limit_active = 0U;
  applied = next.requested_angle_cdeg;
  if (applied > RUDDER_LIMIT_CDEG)
  {
    applied = RUDDER_LIMIT_CDEG;
    next.limit_active = 1U;
  }
  else if (applied < -RUDDER_LIMIT_CDEG)
  {
    applied = -RUDDER_LIMIT_CDEG;
    next.limit_active = 1U;
  }
  next.applied_angle_cdeg = applied;
  next.applied_pulse_us = Rudder_AngleToPulse(applied, &next.limit_active);
  next.update_count++;

  /* 网线与调试邮箱均提供绝对角度，本层不再积分或追赶旧目标。
     临界区同时发布 CCR 和状态，遥测不会读到一半新、一半旧的结构体。 */
  taskENTER_CRITICAL();
  __HAL_TIM_SET_COMPARE(s_rudder_tim, TIM_CHANNEL_4, next.applied_pulse_us);
  g_rudder_state = next;
  taskEXIT_CRITICAL();
}

void RudderDriver_Apply(int16_t angle_cdeg, uint8_t active)
{
  Rudder_ApplyCommand((active != 0U) ? angle_cdeg : 0,
                     (active != 0U) ? RUDDER_SOURCE_ETHERNET : RUDDER_SOURCE_CENTER);
}

void RudderDriver_ApplyDebug(int16_t angle_cdeg)
{
  Rudder_ApplyCommand(angle_cdeg, RUDDER_SOURCE_DEBUG);
}

bool RudderDriver_GetDebugCommand(uint8_t allow, int16_t *out_angle_cdeg)
{
  int32_t requested;

  if (allow == 0U)
  {
    g_rudder_debug.enabled = 0U;
    return false;
  }
  if (out_angle_cdeg == NULL || g_rudder_debug.enabled != 1U)
  {
    return false;
  }

  /* 两个 32 位字段分别原子读取；Ozone 不受任务临界区约束，因此先写目标再使能。
     只饱和到接口的整数范围，±45 度限幅仍统一由 Rudder_ApplyCommand 执行并报告。 */
  requested = g_rudder_debug.target_angle_cdeg;
  if (requested > INT16_MAX)
  {
    requested = INT16_MAX;
  }
  else if (requested < INT16_MIN)
  {
    requested = INT16_MIN;
  }
  *out_angle_cdeg = (int16_t)requested;
  return true;
}

bool RudderDriver_CopyState(RudderState *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *out_state = g_rudder_state;
  taskEXIT_CRITICAL();
  return true;
}
