#include "thruster_driver.h"

static TIM_HandleTypeDef *s_thruster_tim = NULL;

static uint32_t ThrusterDriver_GetTimClockHz(void)
{
  RCC_ClkInitTypeDef clk_init;
  uint32_t flash_latency = 0U;
  uint32_t pclk2;

  HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
  pclk2 = HAL_RCC_GetPCLK2Freq();

  if (clk_init.APB2CLKDivider == RCC_HCLK_DIV1)
  {
    return pclk2;
  }

  return (pclk2 * 2U);
}

uint16_t ThrusterDriver_DutyToPulseUs(float duty_percent)
{
  float clamped = duty_percent;
  float pulse_us;

  if (clamped < 0.0f)
  {
    clamped = 0.0f;
  }
  if (clamped > 100.0f)
  {
    clamped = 100.0f;
  }

  pulse_us = (float)THRUSTER_PULSE_MIN_US + (clamped * 10.0f);
  if (pulse_us < (float)THRUSTER_PULSE_MIN_US)
  {
    pulse_us = (float)THRUSTER_PULSE_MIN_US;
  }
  if (pulse_us > (float)THRUSTER_PULSE_MAX_US)
  {
    pulse_us = (float)THRUSTER_PULSE_MAX_US;
  }

  return (uint16_t)(pulse_us + 0.5f);
}

uint16_t ThrusterDriver_MapDirectionSpeedToPulseUs(uint8_t direction, uint8_t speed_percent)
{
  uint8_t speed = speed_percent;

  if (speed > 100U)
  {
    speed = 100U;
  }

  if (direction == 0U)
  {
    return (uint16_t)(THRUSTER_PULSE_NEUTRAL_US - (speed * 5U));
  }

  if (direction == 1U)
  {
    return (uint16_t)(THRUSTER_PULSE_NEUTRAL_US + (speed * 5U));
  }

  return THRUSTER_PULSE_NEUTRAL_US;
}

HAL_StatusTypeDef ThrusterDriver_Init(TIM_HandleTypeDef *htim15)
{
  uint32_t tim_clk_hz;
  uint64_t total_counts;
  uint32_t psc;
  uint32_t period_counts;
  uint32_t arr;

  if (htim15 == NULL)
  {
    return HAL_ERROR;
  }

  s_thruster_tim = htim15;

  tim_clk_hz = ThrusterDriver_GetTimClockHz();
  if (tim_clk_hz == 0U)
  {
    return HAL_ERROR;
  }

  total_counts = ((uint64_t)tim_clk_hz + (THRUSTER_PWM_FREQ_HZ / 2U)) / (uint64_t)THRUSTER_PWM_FREQ_HZ;
  if (total_counts < 2ULL)
  {
    return HAL_ERROR;
  }

  psc = (uint32_t)((total_counts - 1ULL) / 65536ULL);
  if (psc > 0xFFFFU)
  {
    return HAL_ERROR;
  }

  period_counts = (uint32_t)(total_counts / (uint64_t)(psc + 1U));
  if (period_counts < 2U)
  {
    period_counts = 2U;
  }
  if (period_counts > 65536U)
  {
    period_counts = 65536U;
  }

  arr = period_counts - 1U;

  __HAL_TIM_SET_PRESCALER(s_thruster_tim, psc);
  __HAL_TIM_SET_AUTORELOAD(s_thruster_tim, arr);
  __HAL_TIM_SET_COUNTER(s_thruster_tim, 0U);

  if (HAL_TIM_GenerateEvent(s_thruster_tim, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ThrusterDriver_SetDualDutyPercent(50.0f, 50.0f) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(s_thruster_tim, THRUSTER_CHANNEL_PE5) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(s_thruster_tim, THRUSTER_CHANNEL_PE6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef ThrusterDriver_SetDutyPercent(uint32_t channel, float duty_percent)
{
  uint16_t pulse_us;

  if (s_thruster_tim == NULL)
  {
    return HAL_ERROR;
  }

  if ((channel != THRUSTER_CHANNEL_PE5) && (channel != THRUSTER_CHANNEL_PE6))
  {
    return HAL_ERROR;
  }

  pulse_us = ThrusterDriver_DutyToPulseUs(duty_percent);

  return ThrusterDriver_SetPulseUs(channel, pulse_us);
}

HAL_StatusTypeDef ThrusterDriver_SetPulseUs(uint32_t channel, uint16_t pulse_us)
{
  uint32_t arr;
  uint32_t period_counts;
  uint32_t compare;
  uint16_t pulse = pulse_us;

  if (s_thruster_tim == NULL)
  {
    return HAL_ERROR;
  }

  if ((channel != THRUSTER_CHANNEL_PE5) && (channel != THRUSTER_CHANNEL_PE6))
  {
    return HAL_ERROR;
  }

  if (pulse < THRUSTER_PULSE_MIN_US)
  {
    pulse = THRUSTER_PULSE_MIN_US;
  }
  if (pulse > THRUSTER_PULSE_MAX_US)
  {
    pulse = THRUSTER_PULSE_MAX_US;
  }

  arr = __HAL_TIM_GET_AUTORELOAD(s_thruster_tim);
  period_counts = arr + 1U;

  compare = (uint32_t)((((uint64_t)period_counts * (uint64_t)pulse) + (THRUSTER_PERIOD_US / 2U)) / THRUSTER_PERIOD_US);
  if (compare > arr)
  {
    compare = arr;
  }

  __HAL_TIM_SET_COMPARE(s_thruster_tim, channel, compare);
  return HAL_OK;
}

HAL_StatusTypeDef ThrusterDriver_SetDualDutyPercent(float duty_pe5, float duty_pe6)
{
  if (ThrusterDriver_SetDutyPercent(THRUSTER_CHANNEL_PE5, duty_pe5) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ThrusterDriver_SetDutyPercent(THRUSTER_CHANNEL_PE6, duty_pe6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef ThrusterDriver_SetDualPulseUs(uint16_t pulse_pe5_us, uint16_t pulse_pe6_us)
{
  if (ThrusterDriver_SetPulseUs(THRUSTER_CHANNEL_PE5, pulse_pe5_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ThrusterDriver_SetPulseUs(THRUSTER_CHANNEL_PE6, pulse_pe6_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef ThrusterDriver_Stop(void)
{
  if (s_thruster_tim == NULL)
  {
    return HAL_OK;
  }

  if (HAL_TIM_PWM_Stop(s_thruster_tim, THRUSTER_CHANNEL_PE5) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Stop(s_thruster_tim, THRUSTER_CHANNEL_PE6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}
