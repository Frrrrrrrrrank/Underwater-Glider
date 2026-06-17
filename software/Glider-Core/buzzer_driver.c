/*
 * buzzer_driver.c
 *
 * PWM buzzer driver on TIM2 CH1 / PA0.
 */

#include "buzzer_driver.h"

#define BUZZER_TIM_CHANNEL TIM_CHANNEL_1
#define BUZZER_MIN_FREQ_HZ 50U
#define BUZZER_MAX_FREQ_HZ 20000U

volatile BuzzerDriverDiag g_buzzer_diag = {0};

static TIM_HandleTypeDef *s_buzzer_tim = NULL;
static uint32_t s_timer_clock_hz = 120000000U;
static const BuzzerTone *s_sequence = NULL;

static uint32_t Buzzer_GetTimerClockHz(TIM_HandleTypeDef *htim)
{
  uint32_t pclk1;
  uint32_t timer_clock;

  (void)htim;

  pclk1 = HAL_RCC_GetPCLK1Freq();
  timer_clock = pclk1;

  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0U)
  {
    timer_clock = pclk1 * 2U;
  }

  if (timer_clock == 0U)
  {
    timer_clock = 120000000U;
  }

  return timer_clock;
}

static uint8_t Buzzer_ConfigFrequency(uint32_t frequency_hz, uint32_t duty_permille)
{
  uint32_t timer_clock;
  uint32_t prescaler;
  uint32_t arr;
  uint32_t ccr;

  if (s_buzzer_tim == NULL)
  {
    g_buzzer_diag.last_error = 1U;
    return 0U;
  }

  if ((frequency_hz < BUZZER_MIN_FREQ_HZ) || (frequency_hz > BUZZER_MAX_FREQ_HZ))
  {
    g_buzzer_diag.last_error = 2U;
    return 0U;
  }

  if (duty_permille > 1000U)
  {
    duty_permille = 1000U;
  }

  timer_clock = s_timer_clock_hz;
  prescaler = (timer_clock / (frequency_hz * 65536U)) + 1U;
  if (prescaler == 0U)
  {
    prescaler = 1U;
  }

  arr = (timer_clock / (prescaler * frequency_hz));
  if (arr == 0U)
  {
    arr = 1U;
  }
  arr -= 1U;

  ccr = ((arr + 1U) * duty_permille) / 1000U;

  __HAL_TIM_DISABLE(s_buzzer_tim);
  __HAL_TIM_SET_PRESCALER(s_buzzer_tim, prescaler - 1U);
  __HAL_TIM_SET_AUTORELOAD(s_buzzer_tim, arr);
  __HAL_TIM_SET_COMPARE(s_buzzer_tim, BUZZER_TIM_CHANNEL, ccr);
  __HAL_TIM_SET_COUNTER(s_buzzer_tim, 0U);
  s_buzzer_tim->Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(s_buzzer_tim);

  g_buzzer_diag.frequency_hz = frequency_hz;
  g_buzzer_diag.last_error = 0U;
  return 1U;
}

void BuzzerDriver_Init(TIM_HandleTypeDef *htim)
{
  s_buzzer_tim = htim;
  g_buzzer_diag.initialized = (htim != NULL) ? 1U : 0U;
  g_buzzer_diag.pwm_started = 0U;
  g_buzzer_diag.active = 0U;
  g_buzzer_diag.frequency_hz = 0U;
  g_buzzer_diag.stop_tick = 0U;
  g_buzzer_diag.sequence_active = 0U;
  g_buzzer_diag.sequence_index = 0U;
  g_buzzer_diag.sequence_count = 0U;
  g_buzzer_diag.play_count = 0U;
  g_buzzer_diag.stop_count = 0U;
  g_buzzer_diag.last_error = 0U;
  s_sequence = NULL;

  if (htim == NULL)
  {
    g_buzzer_diag.last_error = 1U;
    return;
  }

  s_timer_clock_hz = Buzzer_GetTimerClockHz(htim);
  __HAL_TIM_SET_COMPARE(htim, BUZZER_TIM_CHANNEL, 0U);

  if (HAL_TIM_PWM_Start(htim, BUZZER_TIM_CHANNEL) == HAL_OK)
  {
    g_buzzer_diag.pwm_started = 1U;
  }
  else
  {
    g_buzzer_diag.last_error = 3U;
  }
}

void BuzzerDriver_Play(uint32_t frequency_hz)
{
  if (g_buzzer_diag.initialized == 0U)
  {
    g_buzzer_diag.last_error = 4U;
    return;
  }

  if (Buzzer_ConfigFrequency(frequency_hz, BUZZER_DEFAULT_DUTY_PERMILLE) == 0U)
  {
    return;
  }

  g_buzzer_diag.active = 1U;
  g_buzzer_diag.stop_tick = 0U;
  g_buzzer_diag.sequence_active = 0U;
  s_sequence = NULL;
  g_buzzer_diag.play_count++;
}

void BuzzerDriver_PlayFor(uint32_t frequency_hz, uint32_t duration_ms)
{
  BuzzerDriver_Play(frequency_hz);

  if ((g_buzzer_diag.active != 0U) && (duration_ms > 0U))
  {
    g_buzzer_diag.stop_tick = HAL_GetTick() + duration_ms;
  }
}

void BuzzerDriver_PlaySequence(const BuzzerTone *tones, uint32_t tone_count)
{
  if ((tones == NULL) || (tone_count == 0U))
  {
    g_buzzer_diag.last_error = 5U;
    return;
  }

  s_sequence = tones;
  g_buzzer_diag.sequence_active = 1U;
  g_buzzer_diag.sequence_index = 0U;
  g_buzzer_diag.sequence_count = tone_count;

  if (Buzzer_ConfigFrequency(tones[0].frequency_hz, BUZZER_DEFAULT_DUTY_PERMILLE) == 0U)
  {
    g_buzzer_diag.sequence_active = 0U;
    s_sequence = NULL;
    return;
  }

  g_buzzer_diag.active = 1U;
  g_buzzer_diag.stop_tick = HAL_GetTick() + tones[0].duration_ms;
  g_buzzer_diag.play_count++;
}

void BuzzerDriver_Stop(void)
{
  if (s_buzzer_tim != NULL)
  {
    __HAL_TIM_SET_COMPARE(s_buzzer_tim, BUZZER_TIM_CHANNEL, 0U);
  }

  g_buzzer_diag.active = 0U;
  g_buzzer_diag.frequency_hz = 0U;
  g_buzzer_diag.stop_tick = 0U;
  g_buzzer_diag.sequence_active = 0U;
  g_buzzer_diag.sequence_index = 0U;
  g_buzzer_diag.sequence_count = 0U;
  s_sequence = NULL;
  g_buzzer_diag.stop_count++;
}

void BuzzerDriver_Poll(uint32_t now_ms)
{
  uint32_t next_index;

  if ((g_buzzer_diag.active == 0U) || (g_buzzer_diag.stop_tick == 0U))
  {
    return;
  }

  if ((int32_t)(now_ms - g_buzzer_diag.stop_tick) >= 0)
  {
    if ((g_buzzer_diag.sequence_active != 0U) && (s_sequence != NULL))
    {
      next_index = g_buzzer_diag.sequence_index + 1U;
      if (next_index < g_buzzer_diag.sequence_count)
      {
        g_buzzer_diag.sequence_index = next_index;
        if (Buzzer_ConfigFrequency(s_sequence[next_index].frequency_hz, BUZZER_DEFAULT_DUTY_PERMILLE) == 0U)
        {
          BuzzerDriver_Stop();
          return;
        }
        g_buzzer_diag.stop_tick = now_ms + s_sequence[next_index].duration_ms;
      }
      else
      {
        BuzzerDriver_Stop();
      }
    }
    else
    {
      BuzzerDriver_Stop();
    }
  }
}
