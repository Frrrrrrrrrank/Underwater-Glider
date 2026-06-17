/*
 * buzzer_driver.h
 *
 * PWM buzzer driver on TIM2 CH1 / PA0.
 */

#ifndef BUZZER_DRIVER_H
#define BUZZER_DRIVER_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#define BUZZER_DEFAULT_DUTY_PERMILLE 500U

typedef struct
{
  uint32_t frequency_hz;
  uint32_t duration_ms;
} BuzzerTone;

typedef struct
{
  uint32_t initialized;
  uint32_t pwm_started;
  uint32_t active;
  uint32_t frequency_hz;
  uint32_t stop_tick;
  uint32_t sequence_active;
  uint32_t sequence_index;
  uint32_t sequence_count;
  uint32_t play_count;
  uint32_t stop_count;
  uint32_t last_error;
} BuzzerDriverDiag;

extern volatile BuzzerDriverDiag g_buzzer_diag;

void BuzzerDriver_Init(TIM_HandleTypeDef *htim);
void BuzzerDriver_Play(uint32_t frequency_hz);
void BuzzerDriver_PlayFor(uint32_t frequency_hz, uint32_t duration_ms);
void BuzzerDriver_PlaySequence(const BuzzerTone *tones, uint32_t tone_count);
void BuzzerDriver_Stop(void);
void BuzzerDriver_Poll(uint32_t now_ms);

#endif /* BUZZER_DRIVER_H */
