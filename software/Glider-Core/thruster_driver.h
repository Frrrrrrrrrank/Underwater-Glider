#ifndef THRUSTER_DRIVER_H
#define THRUSTER_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define THRUSTER_PWM_FREQ_HZ     200U
#define THRUSTER_PERIOD_US       (1000000U / THRUSTER_PWM_FREQ_HZ)
#define THRUSTER_PULSE_MIN_US    1000U
#define THRUSTER_PULSE_NEUTRAL_US 1500U
#define THRUSTER_PULSE_MAX_US    2000U

/* PE5 -> TIM15_CH1, PE6 -> TIM15_CH2 */
#define THRUSTER_CHANNEL_PE5     TIM_CHANNEL_1
#define THRUSTER_CHANNEL_PE6     TIM_CHANNEL_2

/*
 * Initialize external thruster PWM output on TIM15 CH1/CH2.
 * The timer will be configured to 200 Hz and both channels are started.
 */
HAL_StatusTypeDef ThrusterDriver_Init(TIM_HandleTypeDef *htim15);

/*
 * Set one channel duty in percent [0, 100].
 * Mapping:
 *   0%   -> 1.0 ms high level
 *   100% -> 2.0 ms high level
 */
HAL_StatusTypeDef ThrusterDriver_SetDutyPercent(uint32_t channel, float duty_percent);

/* Set PE5 and PE6 channel duty in one call. */
HAL_StatusTypeDef ThrusterDriver_SetDualDutyPercent(float duty_pe5, float duty_pe6);

/* Set one channel pulse width directly in microseconds [1000, 2000]. */
HAL_StatusTypeDef ThrusterDriver_SetPulseUs(uint32_t channel, uint16_t pulse_us);

/* Set PE5 and PE6 channel pulse widths directly in microseconds. */
HAL_StatusTypeDef ThrusterDriver_SetDualPulseUs(uint16_t pulse_pe5_us, uint16_t pulse_pe6_us);

/* Stop both PWM channels. */
HAL_StatusTypeDef ThrusterDriver_Stop(void);

/* Convert duty percent [0, 100] to pulse width [1000, 2000] us. */
uint16_t ThrusterDriver_DutyToPulseUs(float duty_percent);

/*
 * Map thrust direction/speed command to pulse width.
 * direction=0: speed 0->100 maps 1500us -> 1000us
 * direction=1: speed 0->100 maps 1500us -> 2000us
 */
uint16_t ThrusterDriver_MapDirectionSpeedToPulseUs(uint8_t direction, uint8_t speed_percent);

#ifdef __cplusplus
}
#endif

#endif /* THRUSTER_DRIVER_H */
