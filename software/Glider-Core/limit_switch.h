#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define LIMIT_SWITCH_COUNT 4U

/* Index mapping:
 * states[0] -> PB3
 * states[1] -> PB4
 * states[2] -> PB5
 * states[3] -> PD7
 *
 * Level meaning:
 * 1 -> not touched (pulled up)
 * 0 -> touched (connected to GND)
 */
void LimitSwitch_ReadStates(volatile uint8_t states[LIMIT_SWITCH_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* LIMIT_SWITCH_H */
