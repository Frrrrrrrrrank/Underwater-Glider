#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define LIMIT_SWITCH_COUNT 4U

/* 横滚限位有效位：1 表示触发，与旧四路 states/遥测的 0 表示触发不同。 */
#define ROLL_LIMIT_POS_ACTIVE (1U << 0)
#define ROLL_LIMIT_NEG_ACTIVE (1U << 1)
#define ROLL_LIMIT_ALL_ACTIVE (ROLL_LIMIT_POS_ACTIVE | ROLL_LIMIT_NEG_ACTIVE)

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

/* PB5 触发阻止正 RPM，PB3 触发阻止负 RPM；只读取，不更新旧四路状态。 */
uint8_t LimitSwitch_ReadRollActiveMask(void);

#ifdef __cplusplus
}
#endif

#endif /* LIMIT_SWITCH_H */
