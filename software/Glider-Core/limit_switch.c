#include "limit_switch.h"

void LimitSwitch_ReadStates( volatile uint8_t states[LIMIT_SWITCH_COUNT])
{
  if (states == NULL)
  {
    return;
  }

  states[0] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) ? 1U : 0U;
  states[1] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1U : 0U;
  states[2] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1U : 0U;
  states[3] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t LimitSwitch_ReadRollActiveMask(void)
{
  /* 同一次 GPIOB 输入快照判定两端，返回位为 1 表示触发，不改变旧四路状态。 */
  const uint32_t port_levels = GPIOB->IDR;
  uint8_t active_mask = 0U;

/* PB5 触发：阻止正 RPM */
if ((port_levels & GPIO_PIN_5) == 0U)
{
  active_mask |= ROLL_LIMIT_POS_ACTIVE;
}

/* PB3 触发：阻止负 RPM */
if ((port_levels & GPIO_PIN_3) == 0U)
{
  active_mask |= ROLL_LIMIT_NEG_ACTIVE;
}

  return active_mask;
}
