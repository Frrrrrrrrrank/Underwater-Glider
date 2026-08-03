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
