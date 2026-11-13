/* glider_time.c，见 glider_time.h，统一时间戳 */
#include "glider_time.h"
#include "main.h"   /* HAL_GetTick */
#include "FreeRTOS.h"
#include "task.h"

uint64_t GliderTime_NowUs(void)
{
  /* 所有调用来自任务；临界区保证多任务在回绕附近不会读出倒退时间。
   * 至少每 49.7 天调用一次，EKF 的 20ms 心跳满足此条件。 */
  static uint32_t previous_tick;
  static uint64_t high_ms;
  uint64_t result;
  uint32_t tick;
  taskENTER_CRITICAL();
  tick = HAL_GetTick();
  if (tick < previous_tick) high_ms += (1ULL << 32U);
  previous_tick = tick;
  result = (high_ms + tick) * 1000ULL;
  taskEXIT_CRITICAL();
  return result;
}

uint32_t GliderTime_NowMs(void)
{
  return (uint32_t)HAL_GetTick();
}
