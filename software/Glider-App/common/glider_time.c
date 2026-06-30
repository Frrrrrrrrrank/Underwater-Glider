/* glider_time.c，见 glider_time.h，统一时间戳 */
#include "glider_time.h"
#include "main.h"   /* HAL_GetTick */

uint64_t GliderTime_NowUs(void)
{
  /* cast 顺序关键，先转 64 位再乘。若写成 HAL_GetTick()*1000，
    那么乘法在 uint32 里算，约 71 分钟即回绕。 */
  return (uint64_t)HAL_GetTick() * 1000ULL;
}

uint32_t GliderTime_NowMs(void)
{
  return (uint32_t)HAL_GetTick();
}
