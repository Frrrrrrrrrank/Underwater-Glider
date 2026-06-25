/* glider_time.h
 *
 * 全板单一来源的单调时间戳。集中放在 common/ 避免 (uint64_t)HAL_GetTick()*1000
 * 的 cast 顺序在多处被写错
 */
#ifndef GLIDER_TIME_H
#define GLIDER_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单调微秒时间戳。
 * 目前:(uint64_t)HAL_GetTick() * 1000ULL —— 1ms 分辨率，64 位不回绕。
 * 升级路径: DWT CYCCNT / TIM 1MHz —— 注意其 32 位回绕需软件扩展到 64 位。 */
uint64_t GliderTime_NowUs(void);

/* 单调毫秒（HAL_GetTick 直通）。 */
uint32_t GliderTime_NowMs(void);

#ifdef __cplusplus
}
#endif

#endif /* GLIDER_TIME_H */
