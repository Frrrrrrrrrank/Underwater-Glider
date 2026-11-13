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

/* 任务上下文单调微秒时间戳，1ms 分辨率；软件扩展 HAL 32位毫秒回绕。
 * 必须至少每 49.7 天调用一次，不能从 ISR/NMI 调用。
 * 升级路径: DWT CYCCNT / TIM 1MHz，同样需要软件扩展回绕。 */
uint64_t GliderTime_NowUs(void);

/* 单调毫秒（HAL_GetTick 直通）。 */
uint32_t GliderTime_NowMs(void);

#ifdef __cplusplus
}
#endif

#endif /* GLIDER_TIME_H */
