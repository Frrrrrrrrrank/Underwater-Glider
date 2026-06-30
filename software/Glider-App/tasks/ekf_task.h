/* ekf_task.h
 *
 * ESKF 的 FreeRTOS 任务层，事件触发循环 + IMU 输入入口。
 * 估计逻辑在 ekf_bridge；本层只负责 RTOS 调度与唤醒，是唯一碰滤波器单例的任务。
 */
#ifndef EKF_TASK_H
#define EKF_TASK_H

#include "imu_core.h"     /* IMU_RawData */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_FLAG_IMU         (0x01U)   /* osThreadFlags 位，新 IMU 帧就绪 */
#define EKF_TASK_FALLBACK_MS (20U)     /* 无 IMU 帧时的心跳兜底周期 (ms) */

/* FreeRTOS 任务入口在main.c 用 osThreadNew 注册 */
void StartEkfTask(void *argument);

/* IMU task 拿到新 raw 帧后调用，线程上下文，非 ISR：
 *   先把 gyro/accel 投喂给 bridge; 之后记录 raw mag 调试值，第一阶段不入 EKF，需标定软磁硬磁参数；
 *   然后调用 osThreadFlagsSet 唤醒 EKF task。t_us 用 GliderTime_NowUs() */
void EkfTask_PublishImuRaw(const IMU_RawData *raw, uint64_t t_us);

/* Ozone 观察量 */
extern volatile uint32_t g_ekf_task_loop_count;
extern volatile uint32_t g_ekf_stack_high_water_words;  /* 任务栈剩余水位 (words) */
extern volatile float    g_ekf_mag_raw[3];              /* raw mag(uT)，v1 不入 EKF */
extern volatile float    g_ekf_imu_hz;                  /* 实测 IMU 真实新帧率 (Hz)，每秒刷新 */

#ifdef __cplusplus
}
#endif

#endif /* EKF_TASK_H */
