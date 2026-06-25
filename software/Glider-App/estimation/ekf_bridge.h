/* ekf_bridge.h
 *
 * sensors ↔ UgEkf wrapper 的桥接（纯逻辑层）：投喂 IMU/压力、拉取输出、维护
 * 受保护的状态估计快照。
 *
 * 单一所有者约定：
 *   - 只有 EKF task 调 EkfBridge_Init() / EkfBridge_Step()（含 UgEkf_GetOutput）。
 *   - IMU 侧（IMU task）调 EkfBridge_SubmitImu() 投喂样本。
 *   - 读侧（控制/遥测）调 EkfBridge_CopyState() 取受保护快照。
 *
 * 防撕裂：输入缓存与输出快照都用临界区保护；输入带 sequence number 防重复处理。
 */
#ifndef EKF_BRIDGE_H
#define EKF_BRIDGE_H

#include "state_estimate.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化滤波器（EKF task 启动时调一次）。 */
void EkfBridge_Init(void);

/* IMU 侧投喂一帧（线程上下文，非 ISR）：拷进受保护输入缓存并打 seq。
 * gyro rad/s, accel m/s^2（IMU 模块帧 raw；坐标旋转由 wrapper 的 mount_R 处理）。
 * t_us = 单调微秒（GliderTime_NowUs()）。 */
void EkfBridge_SubmitImu(float gx, float gy, float gz,
                         float ax, float ay, float az, uint64_t t_us);

/* EKF task 调：消费待处理 IMU 输入 → predict；有新压力 → 深度更新；
 * 拉输出并发布受保护快照。 */
void EkfBridge_Step(void);

/* 受保护地整体拷出最新状态估计。返回 update_seq。控制/遥测必须走这里。 */
uint32_t EkfBridge_CopyState(StateEstimate *out);

/* Ozone 观察量，散读仅供观察；闭环走 EkfBridge_CopyState */
extern StateEstimate     g_state_estimate;       /* 最新快照镜像 */
extern volatile uint32_t g_ekf_imu_push_count;
extern volatile uint32_t g_ekf_imu_drop_count;   /* 输入缓存被覆盖丢弃的帧数 */
extern volatile uint32_t g_ekf_pressure_push_count;

#ifdef __cplusplus
}
#endif

#endif /* EKF_BRIDGE_H */
