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

/* 磁标定参数表固定为 64 字节：
 * float32 little-endian[16] = b[3] + A(row-major)[9] + reserved[4]。
 * reserved 必须全部为 +0.0f 的全零字节。 */
#define EKF_MAG_CAL_TABLE_BYTES 64U

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化滤波器（EKF task 启动时调一次）。 */
void EkfBridge_Init(void);

/* IMU 侧投喂一帧（线程上下文，非 ISR）：拷进受保护输入缓存并打 seq。
 * gyro rad/s, accel m/s^2, mag 模块帧 raw(uT，与 IMU 同帧；硬/软铁校正与 mount_R
 * 旋转都在 EKF task 侧做)。t_us = 单调微秒（GliderTime_NowUs()）。 */
void EkfBridge_SubmitImu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz, uint64_t t_us);

/* EKF task 调：消费待处理 IMU 输入 → predict；有新压力 → 深度更新；
 * 拉输出并发布受保护快照。 */
void EkfBridge_Step(void);

/* Ethernet task 只把完整表复制进 pending 邮箱；CRC、解析和应用均由 EKF task 完成。
 * 返回 1=已接收，0=table_bytes 为 NULL 或 EKF task 尚未完成初始化。
 * PC 在返回 0 时不会得到匹配 ACK，应按完整事务重试。 */
uint8_t EkfBridge_SubmitMagCalibration(
    const uint8_t table_bytes[EKF_MAG_CAL_TABLE_BYTES],
    uint32_t expected_crc);

/* 受保护地整体拷出最新状态估计。返回 update_seq。控制/遥测必须走这里。 */
uint32_t EkfBridge_CopyState(StateEstimate *out);

/* Ozone 观察量，散读仅供观察；闭环走 EkfBridge_CopyState */
extern StateEstimate     g_state_estimate;       /* 最新快照镜像 */
extern volatile uint32_t g_ekf_imu_push_count;
extern volatile uint32_t g_ekf_imu_drop_count;   /* 输入缓存被覆盖丢弃的帧数 */
extern volatile uint32_t g_ekf_pressure_push_count;
extern volatile uint32_t g_ekf_mag_push_count;   /* 已 push 的校正 mag 帧数 */
extern volatile float    g_ekf_depth_meas;       /* 换算后的深度测量(m)，对比 out.depth_m */
extern volatile float    g_ekf_mag_cal[3];       /* 校正后 mag(模块系,uT)；转动时 |·| 应≈常数 */
extern volatile uint32_t g_ekf_mag_cal_crc;      /* 当前已生效参数表 CRC32，遥测 ACK */
extern volatile uint8_t  g_ekf_mag_cal_valid;    /* 1=已成功提交；0 时不 push mag */
extern volatile uint32_t g_ekf_mag_cal_request_count;
extern volatile uint32_t g_ekf_mag_cal_apply_count;
extern volatile uint32_t g_ekf_mag_cal_reject_count;

#ifdef __cplusplus
}
#endif

#endif /* EKF_BRIDGE_H */
