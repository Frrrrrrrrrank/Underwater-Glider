/* zigzag_control.h
 *
 * 整机 zigzag 运动的 PID 控制层：给定期望深度、期望的下潜/上浮俯仰角、期望航向角
 * （roll 期望值恒为 0），观测量取自 g_state_estimate.out（EKF 输出，经 EkfBridge_CopyState
 * 受保护拷贝，roll/pitch/yaw 单位 rad，depth_m 单位 m，正=水下）。
 *
 * 当前实现：
 *   - up_down_flag 状态机：0=下潜，1=上浮，2=手动/外部接管（暂停本模块的自动泵与俯仰输出）。
 *     depth_m 达到期望深度 → 置 1；置 1 后 depth_m 重新小于 0.2m → 置 0。
 *     置 2 需外部显式调用 ZigzagControl_SetPhase()，本模块不会自行进入/退出该状态。
 *   - 按 up_down_flag 驱动压载泵（IoControlPump_SetSpeed/SetDirection）。
 *   - 俯仰 PID：设定值随 up_down_flag 在下潜/上浮目标俯仰角间切换，观测值为 out.pitch，
 *     输出经 RC_WORM_MOTOR_MAX_RPM_PID 限幅后经 UMotor_SetTargetSpeedById 下发给蜗杆电机。
 *   - roll PID：设定值恒为 0，观测值为 out.roll，输出经 RC_STEPPER_MAX_RPM_PID 限幅后
 *     经 Stepper_SetContinuousSignedRpm 下发给横滚步进电机。
 *   - 航向 PID：设定值为期望航向角，观测值为 out.yaw，仅计算，输出暂不接执行器，
 *     可经 ZigzagControl_GetYawPidOutput() 读取用于观察/调试。
 *   - 深度 PID：结构体已就绪（ZigzagControl_SetDepthPidGains 设置增益），
 *     暂不在 ZigzagControl_Step 中调用，留待后续接执行器时再接入。
 */
#ifndef ZIGZAG_CONTROL_H
#define ZIGZAG_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ZIGZAG_PHASE_DIVE   = 0U,  /* 下潜 */
  ZIGZAG_PHASE_CLIMB  = 1U,  /* 上浮 */
  ZIGZAG_PHASE_MANUAL = 2U   /* 手动/外部接管：泵与俯仰 PID 输出暂停 */
} ZigzagPhase;

/* 初始化内部状态：up_down_flag=ZIGZAG_PHASE_DIVE，俯仰 PID 积分/微分清零。 */
void ZigzagControl_Init(void);

/* 设置本轮 zigzag 的期望值：深度(m)，下潜/上浮俯仰角(deg)，航向角(deg)。
 * roll 期望值固定为 0，不需要设置。 */
void ZigzagControl_SetTargets(float target_depth_m,
                              float target_pitch_dive_deg,
                              float target_pitch_climb_deg,
                              float target_yaw_deg);

/* 设置俯仰 PID 增益（误差单位 deg）。 */
void ZigzagControl_SetPitchPidGains(float kp, float ki, float kd);

/* 设置 roll PID 增益（误差单位 deg，设定值恒为 0）。 */
void ZigzagControl_SetRollPidGains(float kp, float ki, float kd);

/* 设置航向 PID 增益（误差单位 deg，设定值为 ZigzagControl_SetTargets 的 target_yaw_deg）。 */
void ZigzagControl_SetYawPidGains(float kp, float ki, float kd);

/* 读取最近一次 Step() 算出的航向 PID 输出（暂未接执行器，供观察/调试）。 */
float ZigzagControl_GetYawPidOutput(void);

/* 设置深度 PID 增益（误差单位 m，设定值为 ZigzagControl_SetTargets 的 target_depth_m）。
 * 该 PID 暂不在 ZigzagControl_Step 中调用/接执行器。 */
void ZigzagControl_SetDepthPidGains(float kp, float ki, float kd);

/* 外部读/写 up_down_flag，用于置 2 进入手动接管，或从手动接管切回自动阶段。 */
void ZigzagControl_SetPhase(ZigzagPhase phase);
ZigzagPhase ZigzagControl_GetPhase(void);

/* 周期调用：dt_s 为距上次调用的秒数。内部读取 g_state_estimate、跑 up_down_flag
 * 状态机、驱动压载泵、跑俯仰 PID 并下发蜗杆电机目标转速。 */
void ZigzagControl_Step(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ZIGZAG_CONTROL_H */
