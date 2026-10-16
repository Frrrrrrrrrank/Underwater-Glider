#ifndef GLIDER_CORE_STEPPER_DRIVER_H
#define GLIDER_CORE_STEPPER_DRIVER_H

#include <stdint.h>

/* 触发立即禁止；首次采到释放后，至少经过此时间再采样确认。 */
#define STEPPER_LIMIT_RELEASE_MS 20U

typedef enum {
    STEPPER_STOP_NONE = 0,
    STEPPER_STOP_COMMAND,
    STEPPER_STOP_COMPLETED,
    STEPPER_STOP_LIMIT,
    STEPPER_STOP_START_FAILED,
    STEPPER_STOP_RANGE
} StepperStopReason;

/* 脉冲速度和位置都是软件输出/累计值，不是编码器反馈或已标定行程。 */
typedef struct {
    int32_t requested_rpm;
    int32_t pulse_rpm;
    int32_t current_pos;
    int32_t target_pos;
    uint32_t pulses_remaining;
    uint32_t limit_stop_count;
    uint8_t running;
    uint8_t raw_limit_mask; /* 采用 ROLL_LIMIT_*_ACTIVE：1 表示触发。 */
    uint8_t blocked_mask;   /* 包含触发后的释放确认等待。 */
    uint8_t last_stop_reason;
} StepperState;

/* 
 * 步进电机驱动器 (开环外部定频脉冲版)
 * 使用 TIM1_CH2(PE11) 产生 PWM 脉冲, GPIO(PE13) 控制方向
 * 允许 STM32 运行时手动随时修改速度和绝对/相对运行距离
 * 运动命令及 ServiceLimits 必须由同一个任务调用，生产入口为 StartServoTask。
 * CopyState 可由其他任务读取；中断只调用 Stepper_TIM_IRQHandler。
 */

/* 初始化开环步进电机 */
void Stepper_Init(void);

/* 任务接口：每拍无条件调用，停机/未使能时也维护两端释放；不会自行启动。 */
void Stepper_ServiceLimits(uint32_t now_ms);

/* 任务接口：复制一致快照；空指针返回 0，成功返回 1。 */
uint8_t Stepper_CopyState(StepperState *out_state);

/* 
 * 相对运动
 * @param pulses: 相对移动的脉冲数，正数为正转，负数为反转
 * @param rpm: 目标转速(RPM)
 * 非零转速按 30~800 RPM 限幅；距离或转速为零表示停止当前运动。
 * 目标坐标超出 int32_t 范围时停止并记录 RANGE，限位中止后不自动续跑。
 */
void Stepper_MoveRelative(int32_t pulses, uint16_t rpm);

/* 
 * 绝对运动
 * @param target_pos: 目标绝对位置(脉冲数)
 * @param rpm: 目标转速(RPM)
 * 坐标来自本次初始化后的脉冲累计；相对距离超出 int32_t 范围时拒绝。
 * 目标等于当前位置或转速为零表示停止，非零转速按 30~800 RPM 限幅。
 */
void Stepper_MoveAbsolute(int32_t target_pos, uint16_t rpm);

/* 任务接口：立即停止脉冲并清除请求，不释放驱动器供电、不清限位状态。 */
void Stepper_Stop(void);

/*
 * Continuous speed command (open-loop):
 *  signed_rpm > 0: forward, signed_rpm < 0: reverse, 0: stop.
 */
void Stepper_SetContinuousSignedRpm(int32_t signed_rpm);

/* 检查电机是否正在运行 (1=运行中, 0=已停止) */
uint8_t Stepper_IsRunning(void);

/* 获取当前绝对位置累计 (单位：脉冲) */
int32_t Stepper_GetCurrentPosition(void);

/* 
 * 中断处理勾子函数 (由底层自动调用，处理脉冲计数)
 */
void Stepper_TIM_IRQHandler(void);

/* 测试步进电机的 FreeRTOS 任务 */
void Stepper_TestTask(void *argument);

#endif /* GLIDER_CORE_STEPPER_DRIVER_H */
