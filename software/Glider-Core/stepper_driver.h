#ifndef GLIDER_CORE_STEPPER_DRIVER_H
#define GLIDER_CORE_STEPPER_DRIVER_H

#include <stdint.h>

/* 
 * 步进电机驱动器 (开环外部定频脉冲版)
 * 使用 TIM1_CH2(PE11) 产生 PWM 脉冲, GPIO(PE13) 控制方向
 * 允许 STM32 运行时手动随时修改速度和绝对/相对运行距离
 */

/* 初始化开环步进电机 */
void Stepper_Init(void);

/* 
 * 相对运动
 * @param pulses: 相对移动的脉冲数，正数为正转，负数为反转
 * @param rpm: 目标转速(RPM)
 */
void Stepper_MoveRelative(int32_t pulses, uint16_t rpm);

/* 
 * 绝对运动
 * @param target_pos: 目标绝对位置(脉冲数)
 * @param rpm: 目标转速(RPM)
 */
void Stepper_MoveAbsolute(int32_t target_pos, uint16_t rpm);

/* 立即停止正在进行的运动 */
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