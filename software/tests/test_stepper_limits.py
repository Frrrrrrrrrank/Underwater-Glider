"""主机编译真实步进驱动与限位输入；模拟 GPIO、定时器和任务临界区，不连接设备。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]

HAL_STUB = r"""
#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H
#include <stddef.h>
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t IDR, ODR; } GPIO_TypeDef;
typedef struct { uint32_t CR1, DIER, SR, CNT, ARR, CCR2, EGR, PSC; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } UART_HandleTypeDef;
extern GPIO_TypeDef test_gpiob, test_gpiod, test_gpioe;
extern TIM_TypeDef test_tim1;
#define GPIOB (&test_gpiob)
#define GPIOD (&test_gpiod)
#define GPIOE (&test_gpioe)
#define TIM1 (&test_tim1)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define TIM_CHANNEL_2 4U
#define TIM_CR1_CEN 1U
#define TIM_IT_UPDATE 1U
#define TIM_FLAG_UPDATE 1U
#define TIM_EGR_UG 1U
#define __HAL_TIM_ENABLE_IT(h, m) ((h)->Instance->DIER |= (m))
#define __HAL_TIM_DISABLE_IT(h, m) ((h)->Instance->DIER &= ~(m))
#define __HAL_TIM_CLEAR_FLAG(h, m) test_clear_flag((h), (m))
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->Instance->CNT = (v))
#define __HAL_TIM_DISABLE(h) ((h)->Instance->CR1 &= ~TIM_CR1_CEN)
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((h)->Instance->CCR2 = (v))
#define __HAL_TIM_SET_AUTORELOAD(h, v) ((h)->Instance->ARR = (v))
#define __NOP() test_nop()
void test_nop(void);
void test_clear_flag(TIM_HandleTypeDef *tim, uint32_t mask);
uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *tim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *tim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *tim);
#endif
"""

TASK_STUB = r"""
#ifndef TASK_H
#define TASK_H
void test_enter_critical(void);
void test_exit_critical(void);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
#endif
"""

CMSIS_STUB = r"""
#ifndef CMSIS_OS_H
#define CMSIS_OS_H
#include <stdint.h>
void osDelay(uint32_t ms);
uint32_t osKernelGetTickCount(void);
#endif
"""

HARNESS = r"""
#include "stepper_driver.h"
#include "limit_switch.h"
#include "sbus_driver.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

GPIO_TypeDef test_gpiob, test_gpiod, test_gpioe;
TIM_TypeDef test_tim1;
TIM_HandleTypeDef htim1 = { TIM1 };
volatile SbusState g_sbus_state;
static uint32_t tick;
static unsigned critical_depth, critical_entries, in_irq, starts;
static unsigned inject_on_nop;
static uint8_t injected_mask;
static HAL_StatusTypeDef start_result;
static StepperState state(void);
/* 简化的 PWM2 模型：ARR/CCR 只在 UG/UEV 装载，低电平起步、停止回低。
 * 记录电气下降沿，不能模拟驱动器最小脉宽、IRQ延迟分布或机械丢步。 */
static uint32_t active_arr, active_ccr;
static unsigned pulse_high, pwm_enabled, startup_loaded;
static int32_t electrical_position;

static void set_output_level(unsigned high)
{
    if (pulse_high != 0U && high == 0U)
    {
        electrical_position += (test_gpioe.ODR & GPIO_PIN_13) ? 1 : -1;
    }
    pulse_high = high;
}

void test_clear_flag(TIM_HandleTypeDef *tim, uint32_t mask)
{
    if ((tim->Instance->EGR & TIM_EGR_UG) != 0U)
    {
        assert((tim->Instance->CR1 & TIM_CR1_CEN) == 0U);
        active_arr = tim->Instance->ARR;
        active_ccr = tim->Instance->CCR2;
        tim->Instance->CNT = 0U;
        tim->Instance->EGR = 0U;
        tim->Instance->SR |= TIM_FLAG_UPDATE;
        startup_loaded = 1U;
    }
    tim->Instance->SR &= ~mask;
}

static void timer_ticks(uint32_t count)
{
    while (count-- != 0U)
    {
        assert((test_tim1.CR1 & TIM_CR1_CEN) != 0U && pwm_enabled != 0U);
        ++test_tim1.CNT;
        if (test_tim1.CNT > active_arr)
        {
            test_tim1.CNT = 0U;
            active_arr = test_tim1.ARR;
            active_ccr = test_tim1.CCR2;
            test_tim1.SR |= TIM_FLAG_UPDATE;
        }
        assert(active_ccr > 0U && active_ccr <= active_arr);
        set_output_level(test_tim1.CNT >= active_ccr);
    }
}

void test_enter_critical(void)
{
    assert(in_irq == 0U);
    ++critical_depth;
    ++critical_entries;
}

void test_exit_critical(void)
{
    assert(critical_depth > 0U);
    --critical_depth;
}

static void set_limits(uint8_t mask)
{
    test_gpiob.IDR = GPIO_PIN_4 |
        ((mask & ROLL_LIMIT_POS_ACTIVE) ? 0U : GPIO_PIN_5) |
        ((mask & ROLL_LIMIT_NEG_ACTIVE) ? 0U : GPIO_PIN_3);
    test_gpiod.IDR = GPIO_PIN_7;
}

void test_nop(void)
{
    /* 在方向建立等待中模拟机械开关压下，验证启动前最后一次检查。 */
    assert(critical_depth == 0U);
    if (inject_on_nop != 0U)
    {
        /* 启动等待期间仍发布完整停机快照，不能提前公布剩余脉冲。 */
        assert(state().running == 0U);
        inject_on_nop = 0U;
        set_limits(injected_mask);
    }
}

uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
void osDelay(uint32_t ms) { tick += ms; }
uint32_t SbusDriver_IsFrameFresh(uint32_t now, uint32_t timeout)
{
    (void)now;
    (void)timeout;
    return 0U;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) { port->ODR |= pin; }
    else { port->ODR &= ~((uint32_t)pin); }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *tim, uint32_t channel)
{
    assert(tim->Instance == TIM1 && channel == TIM_CHANNEL_2);
    assert(critical_depth > 0U && in_irq == 0U);
    assert(startup_loaded != 0U && tim->Instance->CNT == 0U);
    assert((tim->Instance->SR & TIM_FLAG_UPDATE) == 0U);
    assert(pulse_high == 0U && pwm_enabled == 0U);
    startup_loaded = 0U;
    ++starts;
    if (start_result == HAL_OK)
    {
        pwm_enabled = 1U;
        tim->Instance->CR1 |= TIM_CR1_CEN;
    }
    return start_result;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *tim, uint32_t channel)
{
    assert(tim->Instance == TIM1 && channel == TIM_CHANNEL_2);
    set_output_level(0U);
    pwm_enabled = 0U;
    tim->Instance->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *tim)
{
    tim->Instance->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

static StepperState state(void)
{
    StepperState out;
    assert(Stepper_CopyState(&out) != 0U);
    assert(critical_depth == 0U);
    assert(out.running == Stepper_IsRunning());
    assert(out.current_pos == Stepper_GetCurrentPosition());
    if (out.running == 0U)
    {
        assert(out.pulse_rpm == 0 && out.pulses_remaining == 0U);
        assert((test_tim1.CR1 & TIM_CR1_CEN) == 0U);
        assert((test_tim1.DIER & TIM_IT_UPDATE) == 0U);
    }
    return out;
}

static void reset(uint8_t mask)
{
    assert(critical_depth == 0U);
    memset(&test_tim1, 0, sizeof(test_tim1));
    memset(&test_gpioe, 0, sizeof(test_gpioe));
    starts = 0U;
    active_arr = active_ccr = 0U;
    pulse_high = pwm_enabled = startup_loaded = 0U;
    electrical_position = 0;
    tick = 0U;
    inject_on_nop = 0U;
    start_result = HAL_OK;
    set_limits(mask);
    Stepper_Init();
    assert(state().blocked_mask == mask);
    assert(state().raw_limit_mask == mask);
    assert(state().current_pos == 0);
    assert(state().running == 0U);
}

static void dispatch_irq(void)
{
    unsigned before = critical_entries;
    assert(critical_depth == 0U);
    /* 与真实 HAL_TIM_IRQHandler 一致，先清本次 UIF，再调用更新回调。 */
    test_tim1.SR &= ~TIM_FLAG_UPDATE;
    in_irq = 1U;
    Stepper_TIM_IRQHandler();
    in_irq = 0U;
    assert(critical_entries == before);
}

static void irq(void)
{
    if ((test_tim1.CR1 & TIM_CR1_CEN) != 0U)
    {
        timer_ticks(active_arr + 1U - test_tim1.CNT);
    }
    dispatch_irq();
}

static void service(uint32_t now)
{
    tick = now;
    Stepper_ServiceLimits(now);
    assert(critical_depth == 0U);
}

static void test_gpio(void)
{
    uint8_t levels[LIMIT_SWITCH_COUNT];
    unsigned mask;
    for (mask = 0U; mask <= ROLL_LIMIT_ALL_ACTIVE; ++mask)
    {
        reset((uint8_t)mask);
        assert(LimitSwitch_ReadRollActiveMask() == mask);
        LimitSwitch_ReadStates(levels);
        assert(levels[0] == ((mask & ROLL_LIMIT_NEG_ACTIVE) ? 0U : 1U));
        assert(levels[1] == 1U && levels[3] == 1U);
        assert(levels[2] == ((mask & ROLL_LIMIT_POS_ACTIVE) ? 0U : 1U));
        test_gpiob.IDR &= ~GPIO_PIN_4;
        test_gpiod.IDR &= ~GPIO_PIN_7;
        LimitSwitch_ReadStates(levels);
        assert(levels[1] == 0U && levels[3] == 0U);
        assert(LimitSwitch_ReadRollActiveMask() == mask);
    }
    LimitSwitch_ReadStates(NULL);
    assert(Stepper_CopyState(NULL) == 0U);
}

static void test_start_matrix(void)
{
    unsigned mask;
    int command;
    for (mask = 0U; mask <= ROLL_LIMIT_ALL_ACTIVE; ++mask)
    {
        for (command = -100; command <= 100; command += 100)
        {
            uint8_t direction_mask = command > 0 ? ROLL_LIMIT_POS_ACTIVE : ROLL_LIMIT_NEG_ACTIVE;
            unsigned allowed = command != 0 && (mask & direction_mask) == 0U;
            reset((uint8_t)mask);
            Stepper_SetContinuousSignedRpm(command);
            assert(state().requested_rpm == command);
            assert(state().running == allowed);
            assert(starts == allowed);
            assert(state().blocked_mask == mask);
            assert(state().limit_stop_count == 0U);
            if (allowed)
            {
                assert(state().pulse_rpm == (command > 0 ? 30 : -30));
                assert(!!(test_gpioe.ODR & GPIO_PIN_13) == (command > 0));
            }
        }
    }
}

static void test_irq_stop_and_escape(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        uint8_t mask = sign > 0 ? ROLL_LIMIT_POS_ACTIVE : ROLL_LIMIT_NEG_ACTIVE;
        int32_t stopped_position;
        unsigned before;
        reset(0U);
        Stepper_SetContinuousSignedRpm(sign * 100);
        irq();
        set_limits(mask);
        irq();
        assert(state().running == 0U);
        assert(state().blocked_mask == mask);
        assert(state().last_stop_reason == STEPPER_STOP_LIMIT);
        assert(state().limit_stop_count == 1U);
        assert(state().requested_rpm == sign * 100);
        stopped_position = state().current_pos;
        assert(stopped_position == sign * 2);
        before = starts;
        Stepper_SetContinuousSignedRpm(sign * 100);
        Stepper_SetContinuousSignedRpm(sign * 100);
        irq();
        assert(starts == before && state().running == 0U);
        assert(state().current_pos == stopped_position);
        assert(state().limit_stop_count == 1U);
        Stepper_SetContinuousSignedRpm(-sign * 100);
        assert(state().running == 1U && starts == before + 1U);
        irq();
        assert(state().current_pos == stopped_position - sign);
        assert(state().blocked_mask == mask);
    }
}

static void test_repeated_command_reads_current_gpio(void)
{
    reset(0U);
    Stepper_SetContinuousSignedRpm(100);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    Stepper_SetContinuousSignedRpm(100);
    assert(state().running == 0U && state().limit_stop_count == 1U);
    assert(state().last_stop_reason == STEPPER_STOP_LIMIT);
    assert(starts == 1U);
    reset(0U);
    Stepper_SetContinuousSignedRpm(-100);
    set_limits(ROLL_LIMIT_NEG_ACTIVE);
    service(1U);
    assert(state().running == 0U && state().limit_stop_count == 1U);
}

static void test_release_while_stopped(void)
{
    reset(ROLL_LIMIT_POS_ACTIVE);
    Stepper_SetContinuousSignedRpm(100);
    set_limits(0U);
    service(10U);
    assert(state().raw_limit_mask == 0U && state().blocked_mask == ROLL_LIMIT_POS_ACTIVE);
    service(10U + STEPPER_LIMIT_RELEASE_MS - 1U);
    Stepper_SetContinuousSignedRpm(100);
    assert(state().running == 0U);
    service(10U + STEPPER_LIMIT_RELEASE_MS);
    assert(state().blocked_mask == 0U && state().running == 0U);
    Stepper_SetContinuousSignedRpm(100);
    assert(state().running == 1U);
}

static void test_release_independent_and_bounce(void)
{
    reset(ROLL_LIMIT_ALL_ACTIVE);
    set_limits(ROLL_LIMIT_NEG_ACTIVE);
    service(100U);
    service(100U + STEPPER_LIMIT_RELEASE_MS);
    assert(state().blocked_mask == ROLL_LIMIT_NEG_ACTIVE);
    set_limits(0U);
    service(200U);
    set_limits(ROLL_LIMIT_NEG_ACTIVE);
    Stepper_SetContinuousSignedRpm(-100);
    assert(state().running == 0U);
    set_limits(0U);
    service(201U);
    service(201U + STEPPER_LIMIT_RELEASE_MS - 1U);
    assert(state().blocked_mask == ROLL_LIMIT_NEG_ACTIVE);
    service(201U + STEPPER_LIMIT_RELEASE_MS);
    assert(state().blocked_mask == 0U);
    Stepper_SetContinuousSignedRpm(-100);
    assert(state().running == 1U);
}

static void test_irq_cancels_release(void)
{
    reset(ROLL_LIMIT_POS_ACTIVE);
    Stepper_SetContinuousSignedRpm(-100);
    set_limits(0U);
    service(10U);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    irq();
    assert(state().running == 1U);
    set_limits(0U);
    service(30U);
    assert(state().blocked_mask == ROLL_LIMIT_POS_ACTIVE);
    service(30U + STEPPER_LIMIT_RELEASE_MS);
    assert(state().blocked_mask == 0U);
}

static void test_release_wrap(void)
{
    reset(ROLL_LIMIT_POS_ACTIVE);
    set_limits(0U);
    service(UINT32_MAX - 9U);
    service(9U);
    assert(state().blocked_mask == ROLL_LIMIT_POS_ACTIVE);
    service(10U);
    assert(state().blocked_mask == 0U);
}

static void test_finite_position_and_absolute_direction(void)
{
    int i;
    reset(0U);
    Stepper_MoveRelative(4, 100U);
    assert(state().target_pos == 4);
    for (i = 0; i < 4; ++i) { irq(); }
    assert(state().running == 0U && state().current_pos == 4);
    assert(state().last_stop_reason == STEPPER_STOP_COMPLETED);
    assert(state().target_pos == 4 && state().requested_rpm == 100);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    Stepper_MoveAbsolute(2, 100U);
    assert(state().running == 1U && state().requested_rpm == -100);
    irq();
    irq();
    assert(state().current_pos == 2 && state().target_pos == 2);
    Stepper_MoveAbsolute(4, 100U);
    assert(state().running == 0U);
    set_limits(ROLL_LIMIT_NEG_ACTIVE);
    Stepper_MoveAbsolute(0, 100U);
    assert(state().running == 0U);
    assert(state().current_pos == 2);
}

static void test_finite_limit_cancels_remainder(void)
{
    int32_t stopped_position;
    reset(0U);
    Stepper_MoveRelative(20, 100U);
    irq();
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    irq();
    stopped_position = state().current_pos;
    assert(stopped_position == 2 && state().running == 0U);
    assert(state().target_pos == 20);
    set_limits(0U);
    service(100U);
    service(100U + STEPPER_LIMIT_RELEASE_MS);
    irq();
    assert(state().running == 0U && state().current_pos == stopped_position);
    assert(state().pulses_remaining == 0U);
}

static void test_start_race_and_hal_failure(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        reset(0U);
        inject_on_nop = 1U;
        injected_mask = sign > 0 ? ROLL_LIMIT_POS_ACTIVE : ROLL_LIMIT_NEG_ACTIVE;
        Stepper_MoveRelative(sign * 10, 100U);
        assert(inject_on_nop == 0U && starts == 0U);
        assert(state().running == 0U && state().blocked_mask == injected_mask);
        assert(state().last_stop_reason == STEPPER_STOP_LIMIT);
        assert(state().limit_stop_count == 0U);
    }
    reset(0U);
    start_result = HAL_ERROR;
    Stepper_SetContinuousSignedRpm(100);
    assert(starts == 1U && state().running == 0U);
    assert(state().last_stop_reason == STEPPER_STOP_START_FAILED);
    assert(state().requested_rpm == 100);
    start_result = HAL_OK;
    Stepper_SetContinuousSignedRpm(100);
    assert(starts == 2U && state().running == 1U);
}

static void test_zero_and_stop_semantics(void)
{
    reset(0U);
    Stepper_SetContinuousSignedRpm(100);
    Stepper_MoveRelative(0, 100U);
    assert(state().running == 0U && state().requested_rpm == 0);
    Stepper_SetContinuousSignedRpm(100);
    Stepper_MoveRelative(10, 0U);
    assert(state().running == 0U && state().requested_rpm == 0);
    Stepper_SetContinuousSignedRpm(100);
    Stepper_MoveAbsolute(state().current_pos, 100U);
    assert(state().running == 0U && state().requested_rpm == 0);
    Stepper_SetContinuousSignedRpm(100);
    Stepper_Stop();
    assert(state().running == 0U && state().requested_rpm == 0);
    assert(state().last_stop_reason == STEPPER_STOP_COMMAND);
    Stepper_SetContinuousSignedRpm(100);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    irq();
    Stepper_SetContinuousSignedRpm(0);
    Stepper_SetContinuousSignedRpm(0);
    assert(state().requested_rpm == 0 && state().blocked_mask == ROLL_LIMIT_POS_ACTIVE);
    assert(state().last_stop_reason == STEPPER_STOP_LIMIT);
    assert(state().limit_stop_count == 1U);
}

static void test_integer_ranges(void)
{
    reset(0U);
    Stepper_SetContinuousSignedRpm(INT32_MIN);
    assert(state().running == 1U && state().requested_rpm == -800);
    Stepper_SetContinuousSignedRpm(INT32_MAX);
    assert(state().running == 1U && state().requested_rpm == 800);
    reset(0U);
    Stepper_MoveRelative(INT32_MIN, 100U);
    assert(state().running == 1U && state().target_pos == INT32_MIN);
    assert(state().pulses_remaining == 2147483648U);
    irq();
    assert(state().current_pos == -1);
    Stepper_MoveAbsolute(INT32_MAX, 100U);
    assert(state().running == 0U && state().last_stop_reason == STEPPER_STOP_RANGE);
    assert(state().current_pos == -1);
    reset(0U);
    Stepper_MoveRelative(2, 100U);
    irq();
    Stepper_MoveRelative(INT32_MAX, 100U);
    assert(state().running == 0U && state().last_stop_reason == STEPPER_STOP_RANGE);
    assert(state().current_pos == 1);
    reset(0U);
    Stepper_SetContinuousSignedRpm(1);
    assert(state().requested_rpm == 1 && state().pulse_rpm == 30);
    Stepper_SetContinuousSignedRpm(-1);
    assert(state().requested_rpm == -1 && state().pulse_rpm == -30);
}

static void test_release_arm_contract(void)
{
    reset(0U);
    assert(Stepper_ArmReleaseStop(0U) == 0U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_ALL_ACTIVE) == 0U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_POS_ACTIVE) == 0U);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_NEG_ACTIVE) == 0U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_POS_ACTIVE) == 1U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_POS_ACTIVE) == 1U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_NEG_ACTIVE) == 0U);
    assert(state().release_seq == 0U);
    Stepper_Stop();
    assert(state().release_armed_mask == ROLL_LIMIT_POS_ACTIVE);
    Stepper_SetContinuousSignedRpm(-100);
    assert(state().running == 1U && state().release_armed_mask == ROLL_LIMIT_POS_ACTIVE);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_POS_ACTIVE) == 1U);
    Stepper_DisarmReleaseStop();
    assert(state().release_armed_mask == 0U);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_POS_ACTIVE) == 0U);
    set_limits(0U);
    irq();
    assert(state().running == 1U && state().release_seq == 0U);
}

static void test_release_capture_and_history(void)
{
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        uint8_t mask = sign > 0 ? ROLL_LIMIT_NEG_ACTIVE : ROLL_LIMIT_POS_ACTIVE;
        reset(mask);
        assert(Stepper_ArmReleaseStop(mask) == 1U);
        Stepper_SetContinuousSignedRpm(sign * 100);
        irq();
        set_limits(0U);
        service(10U);
        assert(state().running == 1U && state().release_seq == 0U);
        irq();
        assert(state().running == 0U && state().last_stop_reason == STEPPER_STOP_RELEASE);
        assert(state().release_armed_mask == 0U && state().release_mask == mask);
        assert(state().release_seq == 1U && state().release_pos == sign * 2);
        assert(state().current_pos == electrical_position && state().blocked_mask == mask);
        Stepper_Stop();
        Stepper_DisarmReleaseStop();
        Stepper_SetContinuousSignedRpm(0);
        assert(state().release_seq == 1U && state().release_pos == sign * 2);
        assert(state().last_stop_reason == STEPPER_STOP_RELEASE);
        /* 释放抖动也不重新出脉冲，历史位置不随后续去抖更新。 */
        set_limits(mask);
        service(20U);
        set_limits(0U);
        service(30U);
        service(30U + STEPPER_LIMIT_RELEASE_MS);
        assert(state().running == 0U && state().blocked_mask == 0U);
        assert(state().release_seq == 1U && state().release_pos == sign * 2);
        set_limits(mask);
        assert(Stepper_ArmReleaseStop(mask) == 1U);
        Stepper_MoveRelative(sign * 5, 100U);
        set_limits(0U);
        irq();
        assert(state().running == 0U && state().release_seq == 2U);
        assert(state().release_pos == sign * 3 && state().release_mask == mask);
    }
}

static void test_release_start_race_and_limit_priority(void)
{
    reset(ROLL_LIMIT_NEG_ACTIVE);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_NEG_ACTIVE) == 1U);
    inject_on_nop = 1U;
    injected_mask = 0U;
    Stepper_SetContinuousSignedRpm(100);
    assert(state().running == 0U && starts == 0U && state().release_seq == 0U);
    assert(state().last_stop_reason == STEPPER_STOP_START_FAILED);
    reset(ROLL_LIMIT_NEG_ACTIVE);
    assert(Stepper_ArmReleaseStop(ROLL_LIMIT_NEG_ACTIVE) == 1U);
    Stepper_SetContinuousSignedRpm(100);
    set_limits(ROLL_LIMIT_POS_ACTIVE);
    irq();
    assert(state().running == 0U && state().last_stop_reason == STEPPER_STOP_LIMIT);
    assert(state().release_seq == 0U && state().limit_stop_count == 1U);
}

static void test_stop_counts_pending_and_partial_pulses(void)
{
    unsigned pending, high;
    for (pending = 0U; pending <= 1U; ++pending)
    {
        for (high = 0U; high <= 1U; ++high)
        {
            reset(0U);
            Stepper_MoveRelative(10, 100U);
            if (pending != 0U) timer_ticks(active_arr + 1U);
            timer_ticks(high ? active_ccr + 1U : 1U);
            Stepper_Stop();
            assert(state().current_pos == (int32_t)(pending + high));
            assert(state().current_pos == electrical_position);
            assert(state().running == 0U);
            irq();
            assert(state().current_pos == electrical_position);
        }
    }
    /* ISR 晚到高半周期，停机产生额外下降沿，不能谎报恰好到了有限目标。 */
    reset(0U);
    Stepper_MoveRelative(1, 100U);
    timer_ticks(active_arr + 1U + active_ccr + 1U);
    dispatch_irq();
    assert(state().current_pos == 2 && state().target_pos == 1);
    assert(state().current_pos == electrical_position && state().running == 0U);
}

static void test_preload_commit_and_stop_phase(void)
{
    reset(0U);
    Stepper_MoveRelative(4, 100U);
    assert(state().current_pos == 0 && electrical_position == 0);
    assert(active_arr == 1249U && active_ccr == 625U);
    irq();
    assert(state().pulse_rpm == 30);
    assert(test_tim1.CCR2 < 400U && active_ccr == 625U);
    timer_ticks(400U);
    Stepper_Stop();
    assert(state().current_pos == 1 && electrical_position == 1);
    reset(0U);
    Stepper_MoveRelative(4, 100U);
    irq();
    irq();
    assert(state().pulse_rpm == 65 && active_ccr < 400U);
    timer_ticks(400U);
    Stepper_Stop();
    assert(state().current_pos == 3 && electrical_position == 3);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "gpio") == 0) { test_gpio(); }
    else if (strcmp(argv[1], "start_matrix") == 0) { test_start_matrix(); }
    else if (strcmp(argv[1], "irq_escape") == 0) { test_irq_stop_and_escape(); }
    else if (strcmp(argv[1], "repeat") == 0) { test_repeated_command_reads_current_gpio(); }
    else if (strcmp(argv[1], "release") == 0) { test_release_while_stopped(); }
    else if (strcmp(argv[1], "bounce") == 0) { test_release_independent_and_bounce(); }
    else if (strcmp(argv[1], "irq_bounce") == 0) { test_irq_cancels_release(); }
    else if (strcmp(argv[1], "wrap") == 0) { test_release_wrap(); }
    else if (strcmp(argv[1], "finite") == 0) { test_finite_position_and_absolute_direction(); }
    else if (strcmp(argv[1], "finite_limit") == 0) { test_finite_limit_cancels_remainder(); }
    else if (strcmp(argv[1], "start_race") == 0) { test_start_race_and_hal_failure(); }
    else if (strcmp(argv[1], "stop") == 0) { test_zero_and_stop_semantics(); }
    else if (strcmp(argv[1], "ranges") == 0) { test_integer_ranges(); }
    else if (strcmp(argv[1], "arm") == 0) { test_release_arm_contract(); }
    else if (strcmp(argv[1], "capture") == 0) { test_release_capture_and_history(); }
    else if (strcmp(argv[1], "release_race") == 0) { test_release_start_race_and_limit_priority(); }
    else if (strcmp(argv[1], "stop_phase") == 0) { test_stop_counts_pending_and_partial_pulses(); }
    else if (strcmp(argv[1], "preload") == 0) { test_preload_commit_and_stop_phase(); }
    else { assert(0); }
    assert(critical_depth == 0U);
    return 0;
}
"""


class StepperLimitHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("未找到用于运行假 HAL 测试的主机 gcc")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="stepper_host_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "stm32h7xx_hal.h").write_text(HAL_STUB, encoding="utf-8")
        (work / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
        (work / "FreeRTOS.h").write_text("/* 主机测试不启动调度器。 */\n", encoding="utf-8")
        (work / "task.h").write_text(TASK_STUB, encoding="utf-8")
        (work / "cmsis_os.h").write_text(CMSIS_STUB, encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("stepper_host.exe" if os.name == "nt" else "stepper_host")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(work), "-I", str(ROOT / "software/Glider-Core"),
             str(ROOT / "software/Glider-Core/stepper_driver.c"),
             str(ROOT / "software/Glider-Core/limit_switch.c"),
             str(work / "harness.c"), "-o", str(cls.binary)],
            capture_output=True, text=True, env=cls.env,
        )
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, name):
        result = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, env=self.env,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_gpio_mapping_preserves_legacy_active_low_levels(self):
        self.run_case("gpio")

    def test_all_startup_limit_and_command_combinations(self):
        self.run_case("start_matrix")

    def test_interrupt_stop_rejects_repeat_and_allows_reverse_escape(self):
        self.run_case("irq_escape")

    def test_repeated_command_and_service_detect_new_limit(self):
        self.run_case("repeat")

    def test_stopped_timer_releases_only_after_stable_interval(self):
        self.run_case("release")

    def test_limit_release_is_independent_and_restarts_after_bounce(self):
        self.run_case("bounce")

    def test_interrupt_observation_cancels_pending_release(self):
        self.run_case("irq_bounce")

    def test_release_timestamp_wraparound(self):
        self.run_case("wrap")

    def test_finite_completion_and_absolute_movement_direction(self):
        self.run_case("finite")

    def test_limit_cancels_remaining_finite_movement(self):
        self.run_case("finite_limit")

    def test_direction_wait_rechecks_limit_and_hal_failure_stays_stopped(self):
        self.run_case("start_race")

    def test_zero_commands_and_explicit_stop_preserve_protection(self):
        self.run_case("stop")

    def test_integer_extremes_and_requested_vs_configured_speed(self):
        self.run_case("ranges")

    def test_release_arm_validation_idempotence_and_explicit_disarm(self):
        self.run_case("arm")

    def test_release_capture_stops_once_and_preserves_history_through_debounce(self):
        self.run_case("capture")

    def test_release_before_start_fails_and_opposite_limit_has_priority(self):
        self.run_case("release_race")

    def test_stop_accounts_pending_update_and_high_phase_falling_edge(self):
        self.run_case("stop_phase")

    def test_pwm_preload_is_committed_at_update_and_start_has_no_fake_step(self):
        self.run_case("preload")


if __name__ == "__main__":
    unittest.main()

