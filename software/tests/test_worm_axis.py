"""编译真实双轴校准模块，用模拟反馈覆盖方向、释放捕获、一次启动及失败联锁。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]

TASK_STUB = r"""
#ifndef TASK_H
#define TASK_H
void test_enter_critical(void);
void test_exit_critical(void);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
#endif
"""

HARNESS = r"""
#include "worm_axis.h"
#include "motor_driver.h"
#include "stepper_driver.h"
#include "limit_switch.h"
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

volatile UMotor g_umotor;
volatile uint8_t g_limit_switch_states[4] = {1U, 1U, 1U, 1U};
static unsigned critical_depth;
static StepperState roll = {.running = 1U};
static uint8_t roll_snapshot_valid = 1U;

uint8_t Stepper_CopyState(StepperState *out)
{
    assert(out != NULL);
    *out = roll;
    return roll_snapshot_valid;
}

void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void)
{
    assert(critical_depth > 0U);
    --critical_depth;
}

static WormAxisState state(void)
{
    WormAxisState out;
    WormAxis_CopyState(&out);
    assert(critical_depth == 0U);
    return out;
}

static void near_value(float actual, float expected, float tolerance)
{
    float delta = actual - expected;
    assert(delta <= tolerance && delta >= -tolerance);
}

static void feed(uint32_t now_ms, int32_t raw_pulse, uint32_t valid)
{
    g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].debug.pos_signed_32 = raw_pulse;
    g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].valid = valid;
    WormAxis_Step(now_ms);
    assert(state().pulse == raw_pulse);
    near_value(state().rev, (float)raw_pulse / WORM_PULSES_PER_REV, 0.0001f);
}

static int32_t homing_rpm(uint8_t expected_active)
{
    int32_t rpm = INT32_MIN;
    assert(WormAxis_GetHomingWormRpm(&rpm) == expected_active);
    assert(WormAxis_GetHomingWormRpm(NULL) == expected_active);
    return rpm;
}

static void begin_seek(void)
{
    feed(0U, -100000, 1U);
    WormAxis_RequestHoming();
    feed(20U, -100000, 1U);
    assert(state().home_state == WORM_HOME_SEEK);
    assert(homing_rpm(1U) == 0);
    feed(40U, -90000, 1U);
    assert(homing_rpm(1U) == -350);
}

static void finish_homing(void)
{
    begin_seek();
    /* 新机构机头开关接 PB4，即第 1 路；正硬件 RPM 朝机头运动。 */
    g_limit_switch_states[1] = 0U;
    feed(60U, 10, 1U);
    assert(state().home_state == WORM_HOME_BACKOFF);
    assert(homing_rpm(1U) == 35);
    g_limit_switch_states[1] = 1U;
    feed(80U, -2, 1U);
    assert(state().home_state == WORM_HOME_DONE);
    assert(state().home_fail == WORM_HOME_FAIL_NONE);
    assert(state().reference_kind == WORM_REF_HOMED);
    assert(state().pulse_home_front == -2);
    assert(state().pulse_min_seen == -281323);
    assert(state().pulse_max_seen == -2);
    assert(state().span_pulses == 281321 && state().span_valid == 1U);
    near_value(state().stroke_percent, 0.0f, 0.0001f);
    assert(state().home_done_count == 1U);
    assert(homing_rpm(0U) == 0);
}

static WormCalibrationOutput output(void)
{
    WormCalibrationOutput out;
    assert(WormAxis_GetCalibrationOutput(&out) == state().calib_hold);
    assert(WormAxis_GetCalibrationOutput(NULL) == state().calib_hold);
    assert(out.calib_state == state().calib_state);
    return out;
}

static void stopped(void)
{
    assert(state().calib_hold == 1U);
    assert(output().pitch_rpm == 0);
    assert(output().roll_action == WORM_ROLL_ACTION_STOP);
    assert(output().roll_rpm == 0);
}

/* 模拟 ServoTask 的一次命令及驱动快照，不在测试中实现第二套校准状态机。 */
static void acknowledge_backoff(void)
{
    WormCalibrationOutput first = output();
    assert(first.roll_action == WORM_ROLL_ACTION_START_BACKOFF);
    assert(output().roll_action == first.roll_action); /* 查询不得消费 START。 */
    roll.running = 1U;
    roll.last_stop_reason = STEPPER_STOP_NONE;
    roll.release_armed_mask = first.roll_release_arm_mask;
    WormAxis_ConfirmRollCommand(1U);
    assert(output().roll_action == WORM_ROLL_ACTION_KEEP_BACKOFF);
}

static void capture_release(uint8_t mask, int32_t pos)
{
    roll.current_pos = pos;
    roll.release_pos = pos;
    roll.release_mask = mask;
    roll.release_seq++;
    roll.release_armed_mask = 0U;
    roll.running = 0U;
    roll.last_stop_reason = STEPPER_STOP_RELEASE;
    roll.raw_limit_mask = 0U;
    roll.blocked_mask = mask;
}

static uint32_t measure_roll(uint32_t now, int32_t negative, int32_t positive)
{
    roll.raw_limit_mask = roll.blocked_mask = ROLL_LIMIT_NEG_ACTIVE;
    roll.running = 0U;
    roll.last_stop_reason = STEPPER_STOP_LIMIT;
    feed(now += 20U, -2, 1U);
    assert(state().roll.home_state == WORM_ROLL_HOME_BACKOFF_NEG);
    assert(output().roll_rpm == WORM_ROLL_BACKOFF_RPM);
    assert(output().roll_release_arm_mask == ROLL_LIMIT_NEG_ACTIVE);
    acknowledge_backoff();
    capture_release(ROLL_LIMIT_NEG_ACTIVE, negative);
    feed(now += 20U, -2, 1U);
    assert(output().roll_action == WORM_ROLL_ACTION_STOP);
    assert(state().roll.home_state == WORM_ROLL_HOME_BACKOFF_NEG);
    /* STOP 覆写停止原因后仍使用已捕获位置；不能把 seq 历史当新一轮释放。 */
    roll.last_stop_reason = STEPPER_STOP_COMMAND;
    roll.blocked_mask = 0U;
    feed(now += 20U, -2, 1U);
    assert(state().roll.pulse_min == negative);
    assert(state().roll.home_state == WORM_ROLL_HOME_SEEK_POS);
    assert(output().roll_rpm == WORM_ROLL_SEEK_RPM);

    roll.raw_limit_mask = roll.blocked_mask = ROLL_LIMIT_POS_ACTIVE;
    roll.last_stop_reason = STEPPER_STOP_LIMIT;
    feed(now += 20U, -2, 1U);
    assert(state().roll.home_state == WORM_ROLL_HOME_BACKOFF_POS);
    assert(output().roll_rpm == -WORM_ROLL_BACKOFF_RPM);
    assert(output().roll_release_arm_mask == ROLL_LIMIT_POS_ACTIVE);
    acknowledge_backoff();
    capture_release(ROLL_LIMIT_POS_ACTIVE, positive);
    feed(now += 20U, -2, 1U);
    assert(output().roll_action == WORM_ROLL_ACTION_STOP);
    roll.blocked_mask = 0U;
    feed(now += 20U, -2, 1U);
    return now;
}

static uint32_t complete_center(uint32_t now, uint8_t immediate)
{
    WormCalibrationOutput start = output();
    assert(start.roll_action == WORM_ROLL_ACTION_START_CENTER);
    assert(start.roll_rpm == WORM_ROLL_CENTER_RPM);
    assert(output().roll_action == WORM_ROLL_ACTION_START_CENTER);
    roll.target_pos = start.roll_center_pos;
    roll.running = 1U;
    roll.last_stop_reason = STEPPER_STOP_NONE;
    if (immediate)
    {
        /* 距离很短时在 20ms 内完成，也必须消费明确的启动回执。 */
        roll.running = 0U;
        roll.current_pos = roll.target_pos;
        roll.last_stop_reason = STEPPER_STOP_COMPLETED;
    }
    WormAxis_ConfirmRollCommand(1U);
    assert(output().roll_action == WORM_ROLL_ACTION_KEEP_CENTER);
    if (!immediate)
    {
        feed(now += 20U, -2, 1U);
        assert(output().roll_action == WORM_ROLL_ACTION_KEEP_CENTER);
        roll.running = 0U;
        roll.current_pos = roll.target_pos;
        roll.last_stop_reason = STEPPER_STOP_COMPLETED;
    }
    feed(now += 20U, -2, 1U);
    assert(state().roll.home_state == WORM_ROLL_HOME_DONE);
    assert(state().roll.valid == 1U);
    return now;
}

static void test_power_on_endpoints(void)
{
    WormAxis_CopyState(NULL);
    feed(100U, -2, 1U);
    assert(state().reference_kind == WORM_REF_POWER_ON);
    assert(state().data_fresh == 1U && state().span_valid == 0U);
    feed(120U, -281323, 1U);
    assert(state().span_pulses == 281321 && state().span_valid == 1U);
    assert(state().pulse_min_seen == -281323 && state().pulse_max_seen == -2);
    near_value(state().stroke_percent, 100.0f, 0.0001f);
    feed(140U, -2, 1U);
    near_value(state().stroke_percent, 0.0f, 0.0001f);
    feed(160U, -140662, 1U);
    near_value(state().stroke_percent, 50.0f, 0.0003f);
    feed(180U, -377, 1U);
    near_value(state().stroke_percent, 0.13329968f, 0.000001f);
    near_value(state().mm_from_front,
               state().stroke_percent * WORM_STROKE_MM / 100.0f, 0.0001f);
    assert(state().reference_kind == WORM_REF_POWER_ON);
    assert(state().home_done_count == 0U);
}

static void test_span_threshold(void)
{
    assert(WORM_SPAN_MIN_PULSES == 253188);
    feed(100U, -2, 1U);
    feed(120U, -2 - WORM_SPAN_MIN_PULSES + 1, 1U);
    assert(state().span_valid == 0U);
    assert(state().stroke_percent == 0.0f && state().mm_from_front == 0.0f);
    feed(140U, -2 - WORM_SPAN_MIN_PULSES, 1U);
    assert(state().span_valid == 1U);
    near_value(state().stroke_percent, 100.0f, 0.0001f);
}

static void test_front_switch_homing(void)
{
    finish_homing();
    feed(100U, -281323, 1U);
    near_value(state().stroke_percent, 100.0f, 0.0001f);
}

static void test_rear_switch_is_not_home(void)
{
    begin_seek();
    /* PD7 是新机构机尾；触发它不能使机头回零进入退出段。 */
    g_limit_switch_states[3] = 0U;
    feed(60U, -281323, 1U);
    assert(state().home_state == WORM_HOME_SEEK);
    assert(homing_rpm(1U) == -350);
    assert(state().home_done_count == 0U);
}

static void test_start_with_front_already_pressed(void)
{
    feed(0U, 10, 1U);
    g_limit_switch_states[1] = 0U;
    WormAxis_RequestHoming();
    feed(20U, 10, 1U);
    assert(state().home_state == WORM_HOME_SEEK && homing_rpm(1U) == 0);
    feed(40U, 10, 1U);
    assert(state().home_state == WORM_HOME_BACKOFF && homing_rpm(1U) == 35);
    feed(60U, 2, 1U);
    assert(state().home_state == WORM_HOME_BACKOFF && homing_rpm(1U) == 35);
    g_limit_switch_states[1] = 1U;
    feed(80U, -2, 1U);
    assert(state().home_state == WORM_HOME_DONE && homing_rpm(0U) == 0);
    assert(state().pulse_home_front == -2);
}

static void test_cancel(uint8_t during_backoff)
{
    begin_seek();
    if (during_backoff)
    {
        g_limit_switch_states[1] = 0U;
        feed(60U, 10, 1U);
        assert(state().home_state == WORM_HOME_BACKOFF);
    }
    WormAxis_CancelHoming();
    feed(80U, 10, 1U);
    assert(state().home_state == WORM_HOME_FAILED);
    assert(state().home_fail == WORM_HOME_FAIL_CANCELLED);
    assert(homing_rpm(0U) == 0 && state().home_done_count == 0U);
}

static void test_stale(uint8_t during_backoff)
{
    uint32_t last_valid_ms = 40U;
    int32_t expected_rpm = -350;
    begin_seek();
    if (during_backoff)
    {
        g_limit_switch_states[1] = 0U;
        feed(60U, 10, 1U);
        last_valid_ms = 60U;
        expected_rpm = 35;
    }
    feed(last_valid_ms + WORM_AXIS_STALE_MS - 1U, 10, 0U);
    assert(state().data_fresh == 1U && homing_rpm(1U) == expected_rpm);
    feed(last_valid_ms + WORM_AXIS_STALE_MS, 10, 0U);
    assert(state().data_fresh == 0U);
    assert(state().home_state == WORM_HOME_FAILED);
    assert(state().home_fail == WORM_HOME_FAIL_DATA_STALE);
    assert(homing_rpm(0U) == 0 && state().home_done_count == 0U);
}

static void test_homed_endpoints_do_not_drift(void)
{
    finish_homing();
    feed(100U, 100, 1U);
    assert(state().pulse_min_seen == -281323 && state().pulse_max_seen == -2);
    feed(120U, -281500, 1U);
    assert(state().pulse_min_seen == -281323 && state().pulse_max_seen == -2);
    assert(state().pulse_home_front == -2 && state().span_pulses == 281321);
    feed(140U, -377, 1U);
    near_value(state().stroke_percent, 0.13329968f, 0.000001f);
    assert(state().reference_kind == WORM_REF_HOMED);
}

static void test_reference_loss_and_recovery(void)
{
    uint32_t done;
    finish_homing();
    done = complete_center(measure_roll(80U, -1000, 1000), 1U);
    assert(state().calib_state == WORM_CALIB_DONE);
    feed(done + WORM_AXIS_REF_LOST_MS - 1U, -2, 0U);
    assert(state().reference_kind == WORM_REF_HOMED && state().data_fresh == 0U);
    feed(done + WORM_AXIS_REF_LOST_MS, -2, 0U);
    assert(state().reference_kind == WORM_REF_NONE);
    assert(state().reference_lost_count == 1U);
    assert(state().home_state == WORM_HOME_IDLE);
    assert(homing_rpm(0U) == 0);
    feed(done + WORM_AXIS_REF_LOST_MS + 20U, -100, 1U);
    assert(state().reference_kind == WORM_REF_POWER_ON);
    assert(state().pulse_min_seen == -100 && state().pulse_max_seen == -100);
    assert(state().span_pulses == 0 && state().span_valid == 0U);
    assert(state().reference_lost_count == 1U);
}

static void test_home_endpoint_range(uint8_t exact_boundary)
{
    int32_t front = exact_boundary ? INT32_MIN + WORM_STROKE_PULSES : INT32_MIN;
    feed(0U, front, 1U);
    g_limit_switch_states[1] = 0U;
    WormAxis_RequestHoming();
    feed(20U, front, 1U);
    feed(40U, front, 1U);
    assert(state().home_state == WORM_HOME_BACKOFF);
    g_limit_switch_states[1] = 1U;
    feed(60U, front, 1U);
    assert(homing_rpm(0U) == 0);
    if (exact_boundary)
    {
        assert(state().home_state == WORM_HOME_DONE);
        assert(state().reference_kind == WORM_REF_HOMED);
        assert(state().pulse_min_seen == INT32_MIN);
        assert(state().pulse_max_seen == front);
        assert(state().span_pulses == WORM_STROKE_PULSES);
        assert(state().home_done_count == 1U);
        near_value(state().stroke_percent, 0.0f, 0.0001f);
        feed(80U, INT32_MIN, 1U);
        near_value(state().stroke_percent, 100.0f, 0.0001f);
    }
    else
    {
        assert(state().home_state == WORM_HOME_FAILED);
        assert(state().home_fail == WORM_HOME_FAIL_POSITION_RANGE);
        assert(state().reference_kind == WORM_REF_POWER_ON);
        assert(state().pulse_min_seen == front && state().pulse_max_seen == front);
        assert(state().home_done_count == 0U);
    }
}

static void test_span_overflow_is_invalid(void)
{
    feed(0U, INT32_MIN, 1U);
    feed(20U, INT32_MAX, 1U);
    assert(state().pulse_min_seen == INT32_MIN && state().pulse_max_seen == INT32_MAX);
    assert(state().span_valid == 0U && state().span_pulses == 0);
    assert(state().stroke_percent == 0.0f && state().mm_from_front == 0.0f);
}

static void test_homed_extreme_raw_pulse(void)
{
    float expected;
    finish_homing();
    /* 分子可能超出 int32；仍应保持符号正确且有限，不把越界诊断钳成端点。 */
    feed(100U, INT32_MAX, 1U);
    expected = (float)((-2.0 - (double)INT32_MAX) * 100.0 / 281321.0);
    near_value(state().stroke_percent, expected, 0.25f);
    feed(120U, INT32_MIN, 1U);
    expected = (float)((-2.0 - (double)INT32_MIN) * 100.0 / 281321.0);
    near_value(state().stroke_percent, expected, 0.25f);
    assert(state().pulse_min_seen == -281323 && state().pulse_max_seen == -2);
    assert(state().span_valid == 1U && state().reference_kind == WORM_REF_HOMED);
}

static void test_joint_success(uint8_t roll_first, uint8_t immediate)
{
    uint32_t now;
    if (roll_first) { begin_seek(); now = 40U; }
    else { finish_homing(); now = 80U; }
    now = measure_roll(now, -1000, 1000);
    assert(state().roll.valid == 1U && state().roll.span_pulses == 2000);
    assert(output().roll_center_pos == 0);
    near_value(state().roll.stroke_percent, 100.0f, 0.001f);
    now = complete_center(now, immediate);
    near_value(state().roll.stroke_percent, 50.0f, 0.001f);
    if (roll_first)
    {
        assert(state().calib_state == WORM_CALIB_RUNNING);
        assert(output().roll_action == WORM_ROLL_ACTION_STOP);
        assert(output().pitch_rpm == -WORM_HOME_SEEK_RPM);
        g_limit_switch_states[1] = 0U;
        feed(now += 20U, 10, 1U);
        g_limit_switch_states[1] = 1U;
        feed(now += 20U, -2, 1U);
    }
    assert(state().calib_state == WORM_CALIB_DONE);
    assert(state().calib_fail == WORM_CALIB_FAIL_NONE);
    stopped();
    WormAxis_ReleaseCalibrationHold();
    assert(state().calib_hold == 0U && WormAxis_GetCalibrationOutput(NULL) == 0U);
    /* 新一轮必须作废旧 roll 跨度，不能沿用上一轮计数。 */
    WormAxis_RequestCalibration();
    feed(now += 20U, -2, 1U);
    assert(state().roll.valid == 0U && state().roll.span_pulses == 0);
    assert(state().calib_state == WORM_CALIB_RUNNING);
    WormAxis_ReleaseCalibrationHold();
    assert(state().calib_hold == 1U);
}

static void test_roll_bad_span(int32_t negative, int32_t positive)
{
    finish_homing();
    (void)measure_roll(80U, negative, positive);
    assert(state().calib_state == WORM_CALIB_FAILED);
    assert(state().calib_fail == WORM_CALIB_FAIL_ROLL);
    assert(state().roll.home_fail == WORM_ROLL_FAIL_SPAN);
    assert(state().roll.valid == 0U);
    stopped();
}

static void test_integer_endpoints(void)
{
    uint32_t now;
    finish_homing();
    now = measure_roll(80U, INT32_MAX - 1001, INT32_MAX);
    assert(output().roll_center_pos == INT32_MAX - 501);
    now = complete_center(now, 1U);
    near_value(state().roll.stroke_percent, 100.0f * 500.0f / 1001.0f, 0.001f);
    /* 诊断保留范围外的实际估计，不钳成端点；分子用 int64 避免溢出。 */
    roll.current_pos = INT32_MIN;
    feed(now + 20U, -2, 1U);
    assert(state().roll.stroke_percent < -400000000.0f);
}

static void test_roll_faults(const char *kind)
{
    uint32_t now;
    finish_homing();
    if (strcmp(kind, "both_limits") == 0)
    {
        roll.raw_limit_mask = ROLL_LIMIT_ALL_ACTIVE;
        feed(100U, -2, 1U);
        assert(state().roll.home_fail == WORM_ROLL_FAIL_LIMIT_CONFLICT);
    }
    else if (strcmp(kind, "roll_snapshot") == 0)
    {
        roll_snapshot_valid = 0U;
        feed(100U, -2, 1U);
        assert(state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
    }
    else if (strcmp(kind, "roll_range") == 0)
    {
        roll.last_stop_reason = STEPPER_STOP_RANGE;
        feed(100U, -2, 1U);
        assert(state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
    }
    else if (strcmp(kind, "roll_seek_timeout") == 0)
    {
        WormAxis_RequestCalibration(); /* 不许刷新起始 20ms 的超时。 */
        feed(20U + WORM_ROLL_SEEK_TIMEOUT_MS, -2, 1U);
        assert(state().roll.home_fail == WORM_ROLL_FAIL_SEEK_TIMEOUT);
    }
    else
    {
        now = measure_roll(80U, -1000, 1000);
        if (strcmp(kind, "center_start_fail") == 0)
        {
            WormAxis_ConfirmRollCommand(0U);
            assert(state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
        }
        else
        {
            roll.target_pos = output().roll_center_pos;
            roll.running = 1U;
            roll.last_stop_reason = STEPPER_STOP_NONE;
            WormAxis_ConfirmRollCommand(1U);
            if (strcmp(kind, "center_timeout") == 0)
            {
                feed(now + WORM_ROLL_CENTER_TIMEOUT_MS, -2, 1U);
                assert(state().roll.home_fail == WORM_ROLL_FAIL_CENTER_TIMEOUT);
            }
            else
            {
                roll.running = 0U;
                roll.current_pos = roll.target_pos;
                roll.last_stop_reason = STEPPER_STOP_LIMIT;
                if (strcmp(kind, "center_wrong_count") == 0)
                {
                    roll.last_stop_reason = STEPPER_STOP_COMPLETED;
                    roll.current_pos++;
                }
                feed(now + 20U, -2, 1U);
                assert(state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
            }
        }
    }
    assert(state().calib_state == WORM_CALIB_FAILED);
    assert(state().roll.valid == 0U);
    stopped();
}

static void test_release_faults(const char *kind)
{
    finish_homing();
    roll.raw_limit_mask = roll.blocked_mask = ROLL_LIMIT_NEG_ACTIVE;
    roll.running = 0U;
    roll.last_stop_reason = STEPPER_STOP_LIMIT;
    roll.release_seq = UINT32_MAX; /* 本次捕获允许回绕，不能用 > 判断事件。 */
    feed(100U, -2, 1U);
    if (strcmp(kind, "backoff_start_fail") == 0)
    {
        WormAxis_ConfirmRollCommand(0U);
    }
    else
    {
        acknowledge_backoff();
        if (strcmp(kind, "backoff_timeout") == 0)
        {
            feed(100U + WORM_ROLL_BACKOFF_TIMEOUT_MS, -2, 1U);
            assert(state().roll.home_fail == WORM_ROLL_FAIL_BACKOFF_TIMEOUT);
        }
        else if (strcmp(kind, "release_no_event") == 0)
        {
            roll.running = 0U;
            roll.last_stop_reason = STEPPER_STOP_RELEASE;
            feed(120U, -2, 1U);
        }
        else
        {
            capture_release(ROLL_LIMIT_NEG_ACTIVE, -1000);
            assert(roll.release_seq == 0U);
            if (strcmp(kind, "release_wrong_mask") == 0)
            {
                roll.release_mask = ROLL_LIMIT_POS_ACTIVE;
            }
            feed(120U, -2, 1U);
            if (strcmp(kind, "release_moved") == 0)
            {
                roll.current_pos++;
                feed(140U, -2, 1U);
            }
            else if (strcmp(kind, "release_bounce") == 0)
            {
                roll.raw_limit_mask = ROLL_LIMIT_NEG_ACTIVE;
                feed(140U, -2, 1U);
                assert(output().roll_action == WORM_ROLL_ACTION_STOP);
                feed(100U + WORM_ROLL_BACKOFF_TIMEOUT_MS, -2, 1U);
                assert(state().roll.home_fail == WORM_ROLL_FAIL_BACKOFF_TIMEOUT);
            }
        }
    }
    assert(state().calib_state == WORM_CALIB_FAILED);
    assert(state().roll.valid == 0U);
    stopped();
}

static void test_repeated_start_and_retry(void)
{
    begin_seek();
    WormAxis_RequestCalibration();
    feed(20U + WORM_HOME_SEEK_TIMEOUT_MS, -2, 1U);
    assert(state().home_fail == WORM_HOME_FAIL_SEEK_TIMEOUT);
    stopped();
    roll.last_stop_reason = STEPPER_STOP_START_FAILED;
    WormAxis_RequestCalibration();
    feed(40000U, -2, 1U);
    assert(state().calib_state == WORM_CALIB_RUNNING);
    stopped();
    /* 实际驱动在已停机时保留旧错误；第一条新 SEEK 必须获准重新尝试。 */
    roll.running = 0U;
    feed(40020U, -2, 1U);
    assert(state().calib_state == WORM_CALIB_RUNNING);
    assert(output().roll_action == WORM_ROLL_ACTION_CONTINUOUS);
    assert(output().roll_rpm == -WORM_ROLL_SEEK_RPM);
    /* 新一轮实际下发仍失败时，main 的即时回执必须同拍停 pitch。 */
    WormAxis_ConfirmRollCommand(0U);
    assert(state().calib_fail == WORM_CALIB_FAIL_ROLL);
    stopped();
}

static void test_cancel_and_pid(const char *kind)
{
    if (strcmp(kind, "cancel_pending") == 0)
    {
        WormAxis_RequestCalibration();
        WormAxis_CancelCalibration();
        WormAxis_RequestCalibration(); /* 待消费的取消优先。 */
        feed(0U, -2, 1U);
    }
    else
    {
        uint32_t now;
        finish_homing();
        now = measure_roll(80U, -1000, 1000);
        assert(state().roll.valid == 1U);
        if (strcmp(kind, "pid_cancel") == 0)
        {
            WormAxis_CancelCalibrationForPid(); /* 不等待下一拍。 */
            assert(state().calib_fail == WORM_CALIB_FAIL_PID);
        }
        else
        {
            WormAxis_CancelCalibration();
            feed(now + 20U, -2, 1U);
        }
    }
    assert(state().calib_state == WORM_CALIB_CANCELLED);
    assert(state().roll.valid == 0U);
    stopped();
}

static void test_pitch_fails_after_finishing(void)
{
    finish_homing();
    assert(state().calib_state == WORM_CALIB_RUNNING);
    feed(80U + WORM_AXIS_STALE_MS, -2, 0U);
    assert(state().home_fail == WORM_HOME_FAIL_DATA_STALE);
    assert(state().calib_fail == WORM_CALIB_FAIL_PITCH);
    assert(state().roll.home_fail == WORM_ROLL_FAIL_OTHER_AXIS);
    stopped();
}

static void test_timeout_wrap(void)
{
    uint32_t start = UINT32_MAX - 10U;
    feed(start - 20U, -2, 1U);
    WormAxis_RequestCalibration();
    feed(start, -2, 1U);
    feed(start + WORM_HOME_SEEK_TIMEOUT_MS - 1U, -2, 1U);
    assert(state().calib_state == WORM_CALIB_RUNNING);
    feed(start + WORM_HOME_SEEK_TIMEOUT_MS, -2, 1U);
    assert(state().home_fail == WORM_HOME_FAIL_SEEK_TIMEOUT);
    stopped();
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    assert(WORM_INSTALL_SIGN == -1);
    assert(WORM_FRONT_LIMIT_INDEX == 1U);
    assert(WORM_STROKE_PULSES == 281321);
    if (strcmp(argv[1], "endpoints") == 0) { test_power_on_endpoints(); }
    else if (strcmp(argv[1], "threshold") == 0) { test_span_threshold(); }
    else if (strcmp(argv[1], "homing") == 0) { test_front_switch_homing(); }
    else if (strcmp(argv[1], "rear") == 0) { test_rear_switch_is_not_home(); }
    else if (strcmp(argv[1], "pressed") == 0) { test_start_with_front_already_pressed(); }
    else if (strcmp(argv[1], "cancel_seek") == 0) { test_cancel(0U); }
    else if (strcmp(argv[1], "cancel_backoff") == 0) { test_cancel(1U); }
    else if (strcmp(argv[1], "stale_seek") == 0) { test_stale(0U); }
    else if (strcmp(argv[1], "stale_backoff") == 0) { test_stale(1U); }
    else if (strcmp(argv[1], "fixed") == 0) { test_homed_endpoints_do_not_drift(); }
    else if (strcmp(argv[1], "reference") == 0) { test_reference_loss_and_recovery(); }
    else if (strcmp(argv[1], "home_overflow") == 0) { test_home_endpoint_range(0U); }
    else if (strcmp(argv[1], "home_boundary") == 0) { test_home_endpoint_range(1U); }
    else if (strcmp(argv[1], "span_overflow") == 0) { test_span_overflow_is_invalid(); }
    else if (strcmp(argv[1], "raw_extreme") == 0) { test_homed_extreme_raw_pulse(); }
    else if (strcmp(argv[1], "joint_pitch_first") == 0) { test_joint_success(0U, 0U); }
    else if (strcmp(argv[1], "joint_roll_first") == 0) { test_joint_success(1U, 0U); }
    else if (strcmp(argv[1], "joint_immediate") == 0) { test_joint_success(0U, 1U); }
    else if (strcmp(argv[1], "small_span") == 0) { test_roll_bad_span(0, 31); }
    else if (strcmp(argv[1], "reverse_span") == 0) { test_roll_bad_span(1000, -1000); }
    else if (strcmp(argv[1], "huge_span") == 0) { test_roll_bad_span(INT32_MIN, INT32_MAX); }
    else if (strcmp(argv[1], "roll_integer") == 0) { test_integer_endpoints(); }
    else if (strcmp(argv[1], "retry") == 0) { test_repeated_start_and_retry(); }
    else if (strcmp(argv[1], "pitch_wait_stale") == 0) { test_pitch_fails_after_finishing(); }
    else if (strcmp(argv[1], "timeout_wrap") == 0) { test_timeout_wrap(); }
    else if (strncmp(argv[1], "cancel_", 7U) == 0 || strcmp(argv[1], "pid_cancel") == 0)
        { test_cancel_and_pid(argv[1]); }
    else if (strncmp(argv[1], "release_", 8U) == 0 || strncmp(argv[1], "backoff_", 8U) == 0)
        { test_release_faults(argv[1]); }
    else if (strncmp(argv[1], "center_", 7U) == 0 || strncmp(argv[1], "roll_", 5U) == 0 ||
             strcmp(argv[1], "both_limits") == 0) { test_roll_faults(argv[1]); }
    else { assert(0); }
    assert(critical_depth == 0U);
    return 0;
}
"""


class WormAxisHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("未找到用于运行蜗杆主机测试的 gcc")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="worm_host_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "FreeRTOS.h").write_text(
            "/* 主机测试不启动调度器。 */\n#include <stddef.h>\n", encoding="utf-8",
        )
        (work / "task.h").write_text(TASK_STUB, encoding="utf-8")
        (work / "stm32h7xx_hal.h").write_text(
            "#include <stdint.h>\n", encoding="utf-8",
        )
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("worm_host.exe" if os.name == "nt" else "worm_host")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(work), "-I", str(ROOT / "software/Glider-Core"),
             str(ROOT / "software/Glider-Core/worm_axis.c"),
             str(work / "harness.c"), "-o", str(cls.binary)],
            capture_output=True, text=True, env=cls.env,
        )
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, name):
        # 每个场景用独立进程模拟上电，避免内部静态参考状态相互污染。
        result = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, env=self.env,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_raw_pulses_and_front_to_rear_percentage(self):
        self.run_case("endpoints")

    def test_span_validity_threshold(self):
        self.run_case("threshold")

    def test_pb4_homing_release_establishes_raw_endpoints(self):
        self.run_case("homing")

    def test_pd7_does_not_trigger_front_homing(self):
        self.run_case("rear")

    def test_homing_starts_with_pb4_already_pressed(self):
        self.run_case("pressed")

    def test_cancel_stops_seek_and_backoff(self):
        for case in ("cancel_seek", "cancel_backoff"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_stale_feedback_stops_seek_and_backoff(self):
        for case in ("stale_seek", "stale_backoff"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_homed_reference_does_not_drift_with_observed_extremes(self):
        self.run_case("fixed")

    def test_reference_loss_discards_old_endpoints_after_recovery(self):
        self.run_case("reference")

    def test_homing_rejects_unrepresentable_raw_endpoint(self):
        self.run_case("home_overflow")

    def test_homing_accepts_endpoint_exactly_at_int32_min(self):
        self.run_case("home_boundary")

    def test_raw_span_above_int32_range_is_invalid(self):
        self.run_case("span_overflow")

    def test_homed_raw_extremes_keep_fraction_sign_without_overflow(self):
        self.run_case("raw_extreme")

    def test_both_axis_completion_orders_and_sub_tick_center_completion(self):
        for case in ("joint_pitch_first", "joint_roll_first", "joint_immediate"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_roll_span_is_remeasured_and_checked_in_64_bits(self):
        for case in ("small_span", "reverse_span", "huge_span", "roll_integer"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_driver_limit_and_center_errors_stop_both_axes(self):
        for case in ("both_limits", "roll_snapshot", "roll_range", "roll_seek_timeout",
                     "center_start_fail", "center_timeout", "center_wrong_count", "center_limit"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_backoff_requires_new_matching_release_and_stable_stop(self):
        for case in ("backoff_start_fail", "backoff_timeout", "release_no_event",
                     "release_wrong_mask", "release_moved", "release_bounce"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_repeated_request_keeps_timeout_and_failed_round_can_retry(self):
        self.run_case("retry")

    def test_cancel_including_pending_start_and_pid_holds_both_stopped(self):
        for case in ("cancel_pending", "cancel_center", "pid_cancel"):
            with self.subTest(case=case):
                self.run_case(case)

    def test_pitch_feedback_must_stay_fresh_until_roll_finishes(self):
        self.run_case("pitch_wait_stale")

    def test_phase_timeout_crosses_uint32_tick_wrap(self):
        self.run_case("timeout_wrap")


if __name__ == "__main__":
    unittest.main()

