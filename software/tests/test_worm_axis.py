"""编译真实蜗杆位置与回零模块，用模拟电机反馈验证新 pitch 机构的方向和参考状态。"""

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
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

volatile UMotor g_umotor;
volatile uint8_t g_limit_switch_states[4] = {1U, 1U, 1U, 1U};
static unsigned critical_depth;

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
    finish_homing();
    feed(80U + WORM_AXIS_REF_LOST_MS - 1U, -2, 0U);
    assert(state().reference_kind == WORM_REF_HOMED && state().data_fresh == 0U);
    feed(80U + WORM_AXIS_REF_LOST_MS, -2, 0U);
    assert(state().reference_kind == WORM_REF_NONE);
    assert(state().reference_lost_count == 1U);
    assert(state().home_state == WORM_HOME_IDLE);
    assert(homing_rpm(0U) == 0);
    feed(80U + WORM_AXIS_REF_LOST_MS + 20U, -100, 1U);
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


if __name__ == "__main__":
    unittest.main()

