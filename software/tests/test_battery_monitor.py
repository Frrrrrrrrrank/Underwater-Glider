"""用假 HAL 在主机执行真实电池模块，覆盖换算、告警、失效及时间回绕。"""

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
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { unsigned marker; } ADC_TypeDef;
typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;
extern ADC_TypeDef test_adc;
#define ADC1 (&test_adc)
#define ADC_CALIB_OFFSET_LINEARITY 3U
#define ADC_SINGLE_ENDED 1U
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
#endif
"""

TASK_STUB = r"""
#ifndef TASK_H
#define TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
extern unsigned critical_depth;
extern unsigned critical_entries;
#define taskENTER_CRITICAL() (++critical_depth, ++critical_entries)
#define taskEXIT_CRITICAL() (--critical_depth)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *, TickType_t);
#endif
"""

HARNESS = r"""
#include "battery_monitor.h"
#include "battery_task.h"
#include <assert.h>
#include <limits.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>

ADC_TypeDef test_adc;
unsigned critical_depth;
unsigned critical_entries;
static ADC_HandleTypeDef adc = { ADC1 };
static uint32_t tick;
static uint32_t input_raw;
static unsigned starts, stops, polls, reads, calibrations;
static unsigned fail_start_number;
static unsigned fail_kind;
static unsigned varied;
static HAL_StatusTypeDef calibration_result = HAL_OK;
static uint32_t calibration_duration_ms;
static jmp_buf task_escape;
static unsigned task_delays;
static uint32_t wake_targets[3];

uint32_t HAL_GetTick(void) { return tick; }
uint32_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(uint32_t *previous, uint32_t period)
{
  assert(period == 100U && critical_depth == 0U && task_delays < 3U);
  *previous += period;
  tick = *previous;
  wake_targets[task_delays++] = tick;
  if (task_delays == 3U) { longjmp(task_escape, 1); }
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t mode, uint32_t single)
{
  assert(h == &adc && critical_depth == 0U);
  assert(mode == ADC_CALIB_OFFSET_LINEARITY && single == ADC_SINGLE_ENDED);
  ++calibrations;
  tick += calibration_duration_ms;
  return calibration_result;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{
  assert(h == &adc && critical_depth == 0U);
  ++starts;
  return (starts == fail_start_number && fail_kind == 1U) ? HAL_BUSY : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout)
{
  assert(h == &adc && critical_depth == 0U && timeout == 2U);
  ++polls;
  return (starts == fail_start_number && fail_kind == 2U) ? HAL_TIMEOUT : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
  uint32_t raw = input_raw;
  assert(h == &adc && critical_depth == 0U);
  if (varied != 0U) { raw += reads % 8U; }
  ++reads;
  return raw;
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h)
{
  assert(h == &adc && critical_depth == 0U);
  ++stops;
  return (starts == fail_start_number && fail_kind == 3U) ? HAL_ERROR : HAL_OK;
}

static void set_mv(uint32_t mv)
{
  const uint64_t denominator = 3300ULL * 104700ULL;
  input_raw = (uint32_t)(((uint64_t)mv * 65535ULL * 4700ULL + denominator / 2U) /
                         denominator);
}
static BatteryState snapshot(void)
{
  BatteryState state;
  assert(BatteryMonitor_CopyState(&state));
  assert(critical_depth == 0U);
  return state;
}
static BatteryState poll_after(uint32_t delay_ms)
{
  tick += delay_ms;
  BatteryMonitor_Poll();
  return snapshot();
}
static void expect_close(uint32_t actual, uint32_t expected, uint32_t tolerance)
{
  assert(actual + tolerance >= expected && actual <= expected + tolerance);
}
static void fresh(uint32_t mv, uint32_t start_ms)
{
  tick = start_ms;
  set_mv(mv);
  assert(BatteryMonitor_Init(&adc) == HAL_OK);
  assert(snapshot().valid == 0U && snapshot().state == BATTERY_STATE_INIT);
}
static BatteryState reach_critical(void)
{
  BatteryState state;
  fresh(20900U, 0U);
  state = poll_after(0U);
  assert(state.valid == 1U && state.state == BATTERY_STATE_INIT);
  for (unsigned i = 1U; i <= 10U; ++i)
  {
    state = poll_after(100U);
    assert(state.state == (i < 10U ? BATTERY_STATE_INIT : BATTERY_STATE_CRITICAL));
  }
  return state;
}

int main(int argc, char **argv)
{
  BatteryState state;
  assert(argc >= 2);
  assert(!BatteryMonitor_CopyState(NULL));
  if (strcmp(argv[1], "ratio") == 0)
  {
    const uint32_t voltages[] = {0U, 21000U, 22000U, 29400U, 40000U};
    for (unsigned i = 0U; i < sizeof(voltages) / sizeof(voltages[0]); ++i)
    {
      unsigned entries_before = critical_entries;
      fresh(voltages[i], 0U);
      /* fresh 的两次快照各进入一次临界区，初始化本身不进入。 */
      assert(critical_entries == entries_before + 2U);
      state = poll_after(0U);
      assert(state.valid == 1U && state.sample_count == 1U && state.adc_raw == input_raw);
      expect_close(state.battery_mv, voltages[i], 1U);
      assert(state.filtered_mv == state.battery_mv);
    }
    fresh(24000U, 0U);
    varied = 1U;
    state = poll_after(0U);
    assert(state.adc_raw == input_raw + 4U);
    assert(starts == 48U && stops == starts && polls == starts && reads == starts);
  }
  else if (strcmp(argv[1], "initial_warning") == 0)
  {
    fresh(21900U, 0U);
    state = poll_after(0U);
    for (unsigned i = 1U; i <= 20U; ++i)
    {
      assert(state.valid == 1U && state.state == BATTERY_STATE_INIT);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_WARN);
  }
  else if (strcmp(argv[1], "initial_critical") == 0)
  {
    (void)reach_critical();
    fresh(0U, 0U);
    state = poll_after(0U);
    for (unsigned i = 0U; i < 10U; ++i) { state = poll_after(100U); }
    assert(state.state == BATTERY_STATE_CRITICAL && state.filtered_mv == 0U);
  }
  else if (strcmp(argv[1], "filter_and_hysteresis") == 0)
  {
    uint32_t crossed_at;
    fresh(29400U, 0U);
    state = poll_after(0U);
    assert(state.state == BATTERY_STATE_NORMAL);
    set_mv(21900U);
    state = poll_after(100U);
    expect_close(state.filtered_mv, 27525U, 2U);
    assert(state.state == BATTERY_STATE_NORMAL);
    while (state.filtered_mv > 22000U) { state = poll_after(100U); }
    crossed_at = tick;
    while ((uint32_t)(tick - crossed_at) < 2000U)
    {
      assert(state.state == BATTERY_STATE_NORMAL);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_WARN);
    set_mv(22400U);
    for (unsigned i = 0U; i < 80U; ++i) { state = poll_after(100U); }
    assert(state.state == BATTERY_STATE_WARN);
    set_mv(23000U);
    while (state.filtered_mv < 22500U) { state = poll_after(100U); }
    crossed_at = tick;
    while ((uint32_t)(tick - crossed_at) < 5000U)
    {
      assert(state.state == BATTERY_STATE_WARN);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_NORMAL);
  }
  else if (strcmp(argv[1], "critical_recovery") == 0)
  {
    uint32_t crossed_at;
    state = reach_critical();
    set_mv(21400U);
    for (unsigned i = 0U; i < 70U; ++i) { state = poll_after(100U); }
    assert(state.state == BATTERY_STATE_CRITICAL);
    set_mv(24000U);
    while (state.filtered_mv < 21500U) { state = poll_after(100U); }
    crossed_at = tick;
    while ((uint32_t)(tick - crossed_at) < 5000U)
    {
      assert(state.state == BATTERY_STATE_CRITICAL);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_WARN);
    crossed_at = tick;
    while ((uint32_t)(tick - crossed_at) < 5000U)
    {
      assert(state.state == BATTERY_STATE_WARN);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_NORMAL);
  }
  else if (strcmp(argv[1], "failure") == 0)
  {
    unsigned previous_starts, previous_reads;
    assert(argc == 3);
    fail_kind = (unsigned)strtoul(argv[2], NULL, 10);
    fresh(24000U, 0U);
    state = poll_after(0U);
    previous_starts = starts;
    previous_reads = reads;
    fail_start_number = starts + 3U;
    state = poll_after(100U);
    assert(state.valid == 0U && state.state == BATTERY_STATE_INVALID);
    assert(state.error_count == 1U && state.sample_count == 1U && state.last_sample_ms == 0U);
    assert(state.last_hal_status == (fail_kind == 1U ? HAL_BUSY :
                                    fail_kind == 2U ? HAL_TIMEOUT : HAL_ERROR));
    assert(starts == previous_starts + 3U && stops == starts);
    assert(reads == previous_reads + (fail_kind == 3U ? 3U : 2U));
    fail_start_number = 0U;
    set_mv(20900U);
    state = poll_after(100U);
    assert(state.valid == 1U && state.state == BATTERY_STATE_INIT);
    expect_close(state.filtered_mv, 20900U, 1U);
    assert(state.error_count == 1U && state.sample_count == 2U && state.last_hal_status == HAL_OK);
    for (unsigned i = 0U; i < 10U; ++i) { state = poll_after(100U); }
    assert(state.state == BATTERY_STATE_CRITICAL);
  }
  else if (strcmp(argv[1], "failure_resets_evidence") == 0)
  {
    fresh(21900U, 0U);
    state = poll_after(0U);
    for (unsigned i = 0U; i < 19U; ++i) { state = poll_after(100U); }
    fail_start_number = starts + 1U;
    fail_kind = 2U;
    state = poll_after(100U);
    assert(state.state == BATTERY_STATE_INVALID);
    state = poll_after(100U);
    for (unsigned i = 0U; i < 20U; ++i)
    {
      assert(state.state == BATTERY_STATE_INIT);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_WARN);
    set_mv(24000U);
    while (state.filtered_mv < 22500U) { state = poll_after(100U); }
    for (unsigned i = 0U; i < 49U; ++i) { state = poll_after(100U); }
    fail_start_number = starts + 1U;
    state = poll_after(100U);
    assert(state.state == BATTERY_STATE_INVALID);
    state = poll_after(100U);
    for (unsigned i = 0U; i < 50U; ++i)
    {
      assert(state.valid == 1U && state.state == BATTERY_STATE_WARN);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_NORMAL);
  }
  else if (strcmp(argv[1], "stale_and_gap") == 0)
  {
    fresh(21900U, 0U);
    state = poll_after(0U);
    for (unsigned i = 0U; i < 19U; ++i) { state = poll_after(100U); }
    tick += 500U;
    assert(snapshot().valid == 1U);
    tick += 1U;
    state = snapshot();
    assert(state.valid == 0U && state.state == BATTERY_STATE_INVALID);
    assert(g_battery_state.valid == 1U);
    state = poll_after(0U);
    for (unsigned i = 0U; i < 20U; ++i)
    {
      assert(state.state == BATTERY_STATE_INIT);
      state = poll_after(100U);
    }
    assert(state.state == BATTERY_STATE_WARN);
    set_mv(20900U);
    state = poll_after(501U);
    expect_close(state.filtered_mv, 20900U, 1U);
    assert(state.state == BATTERY_STATE_WARN);
    for (unsigned i = 0U; i < 10U; ++i) { state = poll_after(100U); }
    assert(state.state == BATTERY_STATE_CRITICAL);
  }
  else if (strcmp(argv[1], "wraparound") == 0)
  {
    fresh(20900U, UINT32_MAX - 499U);
    state = poll_after(0U);
    for (unsigned i = 0U; i < 10U; ++i)
    {
      assert(state.state == BATTERY_STATE_INIT);
      state = poll_after(100U);
    }
    assert(tick == 500U && state.state == BATTERY_STATE_CRITICAL);
    tick += 501U;
    assert(snapshot().valid == 0U);
  }
  else if (strcmp(argv[1], "calibration") == 0)
  {
    tick = UINT32_MAX - 499U;
    calibration_result = HAL_ERROR;
    calibration_duration_ms = 1200U;
    set_mv(24000U);
    assert(BatteryMonitor_Init(&adc) == HAL_OK);
    assert(calibrations == 0U && starts == 0U && critical_entries == 0U);
    state = poll_after(0U);
    assert(calibrations == 1U && tick == 700U && state.state == BATTERY_STATE_INVALID);
    for (unsigned i = 0U; i < 9U; ++i) { state = poll_after(100U); }
    assert(calibrations == 1U && state.valid == 0U && state.error_count == 1U);
    state = poll_after(100U);
    assert(calibrations == 2U && state.error_count == 2U && starts == 0U);
    calibration_result = HAL_OK;
    calibration_duration_ms = 0U;
    for (unsigned i = 0U; i < 9U; ++i) { state = poll_after(100U); }
    assert(calibrations == 2U && starts == 0U);
    state = poll_after(100U);
    assert(calibrations == 3U && state.valid == 1U && state.state == BATTERY_STATE_NORMAL);
    assert(state.error_count == 2U && state.sample_count == 1U && starts == 8U);
  }
  else if (strcmp(argv[1], "bad_input") == 0)
  {
    ADC_TypeDef wrong_instance;
    ADC_HandleTypeDef wrong_adc = { &wrong_instance };
    assert(BatteryMonitor_Init(NULL) == HAL_ERROR);
    BatteryMonitor_Poll();
    assert(snapshot().valid == 0U && calibrations == 0U && starts == 0U);
    assert(BatteryMonitor_Init(&wrong_adc) == HAL_ERROR);
    assert(calibrations == 0U);
    fresh(24000U, 0U);
    input_raw = 65535U;
    state = poll_after(0U);
    assert(state.valid == 0U && state.last_hal_status == HAL_ERROR && stops == 1U);
    input_raw = UINT32_MAX;
    state = poll_after(100U);
    assert(state.valid == 0U && state.error_count == 2U && stops == 2U);
  }
  else if (strcmp(argv[1], "task_cadence") == 0)
  {
    fresh(24000U, 0U);
    calibration_duration_ms = 1200U;
    if (setjmp(task_escape) == 0) { StartBatteryTask(NULL); }
    assert(wake_targets[0] == 1300U && wake_targets[1] == 1400U && wake_targets[2] == 1500U);
    state = snapshot();
    assert(calibrations == 1U && state.sample_count == 3U && state.last_sample_ms == 1400U);
  }
  else { assert(0); }
  assert(critical_depth == 0U);
  return 0;
}
"""


class BatteryMonitorHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("未找到用于执行假 HAL 测试的主机 gcc")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="battery_host_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "stm32h7xx_hal.h").write_text(HAL_STUB, encoding="utf-8")
        (work / "FreeRTOS.h").write_text("/* 主机测试不启动调度器。 */\n", encoding="utf-8")
        (work / "task.h").write_text(TASK_STUB, encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("battery_host.exe" if os.name == "nt" else "battery_host")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(work), "-I", str(ROOT / "software/Glider-Core"),
             "-I", str(ROOT / "software/Glider-App/tasks"),
             str(ROOT / "software/Glider-Core/battery_monitor.c"),
             str(ROOT / "software/Glider-App/tasks/battery_task.c"), str(work / "harness.c"),
             "-o", str(cls.binary)],
            capture_output=True, text=True, env=cls.env,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, *args):
        result = subprocess.run(
            [str(self.binary), *map(str, args)], capture_output=True, text=True, env=self.env,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_voltage_ratio_and_eight_sample_average(self):
        self.run_case("ratio")

    def test_initial_low_voltage_waits_without_claiming_normal(self):
        self.run_case("initial_warning")
        self.run_case("initial_critical")

    def test_filter_warning_duration_and_recovery_hysteresis(self):
        self.run_case("filter_and_hysteresis")

    def test_critical_requires_two_separate_recovery_intervals(self):
        self.run_case("critical_recovery")

    def test_start_poll_stop_failures_abort_batch_and_recover(self):
        for failure in (1, 2, 3):
            with self.subTest(failure=failure):
                self.run_case("failure", failure)

    def test_fault_breaks_continuous_timers_and_preserves_confirmed_alarm(self):
        self.run_case("failure_resets_evidence")

    def test_stale_snapshot_and_long_sampling_gap(self):
        self.run_case("stale_and_gap")

    def test_tick_wraparound(self):
        self.run_case("wraparound")

    def test_calibration_retry_is_limited_and_recovers_across_wrap(self):
        self.run_case("calibration")

    def test_wrong_adc_saturation_and_out_of_range(self):
        self.run_case("bad_input")

    def test_long_calibration_does_not_create_sampling_burst(self):
        self.run_case("task_cadence")


if __name__ == "__main__":
    unittest.main()
