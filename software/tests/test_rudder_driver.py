"""使用假 HAL 在主机运行真实尾舵驱动；不连接设备或启动网口。"""

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
typedef struct {
    uint32_t PSC, ARR, CR1, CCMR2, CCR4, CNT, EGR, SR;
} TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t D2CFGR, CFGR; } RCC_TypeDef;
extern TIM_TypeDef test_tim4;
extern RCC_TypeDef test_rcc;
#define TIM4 (&test_tim4)
#define RCC (&test_rcc)
#define RCC_D2CFGR_D2PPRE1_Pos 4U
#define RCC_D2CFGR_D2PPRE1 (7U << 4)
#define RCC_CFGR_TIMPRE (1U << 15)
#define TIM_CR1_CEN 1U
#define TIM_CR1_DIR (1U << 4)
#define TIM_CR1_CMS (3U << 5)
#define TIM_CCMR2_OC4PE (1U << 11)
#define TIM_CHANNEL_4 12U
#define TIM_EGR_UG 1U
#define TIM_FLAG_UPDATE 1U
#define SET_BIT(reg, mask) ((reg) |= (mask))
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((h)->Instance->CCR4 = (v))
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->Instance->CNT = (v))
#define __HAL_TIM_CLEAR_FLAG(h, mask) ((h)->Instance->SR &= ~(mask))
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
#endif
"""

TASK_STUB = r"""
#ifndef TASK_H
#define TASK_H
extern unsigned critical_depth;
extern unsigned critical_entries;
#define taskENTER_CRITICAL() (++critical_depth, ++critical_entries)
#define taskEXIT_CRITICAL() (--critical_depth)
#endif
"""

HARNESS = r"""
#include "rudder_driver.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

TIM_TypeDef test_tim4;
RCC_TypeDef test_rcc;
unsigned critical_depth;
unsigned critical_entries;
static uint32_t hclk = 200000000U;
static uint32_t pclk = 50000000U;
static unsigned starts;
static HAL_StatusTypeDef start_result = HAL_OK;

uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{
    assert(h->Instance == TIM4 && channel == TIM_CHANNEL_4);
    assert(critical_entries == 0U);
    assert(h->Instance->CCR4 == 1500U);
    assert(h->Instance->CNT == 0U);
    assert(h->Instance->EGR == TIM_EGR_UG);
    assert((h->Instance->CCMR2 & TIM_CCMR2_OC4PE) != 0U);
    ++starts;
    if (start_result == HAL_OK) { h->Instance->CR1 |= TIM_CR1_CEN; }
    return start_result;
}

static void TestDebugMailbox(void)
{
    static const struct {
        int32_t requested;
        int16_t command;
        int16_t applied;
        uint16_t pulse;
        uint8_t limited;
    } cases[] = {
        {0, 0, 0, 1500, 0},
        {1000, 1000, 1000, 1611, 0},
        {-1000, -1000, -1000, 1389, 0},
        {4500, 4500, 4500, 2000, 0},
        {-4500, -4500, -4500, 1000, 0},
        {4501, 4501, 4500, 2000, 1},
        {-4501, -4501, -4500, 1000, 1},
        {INT32_MAX, INT16_MAX, 4500, 2000, 1},
        {INT32_MIN, INT16_MIN, -4500, 1000, 1},
    };
    RudderState state;
    int16_t command = 123;
    unsigned i;
    uint32_t before_count;
    uint32_t before_pulse;

    /* 默认关闭；只改邮箱不写输出，必须由最终仲裁显式采用。 */
    assert(g_rudder_debug.enabled == 0U && g_rudder_debug.target_angle_cdeg == 0);
    assert(!RudderDriver_GetDebugCommand(1U, &command) && command == 123);
    assert(!RudderDriver_GetDebugCommand(1U, NULL));
    for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        before_count = g_rudder_state.update_count;
        before_pulse = test_tim4.CCR4;
        g_rudder_debug.target_angle_cdeg = cases[i].requested;
        g_rudder_debug.enabled = 1U;
        assert(RudderDriver_GetDebugCommand(1U, &command));
        assert(command == cases[i].command);
        assert(test_tim4.CCR4 == before_pulse && g_rudder_state.update_count == before_count);
        RudderDriver_ApplyDebug(command);
        assert(RudderDriver_CopyState(&state));
        assert(state.source == RUDDER_SOURCE_DEBUG && state.pwm_started == 1U);
        assert(state.requested_angle_cdeg == cases[i].command);
        assert(state.applied_angle_cdeg == cases[i].applied);
        assert(state.applied_pulse_us == cases[i].pulse && test_tim4.CCR4 == cases[i].pulse);
        assert(state.limit_active == cases[i].limited);
    }

    /* 非 1 的使能不接受；禁用不改目标，下一次是否回中由仲裁 Apply 决定。 */
    g_rudder_debug.enabled = 2U;
    assert(!RudderDriver_GetDebugCommand(1U, &command));
    g_rudder_debug.enabled = UINT32_MAX;
    assert(!RudderDriver_GetDebugCommand(1U, &command));
    g_rudder_debug.enabled = 1U;
    assert(!RudderDriver_GetDebugCommand(1U, NULL));
    before_pulse = test_tim4.CCR4;
    g_rudder_debug.enabled = 0U;
    assert(!RudderDriver_GetDebugCommand(1U, &command));
    assert(test_tim4.CCR4 == before_pulse);
    RudderDriver_Apply(0, 0U);
    assert(test_tim4.CCR4 == 1500U && g_rudder_state.source == RUDDER_SOURCE_CENTER);

    /* 遇到其他控制源时撤销使能；冲突解除后不会自动恢复旧调试命令。 */
    g_rudder_debug.target_angle_cdeg = 4500;
    g_rudder_debug.enabled = 1U;
    assert(!RudderDriver_GetDebugCommand(0U, &command));
    assert(g_rudder_debug.enabled == 0U);
    assert(!RudderDriver_GetDebugCommand(1U, &command));
    assert(test_tim4.CCR4 == 1500U);
    g_rudder_debug.enabled = 1U;
    assert(!RudderDriver_GetDebugCommand(0U, NULL));
    assert(g_rudder_debug.enabled == 0U);
    RudderDriver_Apply(4500, 1U);
    assert(test_tim4.CCR4 == 2000U && g_rudder_state.source == RUDDER_SOURCE_ETHERNET);
    assert(critical_depth == 0U && starts == 1U);
}

int main(int argc, char **argv)
{
    TIM_HandleTypeDef tim = { TIM4 };
    TIM_TypeDef wrong_instance = {0};
    TIM_HandleTypeDef wrong_tim = { &wrong_instance };
    RudderState state;
    uint32_t expected_clock = 100000000U;
    int32_t angle;
    uint16_t previous = 0U;

    assert(argc >= 2);
    test_rcc.D2CFGR = 5U << RCC_D2CFGR_D2PPRE1_Pos;
    test_tim4.PSC = 99U;
    test_tim4.ARR = 3002U;
    test_tim4.CNT = 99U;
    RudderDriver_Apply(1000, 1U);
    RudderDriver_ApplyDebug(4500);
    assert(test_tim4.CCR4 == 0U && starts == 0U);
    assert(RudderDriver_Init(NULL) == HAL_ERROR);
    assert(RudderDriver_Init(&wrong_tim) == HAL_ERROR);
    assert(!RudderDriver_CopyState(NULL));

    if (strcmp(argv[1], "valid") == 0)
    {
        assert(argc == 7);
        test_rcc.D2CFGR = (uint32_t)strtoul(argv[2], NULL, 10) << RCC_D2CFGR_D2PPRE1_Pos;
        test_rcc.CFGR = (strtoul(argv[3], NULL, 10) != 0U) ? RCC_CFGR_TIMPRE : 0U;
        pclk = (uint32_t)strtoul(argv[4], NULL, 10);
        test_tim4.PSC = (uint32_t)strtoul(argv[5], NULL, 10);
        expected_clock = (uint32_t)strtoul(argv[6], NULL, 10);
    }
    else
    {
        if (strcmp(argv[1], "zero_hclk") == 0) { hclk = 0U; }
        else if (strcmp(argv[1], "zero_pclk") == 0) { pclk = 0U; }
        else if (strcmp(argv[1], "wrong_psc") == 0) { test_tim4.PSC = 119U; }
        else if (strcmp(argv[1], "wrong_arr") == 0) { test_tim4.ARR = 19999U; }
        else if (strcmp(argv[1], "fractional_tick") == 0) { pclk = 50000001U; }
        else if (strcmp(argv[1], "down_count") == 0) { test_tim4.CR1 = TIM_CR1_DIR; }
        else if (strcmp(argv[1], "center_count") == 0) { test_tim4.CR1 = TIM_CR1_CMS; }
        else if (strcmp(argv[1], "running") == 0) { test_tim4.CR1 = TIM_CR1_CEN; }
        else if (strcmp(argv[1], "hal_error") == 0) { start_result = HAL_ERROR; }
        else { assert(0); }
        assert(RudderDriver_Init(&tim) == HAL_ERROR);
        assert(critical_entries == 0U);
        assert(RudderDriver_CopyState(&state));
        assert(state.pwm_started == 0U && state.source == RUDDER_SOURCE_NOT_STARTED);
        assert(starts == ((start_result == HAL_ERROR) ? 1U : 0U));
        previous = (uint16_t)test_tim4.CCR4;
        RudderDriver_Apply(1000, 1U);
        assert(test_tim4.CCR4 == previous);
        return 0;
    }

    assert(RudderDriver_Init(&tim) == HAL_OK);
    assert(critical_entries == 0U);
    assert(RudderDriver_CopyState(&state));
    assert(state.pwm_started == 1U && state.source == RUDDER_SOURCE_CENTER);
    assert(state.timer_clock_hz == expected_clock);
    assert(state.applied_pulse_us == 1500U);
    assert(RudderDriver_Init(&tim) == HAL_BUSY && starts == 1U);

    /* 遍历完整 int16 输入域，确认极值不溢出、脉宽单调且绝不越界。 */
    for (angle = INT16_MIN; angle <= INT16_MAX; ++angle)
    {
        RudderDriver_Apply((int16_t)angle, 1U);
        assert(RudderDriver_CopyState(&state));
        assert(state.source == RUDDER_SOURCE_ETHERNET);
        assert(state.requested_angle_cdeg == angle);
        assert(state.applied_angle_cdeg >= -4500 && state.applied_angle_cdeg <= 4500);
        assert(state.applied_pulse_us >= 1000U && state.applied_pulse_us <= 2000U);
        assert(state.applied_pulse_us >= previous);
        assert(state.applied_pulse_us == test_tim4.CCR4);
        assert(state.limit_active == (angle < -4500 || angle > 4500));
        previous = state.applied_pulse_us;
    }

    RudderDriver_Apply(450, 1U);
    assert(RudderDriver_CopyState(&state));
    assert(state.applied_angle_cdeg == 450 && state.applied_pulse_us == 1550U);
    for (angle = 0; angle < 100; ++angle) { RudderDriver_Apply(450, 1U); }
    assert(test_tim4.CCR4 == 1550U); /* 重复绝对目标不会继续转动。 */
    RudderDriver_Apply(-450, 1U);
    assert(test_tim4.CCR4 == 1450U);
    RudderDriver_Apply(INT16_MAX, 0U);
    assert(RudderDriver_CopyState(&state));
    assert(state.source == RUDDER_SOURCE_CENTER);
    assert(state.requested_angle_cdeg == 0 && state.applied_angle_cdeg == 0);
    assert(state.applied_pulse_us == 1500U && state.limit_active == 0U);
    assert(state.update_count == 65639U);
    assert(critical_depth == 0U && starts == 1U);
    TestDebugMailbox();
    return 0;
}
"""


class RudderDriverHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("未找到用于运行假 HAL 测试的主机 gcc")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="rudder_host_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "stm32h7xx_hal.h").write_text(HAL_STUB, encoding="utf-8")
        (work / "FreeRTOS.h").write_text("/* 主机测试不启动调度器。 */\n", encoding="utf-8")
        (work / "task.h").write_text(TASK_STUB, encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("rudder_host.exe" if os.name == "nt" else "rudder_host")
        subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(work), "-I", str(ROOT / "software/Glider-Core"),
             str(ROOT / "software/Glider-Core/rudder_driver.c"), str(work / "harness.c"),
             "-o", str(cls.binary)],
            check=True, capture_output=True, text=True, env=cls.env,
        )

    def run_case(self, *args):
        result = subprocess.run(
            [str(self.binary), *map(str, args)], capture_output=True, text=True, env=self.env,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_clock_tree_and_absolute_position_outputs(self):
        # 覆盖全部 APB1 分频与 TIMPRE 组合；每次由全新进程模拟冷启动。
        clocks = [
            (0, 0, 200000000, 199, 200000000),
            (4, 0, 100000000, 199, 200000000),
            (5, 0, 50000000, 99, 100000000),
            (6, 0, 25000000, 49, 50000000),
            (7, 0, 12500000, 24, 25000000),
            (0, 1, 200000000, 199, 200000000),
            (4, 1, 100000000, 199, 200000000),
            (5, 1, 50000000, 199, 200000000),
            (6, 1, 25000000, 99, 100000000),
            (7, 1, 12500000, 49, 50000000),
        ]
        for case in clocks:
            with self.subTest(case=case):
                self.run_case("valid", *case)

    def test_bad_configuration_or_hal_failure_does_not_activate(self):
        for case in (
            "zero_hclk", "zero_pclk", "wrong_psc", "wrong_arr", "fractional_tick",
            "down_count", "center_count", "running", "hal_error",
        ):
            with self.subTest(case=case):
                self.run_case(case)


if __name__ == "__main__":
    unittest.main()
