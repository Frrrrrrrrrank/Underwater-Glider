"""抽取真实 ServoTask 仲裁，连接真实校准/步进驱动与 PWM2 模型；不连接硬件。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

import test_stepper_limits as stepper_fixture
from test_rudder_protocol import extract_function


ROOT = Path(__file__).resolve().parents[2]

C_SETUP = r"""
#include "worm_axis.h"
#include "motor_driver.h"
#include "ethernet_uart_driver.h"
#include "pid_control_service.h"
#include "rudder_driver.h"
#include "thruster_driver.h"
#include "displacement_sensor.h"
#include "pump_driver.h"
#include "task.h"
#include <setjmp.h>

static void servo_tick(void);
static uint8_t sbus_available;
static uint16_t sbus_channels[SBUS_CHANNEL_COUNT];
static uint32_t sbus_last_tick;
"""

C_STUBS = r"""
volatile UMotor g_umotor;
volatile EthernetUartState g_ethernet_uart_state;
volatile DisplacementSensorData g_displacement_sensor;
static EthernetUartReadoutPack direct_pack;
static uint8_t direct_active = 1U;
static PidControlStatus pid_status_stub;
static ZigzagControlOutput pid_output_stub;
static int32_t pitch_applied;
static uint8_t pitch_enabled;
static unsigned servo_iteration;
static const char *scenario;
static jmp_buf servo_exit;
static unsigned center_start_count;

void UMotor_SetTargetSpeedById(uint8_t id, int32_t rpm)
{
    assert(id == 1U);
    pitch_applied = rpm;
}
void UMotor_SetMotorEnable(uint8_t slot, uint8_t enabled)
{
    assert(slot == 0U);
    pitch_enabled = enabled;
}
bool EthernetUartDriver_CopyControlPack(EthernetUartReadoutPack *out, uint8_t *active)
{
    *out = direct_pack;
    *active = direct_active;
    return true;
}
uint32_t EkfBridge_CopyState(StateEstimate *out) { memset(out, 0, sizeof(*out)); return 1U; }
void PidControlService_Step(const StateEstimate *input, uint8_t valid,
    float displacement, uint32_t now, float dt, ZigzagControlOutput *out)
{
    (void)input; (void)valid; (void)displacement; (void)now; (void)dt;
    *out = pid_output_stub;
}
uint32_t PidControlService_CopyStatus(PidControlStatus *out)
{
    *out = pid_status_stub;
    return 1U;
}
uint32_t SbusDriver_GetChannels(uint16_t *out, uint32_t count, uint32_t *lost,
    uint32_t *failsafe, uint32_t *last)
{
    assert(count >= SBUS_CHANNEL_COUNT);
    memcpy(out, sbus_channels, sizeof(sbus_channels));
    *lost = *failsafe = 0U;
    *last = sbus_last_tick;
    return sbus_available ? SBUS_CHANNEL_COUNT : 0U;
}
void RudderDriver_Apply(int16_t angle, uint8_t active) { (void)angle; (void)active; }
bool RudderDriver_CopyState(RudderState *out) { memset(out, 0, sizeof(*out)); return true; }
HAL_StatusTypeDef ThrusterDriver_SetDualPulseUs(uint16_t a, uint16_t b)
{
    (void)a; (void)b; return HAL_OK;
}
uint16_t ThrusterDriver_MapDirectionSpeedToPulseUs(uint8_t dir, uint8_t speed)
{
    (void)dir; (void)speed; return 1500U;
}
uint8_t IoControlPump_SubmitCommand(IoControlPumpSource source, uint8_t enable,
    uint8_t direction, uint32_t now)
{
    (void)source; (void)enable; (void)direction; (void)now; return 1U;
}

static WormAxisState axis_state(void)
{
    WormAxisState out;
    WormAxis_CopyState(&out);
    return out;
}
static void assert_stopped(void)
{
    assert(state().running == 0U && pitch_applied == 0);
    assert(axis_state().calib_hold != 0U);
}
static void set_sbus_neutral(void)
{
    sbus_channels[SBUS_CH_SWITCH_LEFT_OUTER] = 240U;
    sbus_channels[SBUS_CH_SWITCH_LEFT_MID_1] = 240U;
    sbus_channels[SBUS_CH_SWITCH_LEFT_MID_2] = 240U;
    sbus_channels[SBUS_CH_SWITCH_RIGHT_OUTER] = 240U;
}
static void pulses(unsigned count) { while (count-- != 0U) irq(); }

static void full_calibration_tick(void)
{
    switch (servo_iteration)
    {
    case 1:
        assert_stopped();
        break;
    case 2:
        assert(pitch_applied == WORM_HOME_SEEK_RPM && state().pulse_rpm < 0);
        pulses(5U);
        set_limits(ROLL_LIMIT_NEG_ACTIVE);
        g_limit_switch_states[1] = 0U;
        break;
    case 3:
        assert(pitch_applied == -WORM_HOME_BACKOFF_RPM);
        assert(state().release_armed_mask == ROLL_LIMIT_NEG_ACTIVE);
        assert(starts == 2U);
        pulses(2U);
        set_limits(0U);
        irq();
        assert(state().last_stop_reason == STEPPER_STOP_RELEASE);
        assert(state().release_pos == -2);
        g_limit_switch_states[1] = 1U;
        g_umotor.nodes[0].debug.pos_signed_32 = -2;
        break;
    case 4:
        assert_stopped();
        assert(axis_state().home_state == WORM_HOME_DONE);
        assert(axis_state().roll.home_state == WORM_ROLL_HOME_BACKOFF_NEG);
        assert(state().blocked_mask == ROLL_LIMIT_NEG_ACTIVE);
        assert(starts == 2U);
        break;
    case 5:
        assert(axis_state().roll.pulse_min == -2 && state().pulse_rpm > 0);
        assert(starts == 3U);
        pulses(100U);
        set_limits(ROLL_LIMIT_POS_ACTIVE);
        irq();
        assert(state().last_stop_reason == STEPPER_STOP_LIMIT);
        break;
    case 6:
        assert(state().release_armed_mask == ROLL_LIMIT_POS_ACTIVE);
        assert(state().pulse_rpm < 0 && starts == 4U);
        pulses(2U);
        set_limits(0U);
        irq();
        assert(state().release_seq == 2U && state().release_pos == 96);
        break;
    case 7:
        assert_stopped();
        assert(starts == 4U);
        if (strcmp(scenario, "center_failure") == 0) start_result = HAL_ERROR;
        break;
    case 8:
        if (strcmp(scenario, "center_failure") == 0)
        {
            assert_stopped();
            assert(axis_state().calib_state == WORM_CALIB_FAILED);
            assert(axis_state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
            assert(starts == 5U);
            longjmp(servo_exit, 1);
        }
        assert(axis_state().roll.home_state == WORM_ROLL_HOME_CENTER);
        assert(state().target_pos == 47 && state().pulses_remaining == 49U);
        center_start_count = starts;
        pulses(10U);
        break;
    case 9:
        assert(starts == center_start_count && state().pulses_remaining == 39U);
        assert(state().target_pos == 47 && state().running != 0U);
        pulses(state().pulses_remaining);
        assert(state().current_pos == 47 && electrical_position == 47);
        break;
    case 10:
        assert_stopped();
        assert(axis_state().calib_state == WORM_CALIB_DONE);
        assert(axis_state().roll.valid != 0U && axis_state().roll.stroke_percent == 50.0f);
        assert(starts == center_start_count);
        break;
    case 11:
        assert_stopped(); /* 旧 DIRECT 缓存仍非零，不得自动续动。 */
        ++g_ethernet_uart_diag.control_packet_count;
        break;
    case 12:
        assert_stopped(); /* 新非零命令也不能直接解除终态保持。 */
        direct_pack.pitchmotor_speed_rpm = direct_pack.rollmotor_speed_rpm = 0;
        ++g_ethernet_uart_diag.control_packet_count;
        break;
    case 13:
        assert(state().running == 0U && pitch_applied == 0);
        assert(axis_state().calib_hold == 0U);
        direct_pack.pitchmotor_speed_rpm = direct_pack.rollmotor_speed_rpm = 60;
        ++g_ethernet_uart_diag.control_packet_count;
        break;
    case 14:
        assert(pitch_applied == -60 && state().pulse_rpm > 0);
        assert(starts == center_start_count + 1U);
        longjmp(servo_exit, 1);
    default:
        assert(0);
    }
}

static void failure_retry_tick(void)
{
    switch (servo_iteration)
    {
    case 1:
        assert_stopped();
        start_result = HAL_ERROR;
        break;
    case 2:
        assert_stopped(); /* CONTINUOUS 的启动失败也必须当拍撤销 pitch。 */
        assert(axis_state().calib_state == WORM_CALIB_FAILED);
        assert(axis_state().roll.home_fail == WORM_ROLL_FAIL_DRIVER);
        assert(pitch_enabled == 0U && starts == 1U);
        start_result = HAL_OK;
        WormAxis_RequestCalibration();
        break;
    case 3:
        assert_stopped();
        assert(axis_state().calib_state == WORM_CALIB_RUNNING);
        assert(state().last_stop_reason == STEPPER_STOP_START_FAILED);
        break;
    case 4:
        assert(axis_state().calib_state == WORM_CALIB_RUNNING);
        assert(pitch_applied == WORM_HOME_SEEK_RPM && state().running != 0U);
        assert(starts == 2U);
        longjmp(servo_exit, 1);
    default: assert(0);
    }
}

static void backoff_failure_tick(void)
{
    if (servo_iteration == 1U) { assert_stopped(); return; }
    if (servo_iteration == 2U)
    {
        assert(state().running != 0U && pitch_applied != 0);
        set_limits(ROLL_LIMIT_NEG_ACTIVE);
        start_result = HAL_ERROR;
        return;
    }
    assert(servo_iteration == 3U);
    assert_stopped();
    assert(axis_state().calib_state == WORM_CALIB_FAILED);
    assert(axis_state().home_fail == WORM_HOME_FAIL_OTHER_AXIS);
    assert(pitch_enabled == 0U);
    longjmp(servo_exit, 1);
}

static void cancel_or_pid_tick(void)
{
    unsigned is_pid = strcmp(scenario, "pid") == 0;
    unsigned is_sbus = strcmp(scenario, "sbus_hold") == 0;
    if (servo_iteration == 1U) { assert_stopped(); return; }
    if (servo_iteration == 2U)
    {
        assert(state().running != 0U && pitch_applied != 0);
        pulses(4U);
        if (is_pid)
        {
            pid_status_stub.arm_requested = 1U;
            pid_status_stub.requested_mode = ZIGZAG_MODE_PITCH_HOLD;
            pid_output_stub.output_valid = 1U;
            pid_output_stub.worm_motor_rpm = 77;
        }
        else { WormAxis_CancelCalibration(); }
        return;
    }
    if (servo_iteration == 3U)
    {
        assert(state().running == 0U);
        assert(axis_state().calib_state == WORM_CALIB_CANCELLED);
        if (is_pid)
        {
            assert(axis_state().calib_fail == WORM_CALIB_FAIL_PID);
            assert(pitch_applied == -77 && g_rc_ctrl_mode == RC_CTRL_MODE_PID_ETH);
            pid_status_stub.arm_requested = 0U;
        }
        else { assert_stopped(); }
        if (is_sbus) set_sbus_neutral(); /* 相同帧时间戳仍然不能解锁。 */
        return;
    }
    if (servo_iteration == 4U)
    {
        assert_stopped();
        if (!is_sbus) longjmp(servo_exit, 1);
        ++sbus_last_tick;
        return;
    }
    assert(servo_iteration == 5U && is_sbus);
    assert(axis_state().calib_hold == 0U && pitch_applied == 0 && state().running == 0U);
    longjmp(servo_exit, 1);
}

static void servo_tick(void)
{
    assert(critical_depth == 0U);
    ++servo_iteration;
    if (strcmp(scenario, "full") == 0 || strcmp(scenario, "center_failure") == 0)
        full_calibration_tick();
    else if (strcmp(scenario, "retry") == 0) failure_retry_tick();
    else if (strcmp(scenario, "backoff_failure") == 0) backoff_failure_tick();
    else cancel_or_pid_tick();
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    (void)service; /* 与限位单测共用的夹具辅助函数，本测试由 ServoTask 自己服务限位。 */
    reset(0U);
    g_umotor.nodes[0].valid = 1U;
    g_umotor.nodes[0].debug.pos_signed_32 = -100000;
    direct_pack.pitchmotor_enable = direct_pack.rollmotor_enable = 1U;
    direct_pack.pitchmotor_speed_rpm = direct_pack.rollmotor_speed_rpm = 90;
    if (strcmp(scenario, "sbus_hold") == 0)
    {
        direct_active = 0U;
        sbus_available = 1U;
        set_sbus_neutral();
        sbus_channels[SBUS_CH_SWITCH_LEFT_MID_1] = 1024U;
        sbus_channels[SBUS_CH_LEFT_STICK_X] = 1807U;
        sbus_channels[SBUS_CH_LEFT_STICK_Y] = 1807U;
        sbus_last_tick = 1U;
    }
    WormAxis_RequestCalibration();
    if (setjmp(servo_exit) == 0) StartServoTask(NULL);
    assert(critical_depth == 0U);
    return 0;
}
"""


class ServoCalibrationIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("未找到主机 GCC")
        cls.source = (ROOT / "software/Core/Src/main.c").read_text(encoding="utf-8-sig")
        task_source = extract_function(cls.source, "StartServoTask")
        # 直接包含 main 的仲裁宏、枚举和全部 helper，不在夹具复制控制逻辑。
        helpers = cls.source.split("#define RC_SWITCH_UP_THRESHOLD", 1)[1].split("extern void DemoFatFS", 1)[0]
        helpers = "#define RC_SWITCH_UP_THRESHOLD" + helpers
        globals_source = cls.source.split("volatile uint32_t g_dbg_servo_start_ok", 1)[1].split(
            "volatile uint32_t g_limit_switch_update_count", 1)[0]
        globals_source = "volatile uint32_t g_dbg_servo_start_ok" + globals_source
        fixture = stepper_fixture.HARNESS.split("static void test_gpio(void)", 1)[0]
        fixture = fixture.replace("void osDelay(uint32_t ms) { tick += ms; }",
                                  "void osDelay(uint32_t ms) { tick += ms; servo_tick(); }")
        fixture = fixture.replace("(void)timeout;\n    return 0U;", "(void)timeout;\n    return sbus_available;")
        harness = "\n".join((
            '#include "sbus_driver.h"', C_SETUP, fixture, globals_source,
            C_STUBS.split("static void full_calibration_tick(void)", 1)[0],
            helpers, task_source,
            "static void full_calibration_tick(void)" + C_STUBS.split("static void full_calibration_tick(void)", 1)[1],
        ))
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="servo_calib_host_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        for name, contents in {
            "stm32h7xx_hal.h": stepper_fixture.HAL_STUB,
            "main.h": '#include "stm32h7xx_hal.h"\n',
            "FreeRTOS.h": '#include <stddef.h>\n',
            "task.h": stepper_fixture.TASK_STUB,
            "cmsis_os.h": stepper_fixture.CMSIS_STUB,
            "harness.c": harness,
        }.items():
            (work / name).write_text(contents, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("servo_calib_host.exe" if os.name == "nt" else "servo_calib_host")
        core = ROOT / "software/Glider-Core"
        estimation = ROOT / "software/Glider-App/estimation"
        result = subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(work), "-I", str(core), "-I", str(estimation),
            "-I", str(estimation / "control"), "-I", str(estimation / "ug_estimation/include"),
            str(core / "stepper_driver.c"), str(core / "limit_switch.c"),
            str(core / "worm_axis.c"), str(work / "harness.c"), "-o", str(cls.binary),
        ], capture_output=True, text=True, env=cls.env)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True, env=self.env)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_complete_two_axis_calibration_keeps_center_and_requires_new_neutral(self):
        self.run_case("full")

    def test_seek_start_failure_stops_pitch_same_cycle_and_new_calibration_retries(self):
        self.run_case("retry")

    def test_backoff_start_failure_stops_pitch_same_cycle(self):
        self.run_case("backoff_failure")

    def test_center_start_failure_cannot_report_success(self):
        self.run_case("center_failure")

    def test_cancel_stops_both_and_holds_old_direct_command(self):
        self.run_case("cancel")

    def test_pid_takes_over_same_cycle_and_leaves_roll_stopped(self):
        self.run_case("pid")

    def test_sbus_requires_new_all_switches_up_frame_to_release_hold(self):
        self.run_case("sbus_hold")

    def test_tim1_configuration_matches_pwm2_counting_contract(self):
        init = extract_function(self.source, "MX_TIM1_Init")
        self.assertIn("htim1.Init.Prescaler = 100-1", init)
        self.assertIn("htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE", init)
        self.assertIn("sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE", init)
        ch2, ch4 = init.split("HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)", 1)
        self.assertIn("sConfigOC.OCMode = TIM_OCMODE_PWM2", ch2)
        self.assertIn("sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH", ch2)
        self.assertIn("sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET", ch2)
        self.assertIn("sConfigOC.OCMode = TIM_OCMODE_PWM1", ch4)
        ioc = (ROOT / "software/software.ioc").read_text(encoding="utf-8-sig")
        for expected in (
            "TIM1.Prescaler=100-1",
            "TIM1.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE",
            "TIM1.OffStateIDLEMode=TIM_OSSI_ENABLE",
            r"TIM1.OCMode_PWM-PWM\ Generation2\ CH2=TIM_OCMODE_PWM2",
        ):
            self.assertIn(expected, ioc)


if __name__ == "__main__":
    unittest.main()

