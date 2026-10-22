"""提取真实板端尾舵协议函数，用主机 GCC 检查解析、旧包兼容和租约边界。"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

SOFTWARE_DIR = Path(__file__).resolve().parents[1]
SOURCE_PATH = SOFTWARE_DIR / "Glider-Core" / "ethernet_uart_driver.c"
HEADER_PATH = SOURCE_PATH.with_suffix(".h")


def extract_function(source, name):
    """忽略注释/字符串中的花括号，保留实际函数原文，避免另写解析逻辑。"""
    masked = re.sub(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda match: " " * len(match.group()), source, flags=re.S,
    )
    match = re.search(
        r"^(?:static\s+)?(?:bool|uint32_t|int32_t|int16_t|void)\s+"
        + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", masked, re.M,
    )
    if not match:
        raise AssertionError(f"找不到函数定义：{name}")
    start = masked.index("{", match.start())
    depth = 1
    for end in range(start + 1, len(masked)):
        depth += (masked[end] == "{") - (masked[end] == "}")
        if depth == 0:
            return source[match.start():end + 1]
    raise AssertionError(f"函数花括号不完整：{name}")


def extract_struct(source, name):
    match = re.search(
        r"typedef\s+struct\s*\{[^{}]*\}\s*" + re.escape(name) + r"\s*;", source, re.S,
    )
    if not match:
        raise AssertionError(f"找不到结构体：{name}")
    return match.group()


def extract_define(source, name):
    match = re.search(r"^#define\s+" + re.escape(name) + r"\s+[^\n]+", source, re.M)
    if not match:
        raise AssertionError(f"找不到宏：{name}")
    return match.group()


class RudderFirmwareProtocolTests(unittest.TestCase):
    def test_real_parser_control_mailbox_and_expiry(self):
        source = SOURCE_PATH.read_text(encoding="utf-8-sig")
        header = HEADER_PATH.read_text(encoding="utf-8-sig")
        compiler = shutil.which("gcc")
        if compiler is None:
            self.fail("本测试需要主机 GCC，不能使用 arm-none-eabi-gcc")
        defines = "\n".join(extract_define(source + "\n" + header, name) for name in (
            "UGCP_CONTROL_PAYLOAD_SIZE", "UGCP_CONTROL21_PAYLOAD_SIZE",
            "UGCP_RUDDER_MAX_ANGLE_CDEG", "ETHERNET_UART_CONTROL_POLICY_NONE",
            "ETHERNET_UART_CONTROL_POLICY_STREAM", "ETHERNET_UART_CONTROL_POLICY_LATCHED",
            "ETHERNET_UART_CONTROL_TIMEOUT_MS",
        ))
        structs = "\n".join(extract_struct(header, name) for name in (
            "EthernetUartReadoutPack", "EthernetUartDiag",
        ))
        helpers = []
        for name in ("EthernetUartDriver_ReadLeU32", "EthernetUartDriver_ReadLeI32", "EthernetUartDriver_ReadLeI16"):
            if re.search(r"\b" + name + r"\s*\(", source):
                helpers.append(extract_function(source, name))
        functions = "\n\n".join(helpers + [extract_function(source, name) for name in (
            "EthernetUartDriver_ParseControlPayload", "EthernetUartDriver_HandleControl",
            "EthernetUartDriver_CopyControlPack",
        )])
        harness = "\n\n".join((C_INCLUDES, defines, structs, C_STUBS, functions, C_TESTS))
        # 临时文件只位于测试目录，退出上下文后自动清理。
        with tempfile.TemporaryDirectory(prefix="rudder_protocol_", dir=Path(__file__).parent) as tmp:
            c_path = Path(tmp) / "protocol_test.c"
            exe_path = Path(tmp) / ("protocol_test.exe" if os.name == "nt" else "protocol_test")
            c_path.write_text(harness, encoding="utf-8")
            # Windows 常同时安装旧工具链，优先使用所选 GCC 自带 DLL。
            test_env = {**os.environ, "TMP": tmp, "TEMP": tmp, "TMPDIR": tmp}
            test_env["PATH"] = str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", "")
            compiled = subprocess.run(
                [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", str(c_path), "-o", str(exe_path)],
                capture_output=True, text=True, encoding="utf-8", errors="replace", check=False,
                env=test_env,
            )
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run(
                [str(exe_path)], capture_output=True, text=True, encoding="utf-8", errors="replace", check=False, env=test_env,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("rudder protocol tests passed", result.stdout)


C_INCLUDES = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
"""

C_STUBS = r"""
/* 仅替换硬件/RTOS边界；解析、发布和过期逻辑来自真实固件。 */
static uint32_t fake_tick;
static unsigned critical_depth;
static unsigned pid_off_count;
static uint32_t pid_off_tick;
static volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
static volatile uint8_t g_ethernet_rx_message_flag;
static volatile uint32_t g_ethernet_control_last_rx_tick;
static volatile uint8_t g_ethernet_control_policy;
static EthernetUartDiag g_ethernet_uart_diag;
static uint8_t s_snapshot_pending;
typedef struct { uint8_t mode; } PidControlCommand;
#define ZIGZAG_MODE_OFF 0U
#define taskENTER_CRITICAL() do { ++critical_depth; } while (0)
#define taskEXIT_CRITICAL() do { assert(critical_depth > 0U); --critical_depth; } while (0)
static uint32_t HAL_GetTick(void) { return fake_tick; }
static uint8_t PidControlService_SubmitCommand(const PidControlCommand *command, uint32_t now)
{
    assert(command->mode == ZIGZAG_MODE_OFF);
    ++pid_off_count;
    pid_off_tick = now;
    return 1U;
}
"""

C_TESTS = r"""
static void set_angle(uint8_t *raw, int16_t angle)
{
    uint16_t bits = (uint16_t)angle;
    raw[19] = (uint8_t)bits;
    raw[20] = (uint8_t)(bits >> 8U);
}

static void reset_state(void)
{
    memset((void *)&g_ethernet_uart_readout_pack, 0, sizeof(g_ethernet_uart_readout_pack));
    memset(&g_ethernet_uart_diag, 0, sizeof(g_ethernet_uart_diag));
    g_ethernet_rx_message_flag = 0U;
    g_ethernet_control_last_rx_tick = 0U;
    g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_NONE;
    critical_depth = 0U;
    pid_off_count = 0U;
    pid_off_tick = 0U;
    s_snapshot_pending = 0U;
}

static void test_parse_boundaries(void)
{
    uint8_t raw[22] = {1U, 0x38U, 0xFFU, 0xFFU, 0xFFU, 1U, 0xC8U, 0U, 0U, 0U,
                       0U, 1U, 33U, 44U, 0U, 1U, 1U, 1U, 1U, 0U, 0U, 0U};
    EthernetUartReadoutPack parsed;
    const int16_t angles[] = {-4500, -1000, -1, 0, 1, 1000, 4500};
    unsigned i;
    for (i = 0U; i < sizeof(angles) / sizeof(angles[0]); ++i)
    {
        set_angle(raw, angles[i]);
        assert(EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
        assert(parsed.rudder_valid == 1U);
        assert(parsed.rudder_angle_cdeg == angles[i]);
        assert(parsed.pitchmotor_enable == 1U && parsed.pitchmotor_speed_rpm == -200);
        assert(parsed.rollmotor_enable == 1U && parsed.rollmotor_speed_rpm == 200);
        assert(parsed.thrustmotor_direction == 0U && parsed.thrustmotor_direction2 == 1U);
        assert(parsed.thrustmotor1se_speed_rpm == 33U && parsed.thrustmotor2se_speed_rpm == 44U);
        assert(parsed.pump_direction == 1U && parsed.pump_on_off == 1U);
        assert(parsed.ask_for_status == 1U && parsed.eth_control_flag == 1U);
    }
    set_angle(raw, 4501);
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
    set_angle(raw, -4501);
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
    set_angle(raw, INT16_MIN);
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
    set_angle(raw, 0);
    for (i = 0U; i <= 22U; ++i)
    {
        assert(EthernetUartDriver_ParseControlPayload(raw, (uint16_t)i, 1U, &parsed) == (i == 21U));
        assert(EthernetUartDriver_ParseControlPayload(raw, (uint16_t)i, 0U, &parsed) == (i == 19U));
    }
    assert(!EthernetUartDriver_ParseControlPayload(NULL, 21U, 1U, &parsed));
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, NULL));
    memset(&parsed, 0xA5, sizeof(parsed));
    assert(EthernetUartDriver_ParseControlPayload(raw, 19U, 0U, &parsed));
    assert(parsed.rudder_valid == 0U && parsed.rudder_angle_cdeg == 0);
    raw[18] = 0U;
    set_angle(raw, 500);
    assert(EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
    assert(parsed.rudder_valid == 0U);
    raw[18] = 1U;
    raw[12] = 101U;
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
    raw[12] = 33U;
    raw[18] = 2U;
    assert(!EthernetUartDriver_ParseControlPayload(raw, 21U, 1U, &parsed));
}

static void test_publish_reject_and_legacy(void)
{
    uint8_t raw[21] = {0U};
    EthernetUartReadoutPack copied;
    uint8_t flag = 0U;
    uint32_t accepted_tick;
    reset_state();
    raw[18] = 1U;
    raw[17] = 1U;
    set_angle(raw, -1000);
    fake_tick = 1234U;
    EthernetUartDriver_HandleControl(raw, 21U, ETHERNET_UART_CONTROL_POLICY_STREAM, 1U);
    assert(g_ethernet_uart_diag.control_packet_count == 1U);
    assert(g_ethernet_control_policy == ETHERNET_UART_CONTROL_POLICY_STREAM);
    assert(pid_off_count == 1U && pid_off_tick == fake_tick);
    assert(s_snapshot_pending == 1U);
    assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
    assert(flag == 1U && copied.rudder_valid == 1U && copied.rudder_angle_cdeg == -1000);
    accepted_tick = g_ethernet_control_last_rx_tick;
    fake_tick += 100U;
    set_angle(raw, 4501);
    EthernetUartDriver_HandleControl(raw, 21U, ETHERNET_UART_CONTROL_POLICY_STREAM, 1U);
    assert(g_ethernet_uart_diag.control_reject_count == 1U);
    assert(g_ethernet_uart_diag.control_packet_count == 1U);
    assert(g_ethernet_control_last_rx_tick == accepted_tick);
    assert(g_ethernet_uart_readout_pack.rudder_angle_cdeg == -1000);
    assert(pid_off_count == 1U);
    EthernetUartDriver_HandleControl(raw, 19U, ETHERNET_UART_CONTROL_POLICY_STREAM, 1U);
    assert(g_ethernet_uart_diag.control_reject_count == 2U);
    assert(g_ethernet_uart_diag.frame_length_error_count == 1U);
    assert(g_ethernet_control_last_rx_tick == accepted_tick);
    EthernetUartDriver_HandleControl(raw, 19U, ETHERNET_UART_CONTROL_POLICY_STREAM, 0U);
    assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
    assert(flag == 1U && copied.rudder_valid == 0U && copied.rudder_angle_cdeg == 0);
    EthernetUartDriver_HandleControl(raw, 19U, ETHERNET_UART_CONTROL_POLICY_LATCHED, 0U);
    assert(g_ethernet_control_policy == ETHERNET_UART_CONTROL_POLICY_LATCHED);
    assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
    assert(flag == 1U && copied.rudder_valid == 0U && copied.rudder_angle_cdeg == 0);
    raw[18] = 0U;
    EthernetUartDriver_HandleControl(raw, 19U, ETHERNET_UART_CONTROL_POLICY_LATCHED, 0U);
    assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
    assert(flag == 0U && copied.rudder_valid == 0U);
    assert(g_ethernet_control_policy == ETHERNET_UART_CONTROL_POLICY_NONE);
    assert(!EthernetUartDriver_CopyControlPack(NULL, &flag));
    assert(!EthernetUartDriver_CopyControlPack(&copied, NULL));
    assert(critical_depth == 0U);
}

static void test_consumer_expiry_and_tick_wrap(void)
{
    uint8_t raw[21] = {0U};
    EthernetUartReadoutPack copied;
    uint8_t flag;
    unsigned i;
    const uint32_t starts[] = {10000U, UINT32_MAX - 1000U};
    raw[18] = 1U;
    set_angle(raw, 500);
    for (i = 0U; i < sizeof(starts) / sizeof(starts[0]); ++i)
    {
        reset_state();
        fake_tick = starts[i];
        EthernetUartDriver_HandleControl(raw, 21U, ETHERNET_UART_CONTROL_POLICY_STREAM, 1U);
        fake_tick = starts[i] + ETHERNET_UART_CONTROL_TIMEOUT_MS;
        assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
        assert(flag == 1U && copied.rudder_valid == 1U);
        /* 不运行以太网轮询，仅控制消费者也必须拒绝过期尾舵。 */
        ++fake_tick;
        assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
        assert(copied.rudder_valid == 0U);
        assert(critical_depth == 0U);
        fake_tick += 100U;
        set_angle(raw, -500);
        EthernetUartDriver_HandleControl(raw, 21U, ETHERNET_UART_CONTROL_POLICY_STREAM, 1U);
        assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
        assert(flag == 1U && copied.rudder_valid == 1U && copied.rudder_angle_cdeg == -500);
        /* 即使邮箱还保留角度，非 STREAM 或释放控制权时也不能继续使用尾舵。 */
        g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_LATCHED;
        assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
        assert(copied.rudder_valid == 0U && copied.rudder_angle_cdeg == 0);
        g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_STREAM;
        g_ethernet_rx_message_flag = 0U;
        assert(EthernetUartDriver_CopyControlPack(&copied, &flag));
        assert(flag == 0U && copied.rudder_valid == 0U && copied.rudder_angle_cdeg == 0);
        set_angle(raw, 500);
    }
}

int main(void)
{
    test_parse_boundaries();
    test_publish_reject_and_legacy();
    test_consumer_expiry_and_tick_wrap();
    puts("rudder protocol tests passed");
    return 0;
}
"""


if __name__ == "__main__":
    unittest.main()
