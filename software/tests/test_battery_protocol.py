"""直接执行板端发送/调度函数，并用站端解析器验证实际生成的电池帧。"""

import importlib.util
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


SOFTWARE_DIR = Path(__file__).resolve().parents[1]
CORE_DIR = SOFTWARE_DIR / "Glider-Core"


def extract_function(source, name):
    """屏蔽字符串和注释后配对括号，保留生产函数原文，不复制实现逻辑。"""
    masked = re.sub(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda match: " " * len(match.group()), source, flags=re.S,
    )
    match = re.search(
        r"^(?:static\s+)?(?:HAL_StatusTypeDef|void)\s+"
        + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", masked, re.M,
    )
    if not match:
        raise AssertionError(f"找不到生产函数：{name}")
    start = masked.index("{", match.start())
    depth = 1
    for end in range(start + 1, len(masked)):
        depth += (masked[end] == "{") - (masked[end] == "}")
        if depth == 0:
            return source[match.start():end + 1]
    raise AssertionError(f"函数括号不完整：{name}")


class BatteryFirmwareProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise AssertionError("电池协议测试需要主机 GCC，不能使用 ARM 交叉编译器")
        cls._tmp = tempfile.TemporaryDirectory(prefix="battery_protocol_", dir=Path(__file__).parent)
        cls.addClassCleanup(cls._tmp.cleanup)
        tmp = Path(cls._tmp.name)
        cls.exe = tmp / ("battery_protocol.exe" if os.name == "nt" else "battery_protocol")
        source = (CORE_DIR / "ethernet_uart_driver.c").read_text(encoding="utf-8-sig")
        functions = "\n\n".join(extract_function(source, name) for name in (
            "EthernetUartDriver_SendRaw", "EthernetUartDriver_SendBatteryTelemetry",
            "EthernetUartDriver_RunTask",
        ))
        (tmp / "stm32h7xx_hal.h").write_text(C_HAL_HEADER, encoding="utf-8")
        (tmp / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
        c_path = tmp / "battery_protocol.c"
        c_path.write_text(C_STUBS + functions + C_SCENARIOS, encoding="utf-8")
        cls.env = {**os.environ, "TMP": str(tmp), "TEMP": str(tmp), "TMPDIR": str(tmp)}
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", "")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(tmp),
             "-I", str(CORE_DIR), str(c_path), "-o", str(cls.exe)],
            capture_output=True, env=cls.env, check=False,
        )
        if result.returncode:
            raise AssertionError((result.stdout + result.stderr).decode("utf-8", errors="replace"))
        parser_path = SOFTWARE_DIR.parent / "station" / "battery_telemetry.py"
        spec = importlib.util.spec_from_file_location("battery_protocol_station", parser_path)
        cls.station = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.station)

    def run_scenario(self, scenario):
        result = subprocess.run([str(self.exe), scenario], capture_output=True, env=self.env, check=False)
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", errors="replace"))
        rows = []
        for frame in result.stdout.splitlines(keepends=True):
            self.assertTrue(frame.endswith(b"\r\n"), frame)
            row = self.station.ugbat_parse(frame.decode("ascii"))
            self.assertIsNotNone(row, frame)
            rows.append(row)
        return rows

    def test_actual_frame_fields_and_checksum_match_station_parser(self):
        self.assertEqual(self.run_scenario("normal"), [{
            "schema": 1, "seq": 1, "t_ms": 10000, "adc_raw": 26200,
            "battery_mv": 29400, "filtered_mv": 29380, "valid": 1,
            "state": 1, "sample_age_ms": 50, "error_count": 2,
        }])

    def test_init_without_sample_uses_unknown_age_and_preserves_state(self):
        rows = self.run_scenario("init")
        self.assertEqual([(r["valid"], r["state"], r["sample_age_ms"]) for r in rows],
                         [(0, 0, 0xFFFFFFFF), (1, 0, 50)])

    def test_warning_critical_and_invalid_are_separate_states(self):
        rows = self.run_scenario("states")
        self.assertEqual([(r["valid"], r["state"], r["filtered_mv"]) for r in rows],
                         [(1, 2, 21800), (1, 3, 20800), (0, 4, 20800)])

    def test_sample_age_boundary_and_tick_wrap(self):
        rows = self.run_scenario("age")
        self.assertEqual([(r["valid"], r["state"], r["sample_age_ms"]) for r in rows],
                         [(1, 1, 500), (0, 4, 501), (1, 1, 100)])

    def test_time_is_read_after_snapshot_can_be_preempted(self):
        row, = self.run_scenario("preemption")
        self.assertEqual((row["t_ms"], row["sample_age_ms"], row["valid"]), (10100, 50, 1))

    def test_busy_keeps_inflight_buffer_and_failures_do_not_skip_sequence(self):
        rows = self.run_scenario("errors")
        self.assertEqual([r["seq"] for r in rows], [1, 2])
        self.assertEqual([r["filtered_mv"] for r in rows], [29380, 21800])

    def test_maximum_numeric_fields_fit_static_frame_buffer(self):
        row, = self.run_scenario("maximum")
        self.assertEqual((row["t_ms"], row["adc_raw"], row["error_count"]),
                         (0xFFFFFFFF, 65535, 0xFFFFFFFF))

    def test_real_task_prioritizes_due_battery_over_continuous_snapshots(self):
        rows = self.run_scenario("priority")
        self.assertEqual([r["t_ms"] for r in rows], [5000, 6000])

    def test_real_task_prioritizes_battery_when_rdss_is_also_due(self):
        rows = self.run_scenario("rdss_priority")
        self.assertEqual([r["t_ms"] for r in rows], [5000, 7000])

    def test_real_task_retries_failed_battery_without_advancing_period(self):
        rows = self.run_scenario("retry")
        self.assertEqual([(r["seq"], r["t_ms"]) for r in rows], [(1, 5001), (2, 6001)])

    def test_real_task_waits_for_uart_and_respects_subscription(self):
        rows = self.run_scenario("subscription")
        self.assertEqual([r["t_ms"] for r in rows], [5001, 8001])

    def test_real_task_period_survives_tick_wrap(self):
        rows = self.run_scenario("schedule_wrap")
        self.assertEqual([r["t_ms"] for r in rows], [0xFFFFFFFF - 499, 500])


C_HAL_HEADER = r"""
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t RxState; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } ADC_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#endif
"""

C_STUBS = r"""
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
#include "ethernet_uart_driver.h"
#include "battery_monitor.h"

/* 仅替换硬件、快照来源和其他遥测流；电池发送及整个任务调度取自生产 C。 */
volatile EthernetUartState g_ethernet_uart_state;
volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
volatile uint8_t g_ethernet_rx_message_flag;
volatile uint32_t g_ethernet_control_last_rx_tick;
volatile uint8_t g_ethernet_control_policy;
static UART_HandleTypeDef fake_uart;
static UART_HandleTypeDef *s_uart = &fake_uart;
static uint8_t s_tx_busy, s_snapshot_pending, s_rx_rearm_pending;
static uint8_t s_ekf_telem_enabled, s_mag_telem_enabled, s_ctl_telem_enabled;
static uint8_t s_wrm_telem_enabled, s_bat_telem_enabled, s_rdss_telem_enabled;
static uint32_t s_last_command_tick;
static uint32_t fake_tick = 10000U, copy_delay;
static unsigned copies, tx_attempts, setup_calls, critical_depth;
static bool copy_ok = true;
static HAL_StatusTypeDef fake_tx_status = HAL_OK;
static const uint8_t *inflight;
static uint16_t inflight_len;
static char attempted[160];
static char events[256];
static size_t event_count;
static jmp_buf task_yield;
static BatteryState fake_battery = {
  .adc_raw = 26200U, .battery_mv = 29400U, .filtered_mv = 29380U,
  .last_sample_ms = 9950U, .sample_count = 20U, .error_count = 2U,
  .state = BATTERY_STATE_NORMAL, .valid = 1U
};
#define taskENTER_CRITICAL() do { ++critical_depth; } while (0)
#define taskEXIT_CRITICAL() do { assert(critical_depth); --critical_depth; } while (0)
static uint32_t HAL_GetTick(void) { return fake_tick; }
bool BatteryMonitor_CopyState(BatteryState *out)
{
  ++copies;
  fake_tick += copy_delay;
  if (!copy_ok) return false;
  *out = fake_battery;
  return true;
}
static void event(char value)
{
  assert(event_count < sizeof(events));
  events[event_count++] = value;
}
static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                              const uint8_t *data, uint16_t size)
{
  assert(uart == &fake_uart && size < sizeof(attempted));
  assert(size > 4U && data[size - 2U] == '\r' && data[size - 1U] == '\n');
  memcpy(attempted, data, size);
  attempted[size] = '\0';
  ++tx_attempts;
  event('B');
  if (fake_tx_status == HAL_OK)
  {
    inflight = data;
    inflight_len = size;
    assert(fwrite(data, 1U, size, stdout) == size);
  }
  return fake_tx_status;
}
static void EthernetUartDriver_ProcessReceivedData(void) { }
static void EthernetUartDriver_RearmReceive(void) { s_rx_rearm_pending = 0U; }
void EthernetUartDriver_SendText(const char *text)
{
  assert(text != NULL);
  ++setup_calls;
  s_tx_busy = 1U;
}
void EthernetUartDriver_SendSystemSnapshot(void) { event('S'); s_tx_busy = 1U; }
void EthernetUartDriver_SendEkfTelemetry(void) { event('E'); s_tx_busy = 1U; }
HAL_StatusTypeDef EthernetUartDriver_SendMagTelemetry(void)
{ event('M'); s_tx_busy = 1U; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendControlTelemetry(void)
{ event('C'); s_tx_busy = 1U; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendWormTelemetry(void)
{ event('W'); s_tx_busy = 1U; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendRdssTelemetry(void)
{ event('R'); s_tx_busy = 1U; return HAL_OK; }
static void osDelay(uint32_t delay)
{
  assert(delay == ETHERNET_UART_TASK_PERIOD_MS && critical_depth == 0U);
  longjmp(task_yield, 1);
}
"""

C_SCENARIOS = r"""
static void send_ok(void)
{
  s_tx_busy = 0U;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_OK);
}
static void step(void)
{
  if (setjmp(task_yield) == 0) EthernetUartDriver_RunTask(NULL);
}
static void scheduler_init(void)
{
  unsigned i;
  for (i = 1U; i <= 4U; ++i)
  {
    fake_tick = i * ETHERNET_UART_HEARTBEAT_PERIOD_MS;
    s_tx_busy = 0U;
    step();
  }
  assert(setup_calls == 4U && event_count == 0U);
  s_tx_busy = 0U;
  s_bat_telem_enabled = s_ekf_telem_enabled = s_mag_telem_enabled = 1U;
  s_ctl_telem_enabled = s_wrm_telem_enabled = 1U;
}
static void at(uint32_t now, bool snapshot)
{
  fake_tick = now;
  fake_battery.last_sample_ms = now;
  s_snapshot_pending = snapshot ? 1U : 0U;
  step();
}
static void errors(void)
{
  char saved[160];
  unsigned old_copies;
  send_ok();
  memcpy(saved, inflight, inflight_len);
  old_copies = copies;
  fake_battery.filtered_mv = 21800U;
  fake_battery.state = BATTERY_STATE_WARN;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_BUSY);
  assert(copies == old_copies && tx_attempts == 1U);
  assert(memcmp(saved, inflight, inflight_len) == 0);
  assert(g_ethernet_uart_diag.tx_busy_count == 1U);
  s_tx_busy = 0U;
  s_uart = NULL;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_ERROR);
  s_uart = &fake_uart;
  copy_ok = false;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_ERROR);
  assert(tx_attempts == 1U);
  copy_ok = true;
  fake_tx_status = HAL_BUSY;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_BUSY);
  assert(strncmp(attempted, "$UGBAT,1,2,", 11U) == 0);
  fake_tx_status = HAL_ERROR;
  assert(EthernetUartDriver_SendBatteryTelemetry() == HAL_ERROR);
  assert(strncmp(attempted, "$UGBAT,1,2,", 11U) == 0);
  assert(g_ethernet_uart_diag.tx_fail_count == 2U && s_tx_busy == 0U);
  fake_tx_status = HAL_OK;
  send_ok();
  assert(tx_attempts == 4U && g_ethernet_uart_diag.tx_count == 2U);
}
int main(int argc, char **argv)
{
#ifdef _WIN32
  assert(_setmode(_fileno(stdout), _O_BINARY) != -1);
#endif
  assert(argc == 2);
  if (strcmp(argv[1], "normal") == 0) send_ok();
  else if (strcmp(argv[1], "init") == 0)
  {
    fake_battery.valid = 0U;
    fake_battery.state = BATTERY_STATE_INIT;
    fake_battery.sample_count = 0U;
    send_ok();
    fake_battery.valid = 1U;
    fake_battery.sample_count = 1U;
    send_ok();
  }
  else if (strcmp(argv[1], "states") == 0)
  {
    fake_battery.filtered_mv = 21800U;
    fake_battery.state = BATTERY_STATE_WARN;
    send_ok();
    fake_battery.filtered_mv = 20800U;
    fake_battery.state = BATTERY_STATE_CRITICAL;
    send_ok();
    fake_battery.valid = 0U;
    fake_battery.state = BATTERY_STATE_INVALID;
    send_ok();
  }
  else if (strcmp(argv[1], "age") == 0)
  {
    fake_battery.last_sample_ms = fake_tick - BATTERY_STALE_MS;
    send_ok();
    ++fake_tick;
    send_ok();
    assert(fake_battery.valid == 1U && fake_battery.state == BATTERY_STATE_NORMAL);
    fake_battery.last_sample_ms = UINT32_MAX - 39U;
    fake_tick = 60U;
    send_ok();
  }
  else if (strcmp(argv[1], "preemption") == 0)
  {
    fake_battery.last_sample_ms = fake_tick + 50U;
    copy_delay = 100U;
    send_ok();
  }
  else if (strcmp(argv[1], "maximum") == 0)
  {
    fake_tick = UINT32_MAX;
    fake_battery.last_sample_ms = UINT32_MAX;
    fake_battery.adc_raw = 65535U;
    fake_battery.battery_mv = fake_battery.filtered_mv = 73514U;
    fake_battery.error_count = UINT32_MAX;
    send_ok();
  }
  else if (strcmp(argv[1], "errors") == 0) errors();
  else if (strcmp(argv[1], "priority") == 0)
  {
    scheduler_init();
    at(5000U, true);
    assert(events[0] == 'B' && s_snapshot_pending == 1U);
    s_tx_busy = 0U;
    at(5020U, true);
    assert(events[1] == 'S' && s_snapshot_pending == 0U);
    s_tx_busy = 0U;
    at(5999U, true);
    assert(events[2] == 'S');
    s_tx_busy = 0U;
    at(6000U, true);
    assert(events[3] == 'B' && event_count == 4U && s_snapshot_pending == 1U);
  }
  else if (strcmp(argv[1], "rdss_priority") == 0)
  {
    scheduler_init();
    s_rdss_telem_enabled = 1U;
    /* BAT 与 RDSS 同时到期先发 BAT；UART 忙时不抢占，下一空闲拍再发 RDSS。 */
    at(5000U, true);
    assert(events[0] == 'B' && event_count == 1U && s_snapshot_pending == 1U);
    at(5001U, true);
    assert(event_count == 1U);
    s_tx_busy = 0U;
    at(5020U, true);
    assert(events[1] == 'R' && event_count == 2U && s_snapshot_pending == 1U);
    s_tx_busy = 0U;
    at(5040U, true);
    assert(events[2] == 'S' && s_snapshot_pending == 0U);
    /* 两条低频流再次同时到期，仍保持 BAT 优先且 RDSS 不被饿死。 */
    s_tx_busy = 0U;
    at(7000U, true);
    assert(events[3] == 'B' && event_count == 4U);
    s_tx_busy = 0U;
    at(7020U, true);
    assert(events[4] == 'R' && event_count == 5U);
  }
  else if (strcmp(argv[1], "retry") == 0)
  {
    scheduler_init();
    fake_tx_status = HAL_ERROR;
    at(5000U, true);
    assert(events[0] == 'B' && s_tx_busy == 0U && s_snapshot_pending == 1U);
    fake_tx_status = HAL_OK;
    at(5001U, true);
    assert(events[1] == 'B');
    s_tx_busy = 0U;
    at(6000U, true);
    assert(events[2] == 'S');
    s_tx_busy = 0U;
    at(6001U, true);
    assert(events[3] == 'B' && event_count == 4U);
  }
  else if (strcmp(argv[1], "subscription") == 0)
  {
    scheduler_init();
    s_tx_busy = 1U;
    at(5000U, true);
    assert(event_count == 0U);
    s_tx_busy = 0U;
    at(5001U, true);
    assert(events[0] == 'B');
    s_tx_busy = 0U;
    s_bat_telem_enabled = 0U;
    at(8000U, false);
    assert(events[1] == 'E' && tx_attempts == 1U);
    s_tx_busy = 0U;
    s_bat_telem_enabled = 1U;
    at(8001U, false);
    assert(events[2] == 'B' && event_count == 3U);
  }
  else if (strcmp(argv[1], "schedule_wrap") == 0)
  {
    scheduler_init();
    at(UINT32_MAX - 499U, false);
    assert(events[0] == 'B');
    s_tx_busy = 0U;
    at(499U, true);
    assert(events[1] == 'S');
    s_tx_busy = 0U;
    at(500U, true);
    assert(events[2] == 'B' && event_count == 3U);
  }
  else assert(false);
  return 0;
}
"""


if __name__ == "__main__":
    unittest.main()
