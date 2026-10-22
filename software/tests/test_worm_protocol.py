"""执行真实 UGWRM3 发送函数，用站端解析器检查字段、极值与异步缓冲。"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "station"))
import contcp_t


def extract_function(source, name):
    masked = re.sub(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda m: " " * len(m.group()), source, flags=re.S,
    )
    match = re.search(r"^(?:static\s+)?(?:int|HAL_StatusTypeDef)\s+" +
                      re.escape(name) + r"\s*\([^;{}]*\)\s*\{", masked, re.M)
    if not match:
        raise AssertionError(name)
    start = masked.index("{", match.start())
    depth = 1
    for end in range(start + 1, len(masked)):
        depth += (masked[end] == "{") - (masked[end] == "}")
        if not depth:
            return source[match.start():end + 1]
    raise AssertionError(name)


class WormFirmwareProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise AssertionError("UGWRM 协议测试需要主机 GCC")
        cls.tmp = tempfile.TemporaryDirectory(prefix="worm_protocol_", dir=Path(__file__).parent)
        cls.addClassCleanup(cls.tmp.cleanup)
        tmp = Path(cls.tmp.name)
        cls.exe = tmp / "worm_protocol.exe"
        cls.env = {**os.environ, "TMP": str(tmp), "TEMP": str(tmp), "TMPDIR": str(tmp)}
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", "")
        (tmp / "main.h").write_text(
            '#include <stdint.h>\ntypedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;\n'
            'typedef struct { unsigned unused; } UART_HandleTypeDef;\n', encoding="utf-8")
        source = (ROOT / "software/Glider-Core/ethernet_uart_driver.c").read_text(encoding="utf-8-sig")
        funcs = "\n".join(extract_function(source, name) for name in (
            "EthernetUartDriver_AppendUint", "EthernetUartDriver_AppendFixed",
            "EthernetUartDriver_SendRaw", "EthernetUartDriver_SendWormTelemetry",
        ))
        path = tmp / "worm_protocol.c"
        path.write_text(STUBS + funcs + SCENARIOS, encoding="utf-8")
        proc = subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                               "-I", str(tmp), "-I", str(ROOT / "software/Glider-Core"),
                               str(path), "-o", str(cls.exe)], capture_output=True, env=cls.env)
        if proc.returncode:
            raise AssertionError((proc.stdout + proc.stderr).decode("utf-8", errors="replace"))

    def scenario(self, name):
        result = subprocess.run([str(self.exe), name], capture_output=True, env=self.env)
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", errors="replace"))
        rows = []
        for line in result.stdout.splitlines(keepends=True):
            self.assertTrue(line.endswith(b"\r\n"))
            self.assertLess(len(line), 512)
            row = contcp_t.ugwrm_parse(line.decode("ascii"))
            self.assertIsNotNone(row, line)
            rows.append(row)
        return rows

    def test_real_frame_roundtrips_both_axes_and_total_state(self):
        row, = self.scenario("normal")
        self.assertEqual((row["schema"], row["pulse"], row["stroke_percent"]), (3, -1234, 12.5))
        self.assertEqual((row["roll_pulse"], row["roll_span_pulses"], row["roll_percent"]), (300, 2400, 50.0))
        self.assertEqual((row["calib_state"], row["calib_hold"], row["roll_valid"]), (2, 1, 1))
        self.assertEqual((row["cmd_rpm"], row["fb_rpm"], row["limit_bits"]), (35, 34, 5))

    def test_signed_extreme_pulses_do_not_go_through_float(self):
        row, = self.scenario("maximum")
        self.assertEqual((row["pulse"], row["roll_pulse_min"], row["roll_pulse_max"]),
                         (-2147483648, -2147483648, 2147483647))
        self.assertEqual((row["t_ms"], row["poll_ok"], row["last_error"]),
                         (0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))

    def test_busy_preserves_inflight_frame_and_failed_send_keeps_sequence(self):
        rows = self.scenario("errors")
        self.assertEqual([r["seq"] for r in rows], [1, 2])
        self.assertEqual([r["roll_percent"] for r in rows], [50.0, 75.0])


STUBS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
#include "ethernet_uart_driver.h"
#include "worm_axis.h"
volatile EthernetUartState g_ethernet_uart_state;
static UART_HandleTypeDef fake_uart;
static UART_HandleTypeDef *s_uart = &fake_uart;
static uint8_t s_tx_busy;
static unsigned critical_depth, copies;
static uint32_t tick = 10000U;
static HAL_StatusTypeDef tx_result = HAL_OK;
static const uint8_t *inflight;
static uint16_t inflight_len;
static char saved[512];
static WormAxisState fake_worm;
static int32_t g_rc_worm_cmd_rpm;
#define LIMIT_SWITCH_COUNT 4U
static uint8_t g_limit_switch_states[4] = {1,0,1,0};
static struct {
  struct {
    int32_t feedback_speed_rpm;
    uint16_t status_word;
    struct { uint16_t reg_0007_error_code, reg_0008_error_subcode, reg_0009_input_ports;
             uint32_t poll_ok_count, poll_fail_count; } debug;
  } nodes[1];
  struct { uint32_t overrun_count, enable_resync_count, last_error; } bus_diag;
} g_umotor;
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
static uint32_t HAL_GetTick(void) { return tick; }
void WormAxis_CopyState(WormAxisState *out) {
  assert(critical_depth == 1U); ++copies; *out = fake_worm;
}
static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                             const uint8_t *data, uint16_t size) {
  assert(uart == s_uart && critical_depth == 0U && size < sizeof(saved));
  if (tx_result == HAL_OK) {
    inflight = data; inflight_len = size;
    memcpy(saved, data, size);
    assert(fwrite(data, 1, size, stdout) == size);
  }
  return tx_result;
}
'''

SCENARIOS = r'''
int main(int argc, char **argv) {
  assert(argc == 2);
#ifdef _WIN32
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  fake_worm.pulse = -1234;
  fake_worm.stroke_percent = 12.5f;
  fake_worm.mm_from_front = 7.35f;
  fake_worm.span_valid = fake_worm.data_fresh = 1;
  fake_worm.reference_kind = WORM_REF_HOMED;
  fake_worm.roll = (WormRollState){.pulse=300, .pulse_min=-900, .pulse_max=1500,
    .span_pulses=2400, .stroke_percent=50.0f, .valid=1, .home_state=WORM_ROLL_HOME_DONE};
  fake_worm.calib_state = WORM_CALIB_DONE;
  fake_worm.calib_hold = 1;
  g_rc_worm_cmd_rpm = 35;
  g_umotor.nodes[0].feedback_speed_rpm = 34;
  if (strcmp(argv[1], "maximum") == 0) {
    tick = UINT32_MAX;
    fake_worm.pulse = fake_worm.pulse_min_seen = fake_worm.pulse_home_front = INT32_MIN;
    fake_worm.pulse_max_seen = fake_worm.span_pulses = INT32_MAX;
    fake_worm.reference_lost_count = UINT32_MAX;
    fake_worm.roll.pulse = fake_worm.roll.pulse_min = INT32_MIN;
    fake_worm.roll.pulse_max = fake_worm.roll.span_pulses = INT32_MAX;
    fake_worm.stroke_percent = fake_worm.mm_from_front = fake_worm.roll.stroke_percent = -1.0e6f;
    g_rc_worm_cmd_rpm = g_umotor.nodes[0].feedback_speed_rpm = INT32_MIN;
    memset(&g_umotor.nodes[0].debug, 0xff, sizeof(g_umotor.nodes[0].debug));
    memset(&g_umotor.bus_diag, 0xff, sizeof(g_umotor.bus_diag));
  }
  assert(EthernetUartDriver_SendWormTelemetry() == HAL_OK);
  assert(copies == 1U);
  if (strcmp(argv[1], "errors") == 0) {
    fake_worm.roll.stroke_percent = 75.0f;
    assert(EthernetUartDriver_SendWormTelemetry() == HAL_BUSY);
    assert(copies == 1U && memcmp(inflight, saved, inflight_len) == 0);
    s_tx_busy = 0;
    tx_result = HAL_ERROR;
    assert(EthernetUartDriver_SendWormTelemetry() == HAL_ERROR);
    tx_result = HAL_OK;
    assert(EthernetUartDriver_SendWormTelemetry() == HAL_OK);
  }
  return 0;
}
'''


if __name__ == "__main__":
    unittest.main()
