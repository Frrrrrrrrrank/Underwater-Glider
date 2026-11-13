"""真实raw FIFO/formatter + Ethernet sender/订阅/任务调度；临时产物仅放D工作区。"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from test_battery_protocol import C_HAL_HEADER, extract_function

CORE = Path(__file__).resolve().parents[1] / "Glider-Core"
TMP_ROOT = CORE.parents[2] / ".buildtmp"


def build_harness(work):
    """返回实际sender夹具；fixture/maximum/timeline输出真实UGRAW字节供PC交叉验证。"""
    compiler = shutil.which("gcc") or "D:/msys64/mingw64/bin/gcc.exe"
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    (work / "stm32h7xx_hal.h").write_text(C_HAL_HEADER, encoding="utf-8")
    (work / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
    (work / "FreeRTOS.h").write_text("/* 无调度器的主机夹具 */\n", encoding="utf-8")
    (work / "task.h").write_text(
        "void test_enter(void); void test_exit(void);\n"
        "#define taskENTER_CRITICAL() test_enter()\n"
        "#define taskEXIT_CRITICAL() test_exit()\n", encoding="utf-8")
    production = (CORE / "ethernet_uart_driver.c").read_text(encoding="utf-8-sig")
    functions = "\n\n".join(extract_function(production, name) for name in (
        "EthernetUartDriver_SendRaw", "raw_capture_send", "EthernetUartDriver_SendRawCapture",
        "EthernetUartDriver_HandleTelemetrySubscription", "EthernetUartDriver_RunTask"))
    source = work / "raw_capture_fixture.c"
    source.write_text(STUBS + '\n#include "raw_capture.c"\n' + functions + SCENARIOS, encoding="utf-8")
    binary = work / ("raw_capture_fixture.exe" if os.name == "nt" else "raw_capture_fixture")
    env = dict(os.environ, TMP=str(work), TEMP=str(work), TMPDIR=str(work))
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    result = subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                             "-I", str(work), "-I", str(CORE), str(source), "-o", str(binary)],
                            capture_output=True, env=env)
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).decode("utf-8", errors="replace"))
    return binary, env


class RawCaptureFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        TMP_ROOT.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="raw_capture_", dir=TMP_ROOT)
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.binary, cls.env = build_harness(Path(cls.tmp.name))

    def run_case(self, scenario):
        result = subprocess.run([str(self.binary), scenario], capture_output=True, env=self.env)
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", errors="replace"))
        return result.stdout

    def parse_frame(self, frame):
        self.assertTrue(frame.startswith(b"$UGRAW,") and frame.endswith(b"\r\n"))
        body, suffix = frame[1:-2].rsplit(b"*", 1)
        checksum = 0
        for value in body:
            checksum ^= value
        self.assertEqual(suffix, ("%02X" % checksum).encode("ascii"))
        fields = body.split(b",")[1:]
        self.assertEqual(len(fields), 14)
        values = [int(field) for field in fields[:-1]]
        data = bytes.fromhex(fields[-1].decode("ascii"))
        self.assertEqual(values[12], len(data))
        self.assertEqual(values[12], min(values[11], 384))
        return values, data

    def test_actual_sender_preserves_binary_bad_checksum_and_64bit_times(self):
        values, data = self.parse_frame(self.run_case("fixture"))
        self.assertEqual(data, b"$GNGGA,\x00\xff*ZZ\r")
        self.assertEqual(values[:3], [1, 1, 1])
        self.assertEqual((values[3] << 32) | values[4], 0x12345678ABCDEF00)
        self.assertEqual((values[5] << 32) | values[6], 0x23456789ABCDEF01)
        self.assertEqual(values[8:11], [7, 3, 0])

    def test_maximum_header_and_hex_fit_actual_1024_sender(self):
        wire = self.run_case("maximum")
        values, data = self.parse_frame(wire)
        self.assertLess(len(wire), 1024)
        self.assertEqual(values[1:10], [0xFFFFFFFF] * 9)
        self.assertEqual(values[10:13], [1, 0xFFFFFFFF, 384])
        self.assertEqual(data, bytes(i & 255 for i in range(384)))

    def test_actual_sender_timeline_keeps_duplicate_no_fix_and_recovery(self):
        frames = self.run_case("timeline").splitlines(keepends=True)
        self.assertEqual(len(frames), 4)
        for index, (frame, rx_ms, utc, status) in enumerate(zip(
                frames, (1000, 1100, 2000, 6000),
                ("120000", "120000", "120001", "120005"), ("A", "A", "V", "A")), 1):
            values, raw = self.parse_frame(frame)
            body = ("GNRMC,%s,%s,3015.0000,N,12030.0000,E,1.0,90.0,061126,,,A" %
                    (utc, status)).encode("ascii")
            checksum = 0
            for byte in body:
                checksum ^= byte
            self.assertEqual(raw, b"$" + body + ("*%02X\r" % checksum).encode("ascii"))
            self.assertEqual(values[:3], [1, 1, index])
            self.assertEqual((values[3] << 32) | values[4], rx_ms * 1000)
            self.assertEqual((values[5] << 32) | values[6], rx_ms * 1000 + 10000)
            self.assertEqual(values[7:11], [0, 0, 0, 0])

    def test_capture_lifecycle_renewal_and_rejected_subscription(self): self.run_case("session")
    def test_lease_wrap_and_preempted_older_rx_time(self): self.run_case("time")
    def test_fifo_overflow_drop_oldest_and_sequence_wrap(self): self.run_case("overflow")
    def test_busy_error_retry_and_inflight_buffer_lifetime(self): self.run_case("failure")
    def test_wire_budget_cannot_be_reset_by_renew_or_restart(self): self.run_case("budget")
    def test_rx_preemption_cannot_pop_or_send_replaced_head(self): self.run_case("race")
    def test_real_task_raw_is_last_and_lease_expires_even_when_uart_busy(self): self.run_case("priority")


STUBS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
#include "ethernet_uart_driver.h"
#include "raw_capture.h"
volatile EthernetUartState g_ethernet_uart_state;
volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack;
volatile uint8_t g_ethernet_rx_message_flag, g_ethernet_control_policy;
volatile uint32_t g_ethernet_control_last_rx_tick;
static UART_HandleTypeDef fake_uart;
static UART_HandleTypeDef *s_uart = &fake_uart;
static uint8_t s_tx_busy, s_snapshot_pending, s_rx_rearm_pending;
static uint8_t s_ekf_telem_enabled, s_mag_telem_enabled, s_ctl_telem_enabled;
static uint8_t s_wrm_telem_enabled, s_bat_telem_enabled, s_rdss_telem_enabled;
static uint32_t s_last_command_tick, tick = 5000U;
static uint64_t tx_time = 0x23456789ABCDEF01ULL;
static unsigned critical_depth, critical_calls, tx_attempts, setup_calls;
static unsigned inject_mode, print_wire;
static HAL_StatusTypeDef tx_status = HAL_OK;
static const uint8_t *inflight;
static uint16_t inflight_len;
static char last_wire[1024], events[128];
static unsigned event_count;
static jmp_buf escape_task;
static void inject(void);
void test_enter(void) { assert(critical_depth == 0); ++critical_depth; ++critical_calls; }
void test_exit(void) { assert(critical_depth == 1); --critical_depth; if (inject_mode) inject(); }
static uint32_t HAL_GetTick(void) { return tick; }
static uint64_t GliderTime_NowUs(void) { return tx_time; }
static void event(char c) { assert(event_count < sizeof(events)); events[event_count++] = c; }
static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t n)
{
  assert(critical_depth == 1 && uart == &fake_uart && n < sizeof(last_wire));
  ++tx_attempts;
  memcpy(last_wire, data, n); last_wire[n] = 0;
  event('N');
  if (tx_status == HAL_OK) {
    inflight = data; inflight_len = n;
    if (print_wire) assert(fwrite(data, 1, n, stdout) == n);
  }
  return tx_status;
}
static void EthernetUartDriver_ProcessReceivedData(void) { }
static void EthernetUartDriver_RearmReceive(void) { s_rx_rearm_pending = 0U; }
void EthernetUartDriver_SendText(const char *s) { assert(s); ++setup_calls; s_tx_busy = 1; }
void EthernetUartDriver_SendSystemSnapshot(void) { event('S'); s_tx_busy = 1; }
void EthernetUartDriver_SendEkfTelemetry(void) { event('E'); s_tx_busy = 1; }
HAL_StatusTypeDef EthernetUartDriver_SendMagTelemetry(void) { event('M'); s_tx_busy = 1; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendControlTelemetry(void) { event('C'); s_tx_busy = 1; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendWormTelemetry(void) { event('W'); s_tx_busy = 1; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendBatteryTelemetry(void) { event('B'); s_tx_busy = 1; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendRdssTelemetry(void) { event('R'); s_tx_busy = 1; return HAL_OK; }
HAL_StatusTypeDef EthernetUartDriver_SendGpsTelemetry(void) { event('G'); s_tx_busy = 1; return HAL_OK; }
static void osDelay(uint32_t ms) { assert(ms == ETHERNET_UART_TASK_PERIOD_MS && !critical_depth); longjmp(escape_task, 1); }
'''

SCENARIOS = r'''
static const uint8_t sample[] = {'$', 'G', 'N', 'G', 'G', 'A', ',', 0, 255, '*', 'Z', 'Z', '\r'};
static void observe(void) {
  RawCapture_Observe(sample, sizeof(sample), sizeof(sample), 0x12345678ABCDEF00ULL, 7, 3, tick);
}
static void subscribe(uint8_t mask) { EthernetUartDriver_HandleTelemetrySubscription(&mask, 1); }
static void transmit(void) { s_tx_busy = 0; EthernetUartDriver_SendRawCapture(); }
static void step(void) { if (setjmp(escape_task) == 0) EthernetUartDriver_RunTask(NULL); }
static void inject(void) {
  const unsigned mode = inject_mode; inject_mode = 0;
  if (mode == 1) observe();
  else { RawCapture_SetEnabled(0, tick); RawCapture_SetEnabled(1, tick); observe(); }
}
int main(int argc, char **argv) {
  assert(argc == 2);
#ifdef _WIN32
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  RawCapture_Init(); assert(critical_calls == 0); /* 启动前不得进入RTOS critical。 */
  if (!strcmp(argv[1], "session")) {
    observe(); assert(!s_count && !s_seq && !s_capture_id);
    subscribe(0x7D); assert(s_capture_id == 1 && s_ekf_telem_enabled && s_ctl_telem_enabled && !s_mag_telem_enabled);
    observe(); tick += 5000; subscribe(0x7D); observe();
    assert(s_capture_id == 1 && s_seq == 2 && s_count == 2);
    { uint8_t bad = 0x80; EthernetUartDriver_HandleTelemetrySubscription(&bad, 1); }
    EthernetUartDriver_HandleTelemetrySubscription(NULL, 0);
    assert(g_ethernet_uart_diag.frame_type_error_count == 2 && s_capture_id == 1);
    subscribe(0x3D); assert(!s_enabled && !s_count); observe(); assert(s_seq == 2);
    subscribe(0x7D); assert(s_capture_id == 2 && !s_seq && !s_queue_drop); observe(); assert(s_seq == 1);
    { unsigned old = s_seq; RawCapture_Observe(sample, 10, 9, 0, 0, 0, tick); assert(s_seq == old); }
  } else if (!strcmp(argv[1], "time")) {
    tick = UINT32_MAX - 10000U; subscribe(0x40);
    RawCapture_Observe(sample, sizeof(sample), sizeof(sample), 0, 0, 0, tick - 1U);
    assert(s_enabled && s_count == 1); /* RX旧now不会把新租约误判超时。 */
    RawCapture_Poll(tick + 14999U); assert(s_enabled);
    RawCapture_Poll(tick + 15000U); assert(!s_enabled && !s_count);
    tick += 15001U; subscribe(0x40); assert(s_capture_id == 2 && !s_seq);
  } else if (!strcmp(argv[1], "fixture")) {
    print_wire = 1; subscribe(0x40); observe(); transmit(); assert(tx_attempts == 1 && !s_count);
  } else if (!strcmp(argv[1], "timeline")) {
    const uint32_t rx_ms[] = {1000U, 1100U, 2000U, 6000U};
    const char *utc[] = {"120000", "120000", "120001", "120005"};
    const char status[] = {'A', 'A', 'V', 'A'};
    tick = 0; subscribe(0x40); print_wire = 1;
    for (unsigned i = 0; i < 4; ++i) {
      char body[128], line[160]; uint8_t checksum = 0;
      int n = snprintf(body, sizeof(body), "GNRMC,%s,%c,3015.0000,N,12030.0000,E,1.0,90.0,061126,,,A", utc[i], status[i]);
      assert(n > 0 && (unsigned)n < sizeof(body));
      for (int j = 0; j < n; ++j) checksum ^= (uint8_t)body[j];
      n = snprintf(line, sizeof(line), "$%s*%02X\r", body, (unsigned)checksum);
      assert(n > 0 && (unsigned)n < sizeof(line));
      tick = rx_ms[i];
      RawCapture_Observe((const uint8_t *)line, (uint32_t)n, (uint16_t)n, (uint64_t)tick * 1000U, 0, 0, tick);
      tick += 10U; tx_time = (uint64_t)tick * 1000U; transmit();
      assert(tx_attempts == i + 1U && !s_count);
    }
  } else if (!strcmp(argv[1], "maximum")) {
    uint8_t bytes[384]; for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;
    subscribe(0x40); s_capture_id = UINT32_MAX; s_seq = UINT32_MAX - 1U; s_queue_drop = UINT32_MAX;
    RawCapture_Observe(bytes, UINT32_MAX, 384, UINT64_MAX, UINT32_MAX, UINT32_MAX, tick);
    tx_time = UINT64_MAX; print_wire = 1; transmit(); assert(tx_attempts == 1);
  } else if (!strcmp(argv[1], "overflow")) {
    subscribe(0x40); for (unsigned i = 0; i < 10; ++i) observe();
    assert(s_count == 8 && s_queue_drop == 2 && s_queue[s_head].seq == 3);
    transmit(); assert(s_count == 7 && s_queue[s_head].seq == 4);
    subscribe(0); subscribe(0x40); s_seq = UINT32_MAX; observe();
    assert(s_seq == 0 && s_queue[s_head].seq == 0); transmit(); assert(!s_count);
  } else if (!strcmp(argv[1], "failure")) {
    char saved[1024]; unsigned tries;
    subscribe(0x40); observe(); tx_status = HAL_BUSY; transmit(); assert(s_count == 1 && !s_wire_used);
    tx_status = HAL_ERROR; transmit(); assert(s_count == 1 && !s_wire_used);
    tx_status = HAL_OK; transmit(); assert(s_count == 0 && s_wire_used && s_tx_busy);
    memcpy(saved, inflight, inflight_len); tries = tx_attempts;
    subscribe(0); subscribe(0x40); observe(); EthernetUartDriver_SendRawCapture();
    assert(tx_attempts == tries && !memcmp(saved, inflight, inflight_len));
    transmit(); assert(!s_count && tx_attempts == tries + 1);
  } else if (!strcmp(argv[1], "budget")) {
    uint8_t bytes[384]; memset(bytes, 255, sizeof(bytes)); subscribe(0x40);
    for (unsigned i = 0; i < 3; ++i) RawCapture_Observe(bytes, 384, 384, 0, 0, 0, tick);
    transmit(); transmit(); assert(tx_attempts == 2 && s_wire_used <= 2048); transmit(); assert(tx_attempts == 2 && s_count == 1);
    subscribe(0x40); transmit(); assert(tx_attempts == 2);
    subscribe(0); subscribe(0x40); RawCapture_Observe(bytes, 384, 384, 0, 0, 0, tick); transmit(); assert(tx_attempts == 2);
    tick += 1000; transmit(); assert(tx_attempts == 3 && !s_count);
  } else if (!strcmp(argv[1], "race")) {
    subscribe(0x40); for (unsigned i = 0; i < 8; ++i) observe();
    inject_mode = 1; transmit(); assert(!tx_attempts && s_count == 8 && s_queue_drop == 1 && s_queue[s_head].seq == 2);
    transmit(); assert(tx_attempts == 1 && s_count == 7 && s_queue[s_head].seq == 3);
    inject_mode = 2; transmit(); assert(tx_attempts == 1 && s_count == 1 && s_capture_id == 2 && s_seq == 1);
    transmit(); assert(tx_attempts == 2 && !s_count);
  } else if (!strcmp(argv[1], "priority")) {
    for (unsigned i = 1; i <= 4; ++i) { tick = i * ETHERNET_UART_HEARTBEAT_PERIOD_MS; s_tx_busy = 0; step(); }
    assert(setup_calls == 4 && !event_count); tick = 5000; subscribe(0x7F); observe(); s_snapshot_pending = 1;
    for (unsigned i = 0; i < 9; ++i) { s_tx_busy = 0; step(); }
    assert(event_count == 9 && !memcmp(events, "BRGSEMCWN", 9));
    observe(); tick += 15000; s_tx_busy = 1; step(); assert(!s_enabled && !s_count);
  } else assert(0);
  assert(!critical_depth); return 0;
}
'''


if __name__ == "__main__":
    unittest.main()
