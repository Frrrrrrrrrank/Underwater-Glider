"""编译生产 UGPS 发送函数，用站端解析器验证帧、单位与异步缓冲保护。"""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_battery_protocol import C_HAL_HEADER, CORE_DIR, SOFTWARE_DIR, extract_function


class GpsFirmwareProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise AssertionError("GNSS 协议测试需要主机 GCC")
        cls._tmp = tempfile.TemporaryDirectory(prefix="gps_protocol_")
        cls.addClassCleanup(cls._tmp.cleanup)
        tmp = Path(cls._tmp.name)
        cls.exe = tmp / ("gps_protocol.exe" if os.name == "nt" else "gps_protocol")
        source = (CORE_DIR / "ethernet_uart_driver.c").read_text(encoding="utf-8-sig")
        functions = "\n\n".join(extract_function(source, name) for name in (
            "EthernetUartDriver_SendRaw", "EthernetUartDriver_SendGpsTelemetry",
        ))
        (tmp / "stm32h7xx_hal.h").write_text(C_HAL_HEADER, encoding="utf-8")
        (tmp / "main.h").write_text('#include "stm32h7xx_hal.h"\n', encoding="utf-8")
        c_path = tmp / "gps_protocol.c"
        c_path.write_text(C_STUBS + functions + C_SCENARIOS, encoding="utf-8")
        cls.env = {**os.environ, "TMP": str(tmp), "TEMP": str(tmp), "TMPDIR": str(tmp)}
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + os.environ.get("PATH", "")
        include_dirs = [tmp, CORE_DIR, SOFTWARE_DIR / "Glider-App" / "common",
                        SOFTWARE_DIR / "Glider-App" / "estimation",
                        SOFTWARE_DIR / "Glider-App" / "estimation" / "ug_estimation" / "include"]
        args = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for directory in include_dirs:
            args.extend(["-I", str(directory)])
        result = subprocess.run(args + [str(c_path), "-o", str(cls.exe)],
                                capture_output=True, env=cls.env, check=False, timeout=120)
        if result.returncode:
            raise AssertionError((result.stdout + result.stderr).decode("utf-8", errors="replace"))
        parser_path = SOFTWARE_DIR.parent / "station" / "gps_telemetry.py"
        spec = importlib.util.spec_from_file_location("gps_protocol_station", parser_path)
        cls.station = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.station)

    def run_scenario(self, scenario):
        result = subprocess.run([str(self.exe), scenario], capture_output=True,
                                env=self.env, check=False, timeout=30)
        self.assertEqual(result.returncode, 0,
                         (result.stdout + result.stderr).decode("utf-8", errors="replace"))
        self.frames = result.stdout.splitlines(keepends=True)
        rows = []
        for frame in self.frames:
            self.assertTrue(frame.endswith(b"\r\n"), frame)
            self.assertLess(len(frame), 1024)
            self.assertEqual(len(frame.split(b"*", 1)[0].split(b",")), 60)
            row = self.station.ugps_parse(frame.decode("ascii"))
            self.assertIsNotNone(row, frame)
            rows.append(row)
        return rows

    def test_actual_frame_fields_units_and_checksum_match_station_parser(self):
        row, = self.run_scenario("normal")
        self.assertEqual(row, {
            "schema": 2, "seq": 42, "t_ms": 10000, "state": 2, "reason": 0,
            "flags": 8183, "sample_seq": 7, "rx_age_ms": 120, "fuse_age_ms": 50,
            "utc_date": 20261105, "utc_ms": 36000123,
            "lat_e7": 220000001, "lon_e7": 1145000001, "origin_id": 1,
            "origin_lat_e7": 220000000, "origin_lon_e7": 1145000000,
            "meas_n_mm": 1250, "meas_e_mm": -2500,
            "est_n_mm": 1125, "est_e_mm": -2250,
            "innov_n_mm": 125, "innov_e_mm": -250, "nis_milli": 1250,
            "r_var_cm2": 250000, "pnn_cm2": 40000, "pne_cm2": -5000,
            "pee_cm2": 90000, "accept_count": 3, "reject_count": 1,
            "reset_count": 0, "duplicate_count": 2, "drop_count": 4,
            "input_reject_count": 5,
            "quality_source": 0, "velocity_source": 1, "vel_fuse_age_ms": 60,
            "no_fix_count": 6, "meas_vn_mmps": 1250, "meas_ve_mmps": -500,
            "est_vn_mmps": 1125, "est_ve_mmps": -375, "vel_nis_milli": 750,
            "vel_r_var_cm2ps2": 40000, "pvn_cm2ps2": 2500,
            "pvne_cm2ps2": -1250, "pve_cm2ps2": 5625,
            "vel_accept_count": 2, "vel_reject_count": 1, "recovery_reason": 2,
            "reset_before_n_mm": 500000, "reset_before_e_mm": -200000,
            "reset_before_vn_mmps": 10000, "reset_before_ve_mmps": -20000,
            "reset_after_n_mm": 1250, "reset_after_e_mm": -2500,
            "reset_after_vn_mmps": 1250, "reset_after_ve_mmps": -500,
            "reset_age_ms": 300, "reset_valid_mask": 255,
        })
        self.assertEqual(self.station.ugps_invalid_fields(row), {})
        self.assertFalse(row["flags"] & 8)  # 尚未通过安装杆臂/真北实测。

    def test_invalid_flags_and_unknown_ages_do_not_turn_history_into_current_fix(self):
        row, = self.run_scenario("unknown")
        self.assertEqual((row["flags"], row["rx_age_ms"], row["fuse_age_ms"]),
                         (0, 0xFFFFFFFF, 0xFFFFFFFF))
        invalid = self.station.ugps_invalid_fields(row)
        self.assertTrue({"lat_e7", "lon_e7", "meas_n_mm", "meas_e_mm",
                         "est_n_mm", "est_e_mm", "origin_id", "pnn_cm2",
                         "pne_cm2", "pee_cm2", "nis_milli", "innov_n_mm",
                         "innov_e_mm", "rx_age_ms", "fuse_age_ms"} <= invalid.keys())
        self.assertEqual(row["meas_n_mm"], 1250)  # 原始有限历史值保留供审计。
        self.assertEqual((row["vel_fuse_age_ms"], row["reset_age_ms"]),
                         (0xFFFFFFFF, 0xFFFFFFFF))
        self.assertTrue({"meas_vn_mmps", "est_vn_mmps", "vel_nis_milli",
                         "pvn_cm2ps2", "reset_before_n_mm", "reset_after_n_mm",
                         "vel_fuse_age_ms", "reset_age_ms"} <= invalid.keys())

    def test_overflow_and_nonfinite_revoke_corresponding_validity(self):
        rows = self.run_scenario("overflow")
        fields_and_flags = [
            ("meas_n_mm", 32), ("est_e_mm", 2 | 4), ("innov_n_mm", 64),
            ("nis_milli", 64), ("r_var_cm2", 16 | 4),
            ("pnn_cm2", 16 | 4), ("pne_cm2", 16 | 4), ("pee_cm2", 16 | 4),
        ]
        self.assertEqual(len(rows), len(fields_and_flags))
        for row, (field, revoked) in zip(rows, fields_and_flags):
            self.assertEqual(row[field], 0, field)
            self.assertEqual(row["flags"], 8183 & ~revoked, field)
            if field != "r_var_cm2":
                self.assertIn(field, self.station.ugps_invalid_fields(row))

    def test_negative_nis_and_variances_are_not_emitted_as_unsigned_fields(self):
        rows = self.run_scenario("negative")
        self.assertEqual(len(rows), 4)
        for row, field in zip(rows, ("nis_milli", "r_var_cm2", "pnn_cm2", "pee_cm2")):
            self.assertEqual(row[field], 0)
            self.assertEqual(row["flags"], 8183 & ~(64 if field == "nis_milli" else 20))

    def test_large_wire_values_fit_buffer_and_preserve_sign(self):
        row, = self.run_scenario("maximum")
        self.assertEqual((row["seq"], row["t_ms"], row["sample_seq"],
                          row["input_reject_count"]), (0xFFFFFFFF,) * 4)
        self.assertEqual((row["meas_n_mm"], row["est_e_mm"], row["r_var_cm2"],
                          row["pne_cm2"]), (2000000000, -2000000000, 2000000000, -2000000000))
        self.assertEqual((row["lat_e7"], row["lon_e7"]), (-900000000, 1800000000))
        self.assertGreater(len(self.frames[0]), 512)
        self.assertEqual((row["vel_fuse_age_ms"], row["no_fix_count"],
                          row["vel_accept_count"], row["reset_age_ms"]), (0xFFFFFFFF,) * 4)
        self.assertEqual((row["meas_vn_mmps"], row["est_ve_mmps"],
                          row["vel_r_var_cm2ps2"], row["pvne_cm2ps2"],
                          row["reset_before_n_mm"], row["reset_after_ve_mmps"]),
                         (2000000000, -2000000000, 2000000000, -2000000000,
                          2000000000, -2000000000))

    def test_reset_mask_preserves_each_finite_component_without_false_zero(self):
        rows = self.run_scenario("reset_mask")
        fields = self.station.UGPS_RESET_FIELDS
        self.assertEqual(len(rows), len(fields))
        for index, (row, field) in enumerate(zip(rows, fields)):
            self.assertEqual(row[field], 0, field)
            self.assertEqual(row["reset_valid_mask"], 255 & ~(1 << index), field)
            self.assertEqual(row["flags"], 8183, field)
            invalid = self.station.ugps_invalid_fields(row)
            self.assertEqual(set(invalid), {field})
        partial, = self.run_scenario("reset_partial")
        self.assertEqual(partial["reset_valid_mask"], 15)
        self.assertEqual(partial["reset_after_n_mm"], 1250)
        self.assertIn("reset_after_n_mm", self.station.ugps_invalid_fields(partial))

    def test_velocity_overflow_and_negative_variance_revoke_validity(self):
        rows = self.run_scenario("velocity_invalid")
        fields_and_flags = [
            ("meas_vn_mmps", 512), ("meas_ve_mmps", 512),
            ("est_vn_mmps", 256 | 4), ("est_ve_mmps", 256 | 4),
            ("vel_nis_milli", 4096), ("vel_r_var_cm2ps2", 2048 | 256 | 4),
            ("pvn_cm2ps2", 2048 | 256 | 4), ("pvne_cm2ps2", 2048 | 256 | 4),
            ("pve_cm2ps2", 2048 | 256 | 4), ("vel_nis_milli", 4096),
            ("vel_r_var_cm2ps2", 2048 | 256 | 4),
            ("pvn_cm2ps2", 2048 | 256 | 4), ("pve_cm2ps2", 2048 | 256 | 4),
        ]
        self.assertEqual(len(rows), len(fields_and_flags))
        for row, (field, revoked) in zip(rows, fields_and_flags):
            self.assertEqual(row[field], 0, field)
            self.assertEqual(row["flags"], 8183 & ~revoked, field)
            if field != "vel_r_var_cm2ps2":
                self.assertIn(field, self.station.ugps_invalid_fields(row))

    def test_busy_preserves_inflight_buffer_and_copy_failure_does_not_transmit(self):
        rows = self.run_scenario("errors")
        self.assertEqual([row["seq"] for row in rows], [42, 43])
        self.assertEqual([row["meas_n_mm"] for row in rows], [1250, 3750])


C_STUBS = r"""
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
#include "ekf_bridge.h"

/* 主机断言失败直接回报测试，避免 Windows CRT 错误对话框挂住自动回归。 */
#undef assert
#define assert(condition) do { if (!(condition)) { \
  fprintf(stderr, "assertion failed: %s at %s:%d\n", #condition, __FILE__, __LINE__); \
  exit(1); \
} } while (0)

/* 仅替换 HAL 和快照来源；帧构造与 SendRaw 均编译生产原文及真实结构定义。 */
volatile EthernetUartState g_ethernet_uart_state;
static UART_HandleTypeDef fake_uart;
static UART_HandleTypeDef *s_uart = &fake_uart;
static uint8_t s_tx_busy;
static unsigned copies, tx_attempts;
static uint32_t copy_ok = 1U;
static HAL_StatusTypeDef fake_tx_status = HAL_OK;
static const uint8_t *inflight;
static uint16_t inflight_len;
static StateEstimate fake_state;

uint32_t EkfBridge_CopyState(StateEstimate *out)
{
  ++copies;
  if (copy_ok == 0U) return 0U;
  *out = fake_state;
  return fake_state.update_seq;
}
static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                              const uint8_t *data, uint16_t size)
{
  assert(uart == &fake_uart && size < 1024U);
  assert(size > 4U && data[size - 2U] == '\r' && data[size - 1U] == '\n');
  ++tx_attempts;
  if (fake_tx_status == HAL_OK)
  {
    inflight = data;
    inflight_len = size;
    assert(fwrite(data, 1U, size, stdout) == size);
  }
  return fake_tx_status;
}
"""

C_SCENARIOS = r"""
static void defaults(void)
{
  memset(&fake_state, 0, sizeof(fake_state));
  fake_state.update_seq = 42U;
  fake_state.update_tick_ms = 10000U;
  fake_state.out.pN = 1.125f;
  fake_state.out.pE = -2.25f;
  fake_state.out.Pnn = 4.0f;
  fake_state.out.Pne = -0.5f;
  fake_state.out.Pee = 9.0f;
  fake_state.out.vN = 1.125f;
  fake_state.out.vE = -0.375f;
  fake_state.out.Pvn = 0.25f;
  fake_state.out.Pvne = -0.125f;
  fake_state.out.Pve = 0.5625f;
  fake_state.gnss = (GnssNavDiagnostics) {
    .state = GNSS_NAV_TRACKING, .reason = GNSS_REASON_NONE, .flags = 8183U,
    .sample_seq = 7U, .rx_age_ms = 120U, .fuse_age_ms = 50U,
    .utc_date = 20261105U, .utc_ms = 36000123U,
    .lat_e7 = 220000001, .lon_e7 = 1145000001, .origin_id = 1U,
    .origin_lat_e7 = 220000000, .origin_lon_e7 = 1145000000,
    .meas_n = 1.25f, .meas_e = -2.5f, .innov_n = 0.125f, .innov_e = -0.25f,
    .nis = 1.25f, .r_var = 25.0f, .accept_count = 3U, .reject_count = 1U,
    .duplicate_count = 2U, .drop_count = 4U, .input_reject_count = 5U,
    .quality_source = 0U, .velocity_source = 1U, .vel_fuse_age_ms = 60U,
    .no_fix_count = 6U, .meas_vn = 1.25f, .meas_ve = -0.5f,
    .vel_nis = 0.75f, .vel_r_var = 4.0f, .vel_accept_count = 2U,
    .vel_reject_count = 1U, .recovery_reason = GNSS_RECOVERY_AID_LOSS,
    .reset_before_n = 500.0f, .reset_before_e = -200.0f,
    .reset_before_vn = 10.0f, .reset_before_ve = -20.0f,
    .reset_after_n = 1.25f, .reset_after_e = -2.5f,
    .reset_after_vn = 1.25f, .reset_after_ve = -0.5f,
    .reset_age_ms = 300U, .reset_valid_mask = 255U
  };
}
static void send_ok(void)
{
  s_tx_busy = 0U;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_OK);
}
static void errors(void)
{
  char saved[1024];
  unsigned old_copies;
  send_ok();
  memcpy(saved, inflight, inflight_len);
  old_copies = copies;
  fake_state.gnss.meas_n = 3.75f;
  ++fake_state.update_seq;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_BUSY);
  assert(copies == old_copies && tx_attempts == 1U);
  assert(memcmp(saved, inflight, inflight_len) == 0);
  assert(g_ethernet_uart_diag.tx_busy_count == 1U);
  s_tx_busy = 0U;
  s_uart = NULL;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_ERROR);
  s_uart = &fake_uart;
  copy_ok = 0U;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_ERROR);
  assert(tx_attempts == 1U);
  copy_ok = 1U;
  fake_tx_status = HAL_BUSY;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_BUSY);
  fake_tx_status = HAL_ERROR;
  assert(EthernetUartDriver_SendGpsTelemetry() == HAL_ERROR);
  assert(g_ethernet_uart_diag.tx_fail_count == 2U && s_tx_busy == 0U);
  fake_tx_status = HAL_OK;
  send_ok();
  assert(tx_attempts == 4U && g_ethernet_uart_diag.tx_count == 2U);
}
int main(int argc, char **argv)
{
  unsigned i;
#ifdef _WIN32
  assert(_setmode(_fileno(stdout), _O_BINARY) != -1);
#endif
  assert(argc == 2);
  defaults();
  if (strcmp(argv[1], "normal") == 0) send_ok();
  else if (strcmp(argv[1], "unknown") == 0)
  {
    fake_state.gnss.flags = 0U;
    fake_state.gnss.state = GNSS_NAV_WAIT_FIX;
    fake_state.gnss.rx_age_ms = fake_state.gnss.fuse_age_ms = UINT32_MAX;
    fake_state.gnss.vel_fuse_age_ms = fake_state.gnss.reset_age_ms = UINT32_MAX;
    send_ok();
  }
  else if (strcmp(argv[1], "overflow") == 0)
  {
    for (i = 0U; i < 8U; ++i)
    {
      defaults();
      switch (i)
      {
        case 0: fake_state.gnss.meas_n = 3000000.0f; break;
        case 1: fake_state.out.pE = -3000000.0f; break;
        case 2: fake_state.gnss.innov_n = NAN; break;
        case 3: fake_state.gnss.nis = INFINITY; break;
        case 4: fake_state.gnss.r_var = 300000.0f; break;
        case 5: fake_state.out.Pnn = NAN; break;
        case 6: fake_state.out.Pne = -INFINITY; break;
        case 7: fake_state.out.Pee = 300000.0f; break;
      }
      send_ok();
    }
  }
  else if (strcmp(argv[1], "negative") == 0)
  {
    for (i = 0U; i < 4U; ++i)
    {
      defaults();
      if (i == 0U) fake_state.gnss.nis = -1.0f;
      else if (i == 1U) fake_state.gnss.r_var = -1.0f;
      else if (i == 2U) fake_state.out.Pnn = -1.0f;
      else fake_state.out.Pee = -1.0f;
      send_ok();
    }
  }
  else if (strcmp(argv[1], "maximum") == 0)
  {
    fake_state.update_seq = fake_state.update_tick_ms = UINT32_MAX;
    fake_state.gnss.sample_seq = fake_state.gnss.origin_id = UINT32_MAX;
    fake_state.gnss.accept_count = fake_state.gnss.reject_count = UINT32_MAX;
    fake_state.gnss.reset_count = fake_state.gnss.duplicate_count = UINT32_MAX;
    fake_state.gnss.drop_count = fake_state.gnss.input_reject_count = UINT32_MAX;
    fake_state.gnss.lat_e7 = fake_state.gnss.origin_lat_e7 = -900000000;
    fake_state.gnss.lon_e7 = fake_state.gnss.origin_lon_e7 = 1800000000;
    fake_state.gnss.meas_n = fake_state.out.pN = 2000000.0f;
    fake_state.gnss.meas_e = fake_state.out.pE = -2000000.0f;
    fake_state.gnss.innov_n = fake_state.gnss.nis = 2000000.0f;
    fake_state.gnss.innov_e = -2000000.0f;
    fake_state.gnss.r_var = fake_state.out.Pnn = fake_state.out.Pee = 200000.0f;
    fake_state.out.Pne = -200000.0f;
    fake_state.gnss.vel_fuse_age_ms = fake_state.gnss.no_fix_count = UINT32_MAX;
    fake_state.gnss.vel_accept_count = fake_state.gnss.vel_reject_count = UINT32_MAX;
    fake_state.gnss.reset_age_ms = UINT32_MAX;
    fake_state.gnss.meas_vn = fake_state.out.vN = 2000000.0f;
    fake_state.gnss.meas_ve = fake_state.out.vE = -2000000.0f;
    fake_state.gnss.vel_nis = 2000000.0f;
    fake_state.gnss.vel_r_var = fake_state.out.Pvn = fake_state.out.Pve = 200000.0f;
    fake_state.out.Pvne = -200000.0f;
    fake_state.gnss.reset_before_n = fake_state.gnss.reset_before_vn = 2000000.0f;
    fake_state.gnss.reset_before_e = fake_state.gnss.reset_before_ve = -2000000.0f;
    fake_state.gnss.reset_after_n = fake_state.gnss.reset_after_vn = 2000000.0f;
    fake_state.gnss.reset_after_e = fake_state.gnss.reset_after_ve = -2000000.0f;
    send_ok();
  }
  else if (strcmp(argv[1], "reset_partial") == 0)
  {
    fake_state.gnss.reset_valid_mask = 15U;
    send_ok();
  }
  else if (strcmp(argv[1], "reset_mask") == 0)
  {
    for (i = 0U; i < 8U; ++i)
    {
      defaults();
      /* 每个历史分量单独撤有效位，其他有限分量仍可核查。 */
      switch (i)
      {
        case 0: fake_state.gnss.reset_before_n = NAN; break;
        case 1: fake_state.gnss.reset_before_e = INFINITY; break;
        case 2: fake_state.gnss.reset_before_vn = -INFINITY; break;
        case 3: fake_state.gnss.reset_before_ve = -3000000.0f; break;
        case 4: fake_state.gnss.reset_after_n = 3000000.0f; break;
        case 5: fake_state.gnss.reset_after_e = NAN; break;
        case 6: fake_state.gnss.reset_after_vn = INFINITY; break;
        case 7: fake_state.gnss.reset_after_ve = -INFINITY; break;
      }
      send_ok();
    }
  }
  else if (strcmp(argv[1], "velocity_invalid") == 0)
  {
    for (i = 0U; i < 13U; ++i)
    {
      defaults();
      switch (i)
      {
        case 0: fake_state.gnss.meas_vn = NAN; break;
        case 1: fake_state.gnss.meas_ve = -3000000.0f; break;
        case 2: fake_state.out.vN = INFINITY; break;
        case 3: fake_state.out.vE = 3000000.0f; break;
        case 4: fake_state.gnss.vel_nis = NAN; break;
        case 5: fake_state.gnss.vel_r_var = 300000.0f; break;
        case 6: fake_state.out.Pvn = NAN; break;
        case 7: fake_state.out.Pvne = -INFINITY; break;
        case 8: fake_state.out.Pve = 300000.0f; break;
        case 9: fake_state.gnss.vel_nis = -1.0f; break;
        case 10: fake_state.gnss.vel_r_var = -1.0f; break;
        case 11: fake_state.out.Pvn = -1.0f; break;
        case 12: fake_state.out.Pve = -1.0f; break;
      }
      send_ok();
    }
  }
  else if (strcmp(argv[1], "errors") == 0) errors();
  else assert(0);
  return 0;
}
"""


if __name__ == "__main__":
    unittest.main()
