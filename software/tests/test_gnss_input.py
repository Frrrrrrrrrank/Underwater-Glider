"""实际 RDSS codec + UART IRQ/ring/RX task 的 GNSS 输入回归；产物仅在 D 盘。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

MAIN_STUB = r"""
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
#include <stddef.h>
#define __weak __attribute__((weak))
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; uint32_t ErrorCode; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
"""
PEERS_STUB = r"""
#include "main.h"
#define DISPLACEMENT_SENSOR_H
#define ETHERNET_UART_DRIVER_H
#define IMU_CORE_H
#define PRESSURE_SENSOR_DRIVER_H
#define SBUS_DRIVER_H
void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *);
void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *);
void SbusDriver_OnRxComplete(UART_HandleTypeDef *);
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *);
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *);
void DisplacementSensor_OnRxComplete(UART_HandleTypeDef *);
void IMU_OnUART_RxComplete(UART_HandleTypeDef *);
"""
HARNESS = r"""
#include "glider_core.h"
#include "glider_time.h"
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick = 1234U;
static UART_HandleTypeDef uart = {(void *)1, 0U};
static uint8_t *rx_byte;
static jmp_buf escape_rx;
static GnssSample samples[16];
static unsigned sample_count, tx_count, critical_depth;
static unsigned raw_count;
static uint64_t raw_time;
static char raw_line[512];
static uint32_t raw_src_len;
static uint32_t raw_uart_drop, raw_line_drop;
static uint16_t raw_kept_len;
static bool delay_decoded_callback;
void test_enter_critical(void) { assert(critical_depth == 0U); ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth == 1U); --critical_depth; }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ assert(h == &uart && n == 1U); rx_byte = p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t timeout)
{ assert(h == &uart && p && n && timeout == 200U); ++tx_count; tick += 200U; return HAL_OK; }
void osDelay(uint32_t ms) { assert(ms == 2U); longjmp(escape_rx, 1); }
void GliderCore_OnGnssSample(const GnssSample *sample)
{ assert(critical_depth == 0U && sample_count < 16U); samples[sample_count++] = *sample; }
void GliderCore_OnRawLine(const uint8_t *line, uint32_t src_len, uint16_t kept_len, uint64_t t_rx_us)
{
  assert(critical_depth == 0U && line && kept_len < sizeof(raw_line));
  memcpy(raw_line, line, kept_len); raw_line[kept_len] = '\0';
  raw_src_len = src_len; raw_kept_len = kept_len; raw_time = t_rx_us; ++raw_count;
  raw_uart_drop = GliderCore_GetRxOverflowCount();
  raw_line_drop = GliderCore_GetRxLineDropCount();
}
void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame)
{ (void)frame; if (delay_decoded_callback) { tick += 200U; } }
void PressureSensorDriver_OnRxComplete(UART_HandleTypeDef *h) { (void)h; }
void PressureSensorDriver_OnTxComplete(UART_HandleTypeDef *h) { (void)h; }
void SbusDriver_OnRxComplete(UART_HandleTypeDef *h) { (void)h; }
void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *h) { (void)h; }
void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *h) { (void)h; }
void DisplacementSensor_OnRxComplete(UART_HandleTypeDef *h) { (void)h; }
void IMU_OnUART_RxComplete(UART_HandleTypeDef *h) { (void)h; }

static void feed_line(const char *line)
{
  size_t i;
  for (i = 0U; line[i] != '\0'; ++i)
  { *rx_byte = (uint8_t)line[i]; HAL_UART_RxCpltCallback(&uart); }
  *rx_byte = '\n'; HAL_UART_RxCpltCallback(&uart);
  if (setjmp(escape_rx) == 0) { GliderCore_RunRxTask(); }
}
static void drain_rx(void)
{
  if (setjmp(escape_rx) == 0) { GliderCore_RunRxTask(); }
}
static void feed_bytes(const uint8_t *data, size_t length)
{
  /* 分段让 RX task 消费，单独验证组行器，不把 ring 溢出混入此用例。 */
  for (size_t i = 0U; i < length; ++i)
  {
    *rx_byte = data[i]; HAL_UART_RxCpltCallback(&uart);
    if ((i & 63U) == 63U) { drain_rx(); }
  }
  drain_rx();
}
static void feed_body(const char *body)
{
  char line[512]; unsigned checksum = 0U;
  for (size_t i = 0U; body[i] != '\0'; ++i) { checksum ^= (unsigned char)body[i]; }
  snprintf(line, sizeof(line), "$%s*%02X\r", body, checksum);
  feed_line(line);
}
int main(int argc, char **argv)
{
  assert(argc >= 2);
  if (strcmp(argv[1], "decode") == 0 || strcmp(argv[1], "decode_extended") == 0)
  {
    GliderRdssDecodedFrame frame;
    GnssSample sample, zero = {0};
    bool accepted;
    assert(argc == 3);
    memset(&sample, 0x5A, sizeof(sample));
    (void)GliderRdssCodec_TryDecodeLine(argv[2], &frame);
    accepted = GliderRdssCodec_TryMakeGnssSample(&frame, 123456789ULL, 42U, &sample);
    if (!accepted) { assert(memcmp(&sample, &zero, sizeof(sample)) == 0); }
    printf("%u %u %" PRIu64 " %u %u %" PRId32 " %" PRId32 " %u",
           accepted ? 1U : 0U, sample.seq, sample.t_rx_us, sample.utc_date_yyyymmdd,
           sample.utc_ms_of_day, sample.lat_deg_e7, sample.lon_deg_e7, sample.validity_flags);
    if (strcmp(argv[1], "decode_extended") == 0)
      printf(" %.9g %.9g %u %u %.9g %.9g", (double)sample.vel_n_mps,
             (double)sample.vel_e_mps, sample.quality_source, sample.velocity_source,
             (double)sample.position_var_m2, (double)sample.velocity_var_m2ps2);
    putchar('\n');
    assert(!GliderRdssCodec_TryMakeGnssSample(&frame, 0U, 0U, NULL));
    assert(!GliderRdssCodec_TryMakeGnssSample(NULL, 0U, 0U, &sample));
    return 0;
  }
  GliderCore_Init(&uart);
  if (strcmp(argv[1], "receive") == 0)
  {
    feed_body("GNRMC,235959.999,A,3015.0000,N,12030.0000,E,0,0,311226,,,A");
    assert(sample_count == 1U && samples[0].seq == 1U && samples[0].t_rx_us == 1234000ULL);
    /* 发布序号与测量历元分离：重复帧交给桥识别，不改写成新 UTC。 */
    tick += 10U;
    feed_body("GNRMC,235959.999,A,3015.0000,N,12030.0000,E,0,0,311226,,,A");
    assert(sample_count == 2U && samples[1].seq == 2U);
    assert(samples[0].utc_ms_of_day == samples[1].utc_ms_of_day);
    assert(samples[0].utc_date_yyyymmdd == samples[1].utc_date_yyyymmdd);
    tick += 10U;
    feed_body("GNRMC,000000.000,A,3015.0000,N,12030.0000,E,0,0,010127,,,A");
    assert(sample_count == 3U && samples[2].utc_ms_of_day == 0U);
    assert(samples[2].utc_date_yyyymmdd == 20270101U && samples[2].seq == 3U);
    feed_body("GNRMC,000001.000,V,3015.0000,N,12030.0000,E,0,0,010127,,,A");
    feed_line("$GNRMC,000002.000,A,3015.0000,N,12030.0000,E,0,0,010127,,,A");
    feed_line("$GNRMC,000002.000,A,3015.0000,N,12030.0000,E,0,0,010127,,,A*ZZ");
    assert(sample_count == 4U && GliderCore_GetGnssRejectedCount() == 2U);
    assert(samples[3].validity_flags == (GNSS_VALID_CHECKSUM | GNSS_VALID_UTC | GNSS_VALID_NO_FIX));
    assert(samples[3].seq == 4U && samples[3].lat_deg_e7 == 0 && samples[3].lon_deg_e7 == 0);
    assert(strstr(raw_line,"*ZZ") && raw_count == 6U); /* 坏帧仍能从原始行接口采集。 */
    feed_body("GNRMC,000002.000,A,3015.0000,N,12030.0000,E,10,90,010127,,,A");
    assert(sample_count == 5U && samples[4].seq == 5U); /* 坏校验没有占用发布序号。 */
    assert(samples[4].validity_flags == (GNSS_VALID_REQUIRED | GNSS_VALID_VELOCITY));
    assert(fabsf(samples[4].vel_n_mps) < .00001f && fabsf(samples[4].vel_e_mps - 5.144444f) < .00001f);
    assert(samples[4].velocity_source == GNSS_VELOCITY_RMC && samples[4].quality_source == GNSS_QUALITY_UNKNOWN);
    tick += 180000U;
    feed_line("SEND OK");
    feed_line("RNMODE!");
    assert(sample_count == 5U); /* 短信缓存/模式提示不重新发布位置。 */
    GliderCore_Init(&uart);
    assert(GliderCore_GetGnssRejectedCount() == 0U);
  }
  else if (strcmp(argv[1], "before_callback") == 0)
  {
    uint32_t before;
    feed_body("BDPWI,062500.00,0,1,5,40,42");
    feed_body("BDPWI,062501.00,0,1,5,40,42");
    assert(GliderCore_GetSignalGood() == 1U);
    before = tick;
    delay_decoded_callback = true;
    feed_body("GNRMC,062502.00,A,3015.0000,N,12030.0000,E,0,0,051126,,,A");
    assert(sample_count == 1U && samples[0].t_rx_us == (uint64_t)before * 1000ULL);
    assert(raw_count == 3U && raw_time == samples[0].t_rx_us && strstr(raw_line,"$GNRMC,") == raw_line);
    assert(tx_count == 0U && tick == before + 200U); /* 不把下游回调耗时计入样本时刻；RX不发送。 */
  }
  else if (strcmp(argv[1], "raw_only") == 0)
  {
    feed_body("GNGGA,062500.00,,,,,0,00,99.9,,,,,,");
    feed_body("GNGST,062500.00,1,1,1,0,1,1,1");
    feed_line("$GNGGA,062501.00*ZZ");
    assert(raw_count == 3U && sample_count == 0U && tx_count == 0U);
    assert(GliderCore_GetGnssRejectedCount() == 0U && strstr(raw_line,"*ZZ"));
  }
  else if (strcmp(argv[1], "raw_binary") == 0)
  {
    const uint8_t bytes[] = {'$', 'X', ',', 0U, 0xffU, '*', 'Z', 'Z', '\r', '\n'};
    feed_bytes(bytes, sizeof(bytes));
    assert(raw_count == 1U && raw_src_len == sizeof(bytes) - 1U && raw_kept_len == raw_src_len);
    assert(memcmp(raw_line, bytes, raw_kept_len) == 0 && raw_time == 1234000ULL);
    assert(raw_uart_drop == 0U && raw_line_drop == 0U && sample_count == 0U && tx_count == 0U);
    feed_bytes((const uint8_t *)"\n", 1U);
    assert(raw_count == 2U && raw_src_len == 0U && raw_kept_len == 0U);
  }
  else if (strcmp(argv[1], "raw_oversize") == 0)
  {
    uint8_t bytes[602];
    memset(bytes, 'X', 600U); bytes[600] = '\r'; bytes[601] = '\n';
    feed_bytes(bytes, sizeof(bytes));
    assert(raw_count == 1U && raw_src_len == 601U && raw_kept_len == 384U);
    assert(memcmp(raw_line, bytes, 384U) == 0 && raw_line_drop == 1U && raw_uart_drop == 0U);
    assert(sample_count == 0U && GliderCore_GetRxLineDropCount() == 1U);
    feed_body("GNRMC,062500.00,A,3015.0000,N,12030.0000,E,0,0,051126,,,A");
    assert(raw_count == 2U && sample_count == 1U && raw_line_drop == 1U);
    GliderCore_Init(&uart);
    assert(GliderCore_GetRxLineDropCount() == 0U && GliderCore_GetRxOverflowCount() == 0U);
  }
  else if (strcmp(argv[1], "raw_ring_loss") == 0)
  {
    /* ring 容量 512、留空一槽；故意延迟 task，验证字节丢失与超长行计数分开。 */
    for (unsigned i = 0U; i < 600U; ++i)
    { *rx_byte = 'X'; HAL_UART_RxCpltCallback(&uart); }
    *rx_byte = '\n'; HAL_UART_RxCpltCallback(&uart);
    assert(GliderCore_GetRxOverflowCount() == 90U);
    drain_rx();
    assert(raw_count == 0U && GliderCore_GetRxLineDropCount() == 0U);
    feed_bytes((const uint8_t *)"\n", 1U);
    assert(raw_count == 1U && raw_src_len == 511U && raw_kept_len == 384U);
    assert(raw_uart_drop == 90U && raw_line_drop == 0U);
    feed_body("GNRMC,062500.00,A,3015.0000,N,12030.0000,E,0,0,051126,,,A");
    assert(sample_count == 1U && raw_uart_drop == 90U && tx_count == 0U);
  }
  else { assert(0); }
  assert(critical_depth == 0U);
  return 0;
}
"""


def sentence(fields):
    body = ",".join(fields)
    checksum = 0
    for value in body.encode("ascii"):
        checksum ^= value
    return f"${body}*{checksum:02X}"


class GnssInputHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("需要主机 gcc 执行真实 GNSS 解析与接收任务")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="gnss_input_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "main.h").write_text(MAIN_STUB, encoding="utf-8")
        (work / "peers.h").write_text(PEERS_STUB, encoding="utf-8")
        (work / "cmsis_os.h").write_text("#include <stdint.h>\nvoid osDelay(uint32_t);\n", encoding="utf-8")
        (work / "FreeRTOS.h").write_text("/* 主机不启动调度器。 */\n", encoding="utf-8")
        (work / "task.h").write_text(
            "void test_enter_critical(void);\nvoid test_exit_critical(void);\n"
            "#define taskENTER_CRITICAL() test_enter_critical()\n"
            "#define taskEXIT_CRITICAL() test_exit_critical()\n", encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("gnss_input.exe" if os.name == "nt" else "gnss_input")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-include", str(work / "peers.h"),
             "-I", str(work), "-I", str(ROOT / "software/Glider-Core"),
             "-I", str(ROOT / "software/Glider-App/common"),
             str(ROOT / "software/Glider-Core/glider_rdss_codec.c"),
             str(ROOT / "software/Glider-Core/glider_core.c"),
             str(ROOT / "software/Glider-App/common/glider_time.c"), str(work / "harness.c"),
             "-lm", "-o", str(cls.binary)], capture_output=True, text=True,
            encoding="utf-8", errors="replace", env=cls.env)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, *args):
        result = subprocess.run([str(self.binary), *args], capture_output=True, text=True,
                                encoding="utf-8", errors="replace", env=self.env)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout.strip()

    def decode(self, fields=None, **changes):
        fields = list(fields or ["GNRMC", "062500.00", "A", "3015.0000", "N",
                                 "12030.0000", "E", "0.0", "0.0", "051126", "", "", "A"])
        for key, value in changes.items():
            fields[int(key)] = value
        return [int(value) for value in self.run_case("decode", sentence(fields)).split()]

    def test_valid_coordinate_precision_hemispheres_and_legacy_mode(self):
        self.assertEqual(self.decode(), [1, 42, 123456789, 20261105, 23100000, 302500000, 1205000000, 31])
        self.assertEqual(self.decode(**{"3": "3015.123456789", "4": "S", "6": "W"})[5:7],
                         [-302520576, -1205000000])
        for mode in ("", "A", "D", "F", "R", "P"):
            with self.subTest(mode=mode):
                self.assertEqual(self.decode(**{"12": mode})[0], 1)
        legacy = ["GNRMC", "062500", "A", "3015", "N", "12030", "E", "0", "0", "051126"]
        self.assertEqual(self.decode(legacy)[0], 1)

    def test_missing_bad_and_malformed_checksum_never_make_sample(self):
        line = sentence(["GNRMC", "062500.00", "A", "3015.0000", "N", "12030.0000", "E", "0", "0", "051126", "", "", "A"])
        for invalid in (line.split("*")[0], line[:-2] + "00", line[:-2] + "GG", line + "X", line[:-1]):
            with self.subTest(line=invalid):
                self.assertEqual(int(self.run_case("decode", invalid).split()[0]), 0)

    def test_invalid_status_modes_navigation_and_unknown_talker(self):
        for key, values in (("2", ("AA", "a", "")), ("12", ("E", "M", "S", "X", "AA")),
                            ("0", ("GPRMC", "GNGGA"))):
            for value in values:
                with self.subTest(key=key, value=value):
                    self.assertEqual(self.decode(**{key: value})[0], 0)
        fields = ["GNRMC", "062500.00", "A", "3015.0000", "N", "12030.0000", "E", "0", "0", "051126", "", "", "A", "S"]
        self.assertEqual(self.decode(fields)[0], 1)
        for status in ("C", "U", "SS"):
            self.assertEqual(self.decode(fields, **{"13": status})[0], 0)
        self.assertEqual(self.decode(fields + ["extra"])[0], 0)

    def test_utc_and_calendar_validation(self):
        for utc in ("", "240000", "126000", "120060", "062500.", "62500", "062500.0001", "06a500", "-62500"):
            with self.subTest(utc=utc):
                self.assertEqual(self.decode(**{"1": utc})[0], 0)
        for date in ("", "000126", "320126", "310426", "290226", "011326", "010026", "05112x", "05112026"):
            with self.subTest(date=date):
                self.assertEqual(self.decode(**{"9": date})[0], 0)
        self.assertEqual(self.decode(**{"9": "290224"})[3], 20240229)
        self.assertEqual(self.decode(**{"9": "290200"})[3], 20000229)
        self.assertEqual(self.decode(**{"1": "235959.999", "9": "311226"})[3:5], [20261231, 86399999])
        self.assertEqual(self.decode(**{"1": "000000.0", "9": "010127"})[3:5], [20270101, 0])

    def test_coordinate_format_range_and_direction(self):
        for key, values in (("3", ("", "315.0", "03015.0", "9060.0", "9000.00001", "3060.0", "30a5.0", "3015.", "3015.1234567890")),
                            ("5", ("1203.0", "18100.0", "18000.1", "12060.0", "99999999999999999")),
                            ("4", ("E", "NN", "n", "")), ("6", ("N", "EE", "e", ""))):
            for value in values:
                with self.subTest(key=key, value=value):
                    self.assertEqual(self.decode(**{key: value})[0], 0)
        self.assertEqual(self.decode(**{"3": "9000.0", "5": "18000.0"})[5:7], [900000000, 1800000000])

    def test_real_uart_task_keeps_duplicate_epoch_and_midnight_identity(self):
        self.run_case("receive")

    def test_receive_timestamp_precedes_downstream_callback_delay(self):
        self.run_case("before_callback")

    def test_no_fix_events_require_checksum_and_utc_but_not_coordinates(self):
        fields = ["GNRMC", "235959.999", "V", "", "", "", "", "", "", "311226", "", "", "N"]
        values = self.decode(fields)
        self.assertEqual(values, [1, 42, 123456789, 20261231, 86399999, 0, 0, 73])
        self.assertEqual(self.decode(**{"12": "N"})[7], 73)
        self.assertEqual(self.decode(fields[:-1] + ["E"])[7], 73)
        active = ["GNRMC", "000000.000", "A", "3015.0", "N", "12030.0", "E", "10", "90", "010127", "", "", "A", "V"]
        self.assertEqual(self.decode(active)[3:8], [20270101, 0, 0, 0, 73])
        for utc, date in (("240000", "311226"), ("235959", "311126")):
            self.assertEqual(self.decode(fields, **{"1": utc, "9": date})[0], 0)
        raw = sentence(fields)
        for bad in (raw.split("*")[0], raw[:-2] + "ZZ"):
            self.assertEqual(int(self.run_case("decode", bad).split()[0]), 0)
        extra = self.run_case("decode_extended", raw).split()[8:]
        self.assertEqual([float(value) for value in extra], [0.0] * 6)

    def test_rmc_sog_cog_convert_to_north_east_without_claiming_accuracy(self):
        fields = ["GNRMC", "062500.00", "A", "3015.0000", "N", "12030.0000", "E", "10", "0", "051126", "", "", "A"]
        speed = 10 * 1852 / 3600
        for course, north, east in (("0", speed, 0), ("90", 0, speed),
                                    ("180", -speed, 0), ("270", 0, -speed)):
            with self.subTest(course=course):
                fields[8] = course
                values = self.run_case("decode_extended", sentence(fields)).split()
                self.assertEqual(int(values[7]), 63)
                self.assertAlmostEqual(float(values[8]), north, places=5)
                self.assertAlmostEqual(float(values[9]), east, places=5)
                self.assertEqual([int(value) for value in values[10:12]], [0, 1])
                self.assertEqual([float(value) for value in values[12:]], [0.0, 0.0])

    def test_missing_invalid_or_low_speed_never_discards_valid_position(self):
        fields = ["GNRMC", "062500.00", "A", "3015.0000", "N", "12030.0000", "E", "10", "90", "051126", "", "", "A"]
        pairs = [(bad, "90") for bad in ("", "0", "0.5", "0.5831", "-1", "+1", "NaN", "Inf", "1e2", "10.", "1.2.3", "9999999999")]
        pairs += [("10", bad) for bad in ("", "-1", "360", "361", "NaN", "Inf", "1e2", "90.")]
        for speed, course in pairs:
            with self.subTest(speed=speed, course=course):
                fields[7:9] = [speed, course]
                values = self.run_case("decode_extended", sentence(fields)).split()
                self.assertEqual([int(value) for value in values[5:8]], [302500000, 1205000000, 31])
                self.assertEqual([float(value) for value in values[8:]], [0.0] * 6)
        fields[7:9] = ["0.5832", "90"]
        self.assertEqual(int(self.run_case("decode_extended", sentence(fields)).split()[7]), 63)

    def test_raw_hook_records_unsupported_and_bad_sentences_without_fusion(self):
        self.run_case("raw_only")

    def test_raw_hook_preserves_binary_cr_and_empty_line(self):
        self.run_case("raw_binary")

    def test_oversize_decoder_line_still_captures_prefix_and_resynchronizes(self):
        self.run_case("raw_oversize")

    def test_uart_ring_lost_bytes_have_separate_counter(self):
        self.run_case("raw_ring_loss")


if __name__ == "__main__":
    unittest.main()
