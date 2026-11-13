"""真实 GNRMC codec→导航状态机→FP32 ESKF 的无 ROS 集成回放。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

HARNESS = r"""
extern "C" {
#include "glider_rdss_codec.h"
}
#include "gnss_navigation.h"
#include "ug_estimation/core/types.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>

static_assert(sizeof(ug_ekf::Scalar) == sizeof(float), "必须验证板端 FP32 配置");
static GnssNavigation nav;
static UgEkfOutput output;
static uint64_t now_us = 1000000ULL;
static uint32_t seq;

static void read_output()
{
  UgEkf_GetOutput(&output);
  assert(output.finite && std::isfinite(output.pN) && std::isfinite(output.pE));
}

static void advance_to(uint64_t target)
{
  assert(target >= now_us && (target - now_us) % 20000ULL == 0U);
  while (now_us < target) {
    now_us += 20000ULL;
    /* 本夹具直接使用 FRD；静止 IMU 的比力向上，z 为 -g。 */
    UgEkf_PushImu(0,0,0,0,0,-9.81f,now_us);
    read_output();
    GnssNavigation_Step(&nav,nullptr,now_us,now_us,&output);
  }
}

static void feed_rmc(unsigned utc_second, const char *latitude = "3015.0000",
                     const char *speed = "", const char *course = "",
                     char status = 'A', const char *date = "061126", uint64_t rx_us = 0ULL)
{
  char body[180], line[200];
  unsigned checksum = 0U;
  GliderRdssDecodedFrame frame;
  GnssSample sample;
  assert(utc_second < 86400U);
  if (rx_us == 0ULL) rx_us = now_us;
  std::snprintf(body,sizeof(body),
      "GNRMC,%02u%02u%02u.000,%c,%s,N,12030.0000,E,%s,%s,%s,,,A",
      utc_second/3600U,utc_second/60U%60U,utc_second%60U,status,
      latitude,speed,course,date);
  for (const char *p = body; *p; ++p) checksum ^= static_cast<unsigned char>(*p);
  std::snprintf(line,sizeof(line),"$%s*%02X",body,checksum);
  assert(GliderRdssCodec_TryDecodeLine(line,&frame));
  assert(GliderRdssCodec_TryMakeGnssSample(&frame,rx_us,++seq,&sample));
  assert(sample.utc_ms_of_day == utc_second * 1000U && sample.t_rx_us == rx_us);
  if (status == 'V') assert(sample.validity_flags & GNSS_VALID_NO_FIX);
  else assert((sample.validity_flags & GNSS_VALID_REQUIRED) == GNSS_VALID_REQUIRED);
  read_output();
  GnssNavigation_Step(&nav,&sample,now_us,now_us,&output);
  read_output();
}

static void origin_unchanged()
{
  assert(nav.diag.origin_id == 1U);
  assert(nav.diag.origin_lat_e7 == 302500000 && nav.diag.origin_lon_e7 == 1205000000);
}

int main()
{
  UgEkfConfig config;
  UgEkf_DefaultConfig(&config);
  config.static_align_seconds = .1f;
  config.static_align_delay_seconds = .01f;
  UgEkf_Init(&config);
  GnssNavigation_Init(&nav,nullptr);
  UgEkf_PushImu(0,0,0,0,0,-9.81f,now_us);
  advance_to(2000000ULL);
  assert(output.status == UG_EKF_RUNNING && output.valid_for_control);
  assert(output.accept_gps == 0U && output.gps_reset_count == 0U);
  assert(nav.diag.origin_id == 0U && !(nav.diag.flags & GNSS_FLAG_AIDING_VALID));

  /* 第一个合格位置立即锚定；缺 RMC 速度只建立一次宽方差先验。 */
  advance_to(3000000ULL); feed_rmc(3U);
  origin_unchanged();
  assert(output.gps_reset_count == 1U && output.accept_gps == 0U);
  assert(output.horizontal_velocity_reset_count == 1U);
  assert(output.horizontal_velocity_source == UG_EKF_VELOCITY_UNKNOWN_PRIOR);
  assert(std::fabs(output.vN) < .001f && std::fabs(output.vE) < .001f);
  assert(output.Pvn >= 25.0f && output.Pve >= 25.0f);
  assert(nav.diag.state == GNSS_NAV_POSITION_ONLY && nav.diag.vel_accept_count == 0U);
  assert(!(nav.diag.flags & GNSS_FLAG_VELOCITY_VALID));

  /* 间隔四秒和五秒的真实位置矩阵更新，通过 Ppv 学到向北约 1 m/s。
   * 不构造差分速度测量，也不反复 reset 破坏刚学到的速度。 */
  advance_to(7000000ULL); feed_rmc(7U,"3015.0022");
  advance_to(12000000ULL); feed_rmc(12U,"3015.0049");
  origin_unchanged();
  assert(output.gps_reset_count == 1U && output.accept_gps == 2U);
  assert(output.accept_gps_velocity == 0U && nav.diag.vel_accept_count == 0U);
  assert(output.vN > .1f && output.vN < 2.0f && std::fabs(output.vE) < .05f);
  assert(nav.diag.velocity_source == 2U && nav.diag.state == GNSS_NAV_TRACKING);
  assert(nav.diag.flags & GNSS_FLAG_AIDING_VALID);

  /* 同一次串口处理时刻，两个 UTC 不同的历元仍各融合一次；重发同 UTC 不融合。 */
  const uint64_t burst_rx = now_us;
  feed_rmc(13U,"3015.0049");
  assert(now_us == burst_rx && output.accept_gps == 3U && output.gps_reset_count == 1U);
  const uint64_t last_fuse = nav.last_fuse_us;
  advance_to(12100000ULL); feed_rmc(13U,"3015.0049");
  assert(nav.diag.reason == GNSS_REASON_DUPLICATE && nav.diag.duplicate_count == 1U);
  assert(nav.last_fuse_us == last_fuse && output.accept_gps == 3U);

  /* 有效校验 V 句是状态事件，不把无定位坐标送进滤波器。 */
  advance_to(13000000ULL); feed_rmc(14U,"3015.0049","","",'V');
  assert(nav.diag.no_fix_count == 1U && nav.diag.reason == GNSS_REASON_NO_FIX);
  assert(nav.last_fuse_us == last_fuse && output.accept_gps == 3U);
  assert(!(nav.diag.flags & GNSS_FLAG_MEASUREMENT));

  /* 五分钟无绝对定位：只做真实 IMU 预测，保留先前速度会造成明显位置漂移。 */
  advance_to(313000000ULL);
  assert(nav.diag.state == GNSS_NAV_DR_UNTRUSTED);
  assert(!(nav.diag.flags & GNSS_FLAG_AIDING_VALID));
  assert(output.accept_gps == 3U && output.gps_reset_count == 1U);
  const float drift_n = output.pN;
  feed_rmc(313U,"3015.0054");
  origin_unchanged();
  assert(output.gps_reset_count == 2U && output.accept_gps == 3U);
  assert(std::fabs(output.pN-nav.diag.meas_n) < .002f);
  assert(std::fabs(output.vN) < .001f && std::fabs(output.vE) < .001f);
  assert(output.Pvn >= 25.0f && output.Pve >= 25.0f);
  assert(nav.diag.recovery_reason == GNSS_RECOVERY_AID_LOSS);
  assert(std::fabs(nav.diag.reset_before_n-drift_n) < .01f);
  assert(nav.diag.vel_accept_count == 0U && nav.diag.vel_fuse_age_ms == UINT32_MAX);

  /* 下一次长间断带 RMC 真正速度：p/v 原子恢复为向东 2 节，不受旧漂移 NIS 卡住。 */
  advance_to(344000000ULL); feed_rmc(344U,"3015.0054","2.0","90.0");
  origin_unchanged();
  assert(output.gps_reset_count == 3U && output.accept_gps == 3U);
  assert(output.horizontal_velocity_source == UG_EKF_VELOCITY_GNSS_MEASURED);
  assert(std::fabs(output.vN) < .001f && std::fabs(output.vE-1.0288889f) < .001f);
  assert(nav.diag.vel_accept_count == 1U && output.accept_gps_velocity == 0U);
  advance_to(345000000ULL); feed_rmc(345U,"3015.0054","2.0","90.0");
  assert(output.accept_gps == 4U && output.accept_gps_velocity == 1U);
  assert(nav.diag.vel_accept_count == 2U && (nav.diag.flags & GNSS_FLAG_VELOCITY_VALID));

  /* 磁标定重启保持原点/UTC 水位；真实重新对齐后，旧 UTC 仍拒绝，新帧立即重锚。 */
  const uint64_t last_epoch = nav.last_epoch_ms;
  UgEkf_Init(&config); GnssNavigation_OnFilterReset(&nav,now_us);
  origin_unchanged(); advance_to(347000000ULL);
  assert(output.status == UG_EKF_RUNNING && output.valid_for_control);
  feed_rmc(345U,"3015.0054");
  assert(nav.last_epoch_ms == last_epoch && nav.diag.reason == GNSS_REASON_DUPLICATE);
  assert(!nav.anchored && output.accept_gps == 0U && output.gps_reset_count == 0U);
  advance_to(348000000ULL); feed_rmc(348U,"3015.0054");
  origin_unchanged();
  assert(nav.anchored && output.gps_reset_count == 1U && nav.diag.reset_count == 4U);
  assert(nav.diag.recovery_reason == GNSS_RECOVERY_FILTER_RESET);
  assert(output.accept_gps == 0U && !(nav.diag.flags & GNSS_FLAG_AIDING_VALID));

  /* 日期翻转，接收时刻可以相同；完整日期+UTC 键避免把午夜误判为乱序。 */
  advance_to(349000000ULL); feed_rmc(86399U,"3015.0054","","",'A',"311226");
  feed_rmc(0U,"3015.0054","","",'A',"010127");
  assert(nav.diag.utc_date == 20270101U && nav.diag.utc_ms == 0U);
  assert(output.accept_gps == 2U && output.gps_reset_count == 1U);

  /* 先保留一条已融合的速度证据，再模拟长失锁后积压 500ms 的合法 RMC。
   * 最新 IMU 新鲜，但不能用它冒充旧接收时刻的 gyro 做杆臂速度补偿。
   * 实际 wrapper 拒绝测量速度重锚，导航同历元降级未知速度；位置仍成功恢复。 */
  advance_to(350000000ULL);
  feed_rmc(1U,"3015.0054","2.0","90.0",'A',"010127");
  const uint64_t velocity_fuse_before = nav.last_velocity_us;
  const uint32_t velocity_accept_before = nav.diag.vel_accept_count;
  const uint32_t velocity_reject_before = nav.diag.vel_reject_count;
  assert(velocity_fuse_before == now_us && nav.have_velocity);
  advance_to(381000000ULL);
  feed_rmc(32U,"3015.0054","2.0","90.0",'A',"010127",380500000ULL);
  origin_unchanged();
  assert(nav.anchored && output.gps_reset_count == 2U);
  assert(std::fabs(output.pN-nav.diag.meas_n) < .002f);
  assert(std::fabs(output.vN) < .001f && std::fabs(output.vE) < .001f);
  assert(output.Pvn >= 25.0f && output.Pve >= 25.0f);
  assert(output.horizontal_velocity_source == UG_EKF_VELOCITY_UNKNOWN_PRIOR);
  assert(nav.diag.vel_reject_count == velocity_reject_before + 1U);
  assert(nav.diag.vel_accept_count == velocity_accept_before);
  assert(nav.last_velocity_us == velocity_fuse_before && nav.diag.vel_fuse_age_ms == 31000U);
  assert(nav.last_fuse_us == 380500000ULL && nav.diag.fuse_age_ms == 500U);
  assert(nav.diag.flags & GNSS_FLAG_VELOCITY_MEASUREMENT);
  assert(std::fabs(nav.diag.meas_ve-1.0288889f) < .001f);
  assert(!(nav.diag.flags & GNSS_FLAG_VELOCITY_VALID) && nav.diag.velocity_source == 0U);

  /* 定向覆盖 5..30s 失锁的另一条通路：先普通位置 NIS 拒绝，再恢复。
   * 这里故意收紧门限稳定触发拒绝，仍使用实际位置模型/矩阵/返回码。
   * measured 重锚因旧 gyro 暂不可用，不能抹掉该历元已获准的未知速度重试资格。 */
  config.nis_gate_gps = 1.0e-6f;
  UgEkf_Init(&config); GnssNavigation_OnFilterReset(&nav,now_us);
  advance_to(384000000ULL);
  feed_rmc(35U,"3015.0054","2.0","90.0",'A',"010127");
  assert(output.gps_reset_count == 1U && output.accept_gps == 0U);
  const uint32_t prior_rejects = nav.diag.reject_count;
  const uint32_t prior_velocity_rejects = nav.diag.vel_reject_count;
  const uint32_t prior_velocity_accepts = nav.diag.vel_accept_count;
  advance_to(390000000ULL);
  assert(nav.diag.fuse_age_ms == 6000U && nav.diag.state == GNSS_NAV_REACQUIRING);
  feed_rmc(41U,"3015.0054","2.0","90.0",'A',"010127",389500000ULL);
  assert(output.reject_gps == 1U && output.accept_gps == 0U);
  assert(output.gps_reset_count == 2U && nav.diag.reason == GNSS_REASON_RESET);
  assert(nav.diag.recovery_reason == GNSS_RECOVERY_REJECT);
  assert(nav.diag.reject_count == prior_rejects + 1U);
  assert(nav.diag.vel_reject_count == prior_velocity_rejects + 1U);
  assert(nav.diag.vel_accept_count == prior_velocity_accepts);
  assert(nav.last_velocity_us == 384000000ULL && nav.diag.vel_fuse_age_ms == 6000U);
  assert(nav.last_fuse_us == 389500000ULL && nav.diag.fuse_age_ms == 500U);
  assert(std::fabs(output.pN-nav.diag.meas_n) < .002f);
  assert(std::fabs(output.vN) < .001f && std::fabs(output.vE) < .001f);
  assert(output.Pvn >= 25.0f && output.Pve >= 25.0f);
  const uint32_t recovered_resets = output.gps_reset_count;
  feed_rmc(41U,"3015.0054","2.0","90.0",'A',"010127");
  assert(nav.diag.reason == GNSS_REASON_DUPLICATE && output.gps_reset_count == recovered_resets);
  assert(output.reject_gps == 1U && output.accept_gps == 0U);
  std::puts("PASS actual GNRMC -> navigation -> FP32 ESKF: sparse velocity learning, V, burst epochs, p/v recovery, midnight, reset");
  return 0;
}
"""


class GnssPipelineHostTests(unittest.TestCase):
    def test_actual_codec_navigation_and_fp32_ekf_replay(self):
        cxx = shutil.which("g++")
        if cxx is None and Path("D:/msys64/mingw64/bin/g++.exe").is_file():
            cxx = "D:/msys64/mingw64/bin/g++.exe"
        if cxx is None:
            self.skipTest("需要主机 GCC/G++ 执行真实 FP32 GNSS 全链回放")
        cc = Path(cxx).with_name("gcc.exe" if os.name == "nt" else "gcc")
        self.assertTrue(cc.is_file(), "GCC 与 G++ 必须来自同一工具链")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="gnss_pipeline_", dir=tmp_root) as tmp:
            work = Path(tmp)
            env = dict(os.environ)
            env["PATH"] = str(Path(cxx).parent) + os.pathsep + env.get("PATH", "")
            env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
            includes = []
            for directory in ("Glider-Core", "Glider-App/common", "Glider-App/estimation",
                              "Glider-App/estimation/ug_estimation/include",
                              "Middlewares/Third_Party/Eigen"):
                includes += ["-I", str(ROOT / "software" / directory)]
            fixture = work / "pipeline.cpp"
            fixture.write_text(HARNESS, encoding="utf-8")

            def run(command, timeout=180):
                result = subprocess.run(command, env=env, capture_output=True, text=True,
                                        encoding="utf-8", errors="replace", timeout=timeout)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                return result.stdout

            objects = []
            for relative in ("Glider-Core/glider_rdss_codec.c", "Glider-App/estimation/gnss_navigation.c"):
                source = ROOT / "software" / relative
                obj = work / (source.stem + ".o")
                run([str(cc), "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", *includes,
                     "-c", str(source), "-o", str(obj)])
                objects.append(str(obj))
            binary = work / ("gnss_pipeline.exe" if os.name == "nt" else "gnss_pipeline")
            core = ROOT / "software/Glider-App/estimation/ug_estimation/src"
            run([cxx, "-std=gnu++14", "-O1", "-fno-exceptions", "-fno-rtti", "-DEIGEN_NO_MALLOC",
                 "-DEIGEN_DONT_VECTORIZE", "-DUG_EKF_USE_FP32", *includes, str(fixture),
                 str(core / "core/eskf.cpp"), str(core / "embedded/ug_ekf_glider.cpp"),
                 *objects, "-o", str(binary)])
            self.assertIn("PASS actual GNRMC", run([str(binary)], timeout=90))


if __name__ == "__main__":
    unittest.main()
