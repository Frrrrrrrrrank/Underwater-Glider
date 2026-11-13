"""执行真实 GNSS 导航状态机与 EKF 桥；算法矩阵和 HAL/RTOS 用可观测桩隔离。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

TASK_STUB = r"""
#include <stdint.h>
extern unsigned critical_depth, critical_entries;
#define taskENTER_CRITICAL() (++critical_depth, ++critical_entries)
#define taskEXIT_CRITICAL() (--critical_depth)
"""
PRESSURE_STUB = r"""
#include <stdint.h>
typedef struct {
  uint32_t pressure_valid, pressure_update_seq, pressure_update_tick;
  float pressure_bar;
} TestPressure;
extern TestPressure g_pressure_sensor_readout;
"""
HARNESS = r"""
#include "ekf_bridge.h"
#include "gnss_navigation.h"
#include "glider_time.h"
#include "pressure_sensor_driver.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

unsigned critical_depth, critical_entries;
TestPressure g_pressure_sensor_readout;
static UgEkfOutput filter;
static uint64_t now_us;
static unsigned gps_calls, reset_calls, velocity_calls, init_calls, imu_calls;
static int gps_result = UG_EKF_GPS_ACCEPTED;
static int reset_result = UG_EKF_GPS_ACCEPTED;
static int velocity_result = UG_EKF_GPS_ACCEPTED;
static uint32_t input_rejects;
static float last_n, last_e;
static float last_vn, last_ve, last_vvar;
static uint32_t last_velocity_measured;

uint64_t GliderTime_NowUs(void) { return now_us; }
uint32_t GliderTime_NowMs(void) { return (uint32_t)(now_us / 1000ULL); }
uint32_t GliderCore_GetGnssRejectedCount(void) { return input_rejects; }
static void running(void)
{
  memset(&filter, 0, sizeof(filter));
  filter.status = UG_EKF_RUNNING;
  filter.aligned = filter.healthy = filter.finite = filter.valid_for_control = 1U;
  filter.Pnn = filter.Pee = 25.0f;
  filter.Pvn = filter.Pve = 1.0f;
  filter.horizontal_recovery_ready = 1U;
}
void UgEkf_DefaultConfig(UgEkfConfig *cfg) { memset(cfg, 0, sizeof(*cfg)); }
void UgEkf_Init(const UgEkfConfig *cfg)
{
  assert(cfg != NULL && critical_depth == 0U); ++init_calls;
  running(); filter.status = UG_EKF_ALIGNING; filter.aligned = 0U;
  filter.horizontal_recovery_ready = 0U;
}
void UgEkf_GetOutput(UgEkfOutput *out)
{ assert(critical_depth == 0U); *out = filter; }
void UgEkf_PushImu(float gx, float gy, float gz, float ax, float ay, float az, uint64_t t)
{
  (void)gx; (void)gy; (void)gz; (void)ax; (void)ay; (void)az; (void)t;
  assert(critical_depth == 0U); ++imu_calls;
  filter.status = UG_EKF_RUNNING; filter.aligned = 1U;
  filter.horizontal_recovery_ready = 1U;
}
void UgEkf_PushMag(float x, float y, float z, uint64_t t)
{ (void)x; (void)y; (void)z; (void)t; assert(critical_depth == 0U); }
void UgEkf_PushDepth(float d, uint64_t t)
{ (void)d; (void)t; assert(critical_depth == 0U); }
static int position_result(float n, float e, float vn, float cov, float ve, int result)
{
  assert(critical_depth == 0U && isfinite(vn) && vn > 0 && ve == vn && cov == 0.0f);
  last_n = n; last_e = e;
  filter.gps_innov_n = n - filter.pN; filter.gps_innov_e = e - filter.pE;
  filter.nis_gps = result == UG_EKF_GPS_REJECTED ? 100.0f : 0.01f;
  if (result == UG_EKF_GPS_ACCEPTED) {
    filter.pN = n; filter.pE = e; filter.Pnn = vn; filter.Pee = ve; filter.Pne = 0;
  }
  return result;
}
int UgEkf_PushGpsPosition(float n, float e, float vn, float cov, float ve, uint64_t epoch, uint64_t t)
{
  assert(t > 0U && epoch > 0U); ++gps_calls;
  if (gps_result == UG_EKF_GPS_ACCEPTED) ++filter.accept_gps;
  else ++filter.reject_gps;
  return position_result(n,e,vn,cov,ve,gps_result);
}
int UgEkf_ResetHorizontalPosition(float n, float e, float vn, float cov, float ve, uint64_t epoch, uint64_t t)
{ (void)epoch;(void)n;(void)e;(void)vn;(void)cov;(void)ve;(void)t; assert(0); return UG_EKF_GPS_INVALID; }
int UgEkf_PushGpsVelocity(float vn, float ve, float rn, float cov, float re, uint64_t epoch, uint64_t t)
{
  assert(epoch > 0U);
  assert(critical_depth == 0U && t > 0 && rn > 0 && rn == re && cov == 0);
  ++velocity_calls; last_vn = vn; last_ve = ve; last_vvar = rn;
  filter.nis_gps_velocity = velocity_result == UG_EKF_GPS_REJECTED ? 100.0f : .01f;
  if (velocity_result == UG_EKF_GPS_ACCEPTED) {
    filter.vN = vn; filter.vE = ve; filter.Pvn = rn; filter.Pve = re; filter.Pvne = 0;
    ++filter.accept_gps_velocity;
  } else ++filter.reject_gps_velocity;
  return velocity_result;
}
int UgEkf_ReanchorHorizontal(float n, float e, float rn, float cov, float re,
                            float vn, float ve, float rvn, float vcov, float rve,
                            uint32_t measured, uint64_t epoch, uint64_t t)
{
  assert(epoch > 0U);
  assert(critical_depth == 0U && t > 0 && rvn > 0 && rvn == rve && vcov == 0);
  ++reset_calls; last_velocity_measured = measured;
  last_vn = vn; last_ve = ve; last_vvar = rvn;
  if (reset_result == UG_EKF_GPS_ACCEPTED) {
    filter.vN = measured ? vn : 0; filter.vE = measured ? ve : 0;
    filter.Pvn = rvn; filter.Pve = rve; filter.Pvne = 0;
    filter.finite = filter.healthy = filter.valid_for_control = 1U;
    filter.status = UG_EKF_RUNNING;
    filter.horizontal_velocity_source = measured ? UG_EKF_VELOCITY_GNSS_MEASURED : UG_EKF_VELOCITY_UNKNOWN_PRIOR;
    ++filter.gps_reset_count; ++filter.horizontal_velocity_reset_count;
  }
  return position_result(n,e,rn,cov,re,reset_result);
}

static GnssSample sample(uint32_t seq, uint64_t t)
{
  GnssSample s = {0};
  s.seq = seq; s.t_rx_us = t; s.utc_date_yyyymmdd = 20261105U;
  s.utc_ms_of_day = (uint32_t)(t / 1000ULL);
  s.lat_deg_e7 = 302500000; s.lon_deg_e7 = 1205000000;
  s.validity_flags = GNSS_VALID_REQUIRED;
  return s;
}
/* 和真实桥一样，传入调用前快照，避免桩修改 filter 后覆盖 reset-before 证据。 */
static void nav_step(GnssNavigation *n, const GnssSample *s, uint64_t now, uint64_t imu)
{
  UgEkfOutput before = filter;
  GnssNavigation_Step(n,s,now,imu,&before);
}
static void nav_feed(GnssNavigation *n, uint32_t seconds)
{
  GnssSample s = sample(seconds, (uint64_t)seconds * 1000000ULL);
  now_us = s.t_rx_us;
  nav_step(n, &s, now_us, now_us);
}
static void bridge_imu(void)
{ EkfBridge_SubmitImu(0,0,0,0,0,9.81f,30,0,0,now_us); }
static StateEstimate snapshot(void)
{
  StateEstimate s;
  assert(EkfBridge_CopyState(&s) != 0U && critical_depth == 0U);
  return s;
}
static StateEstimate bridge_feed(uint32_t seconds)
{
  GnssSample s = sample(seconds, (uint64_t)seconds * 1000000ULL);
  now_us = s.t_rx_us; bridge_imu(); EkfBridge_SubmitGnss(&s); EkfBridge_Step();
  return snapshot();
}
static uint32_t crc32(const uint8_t *bytes, size_t len)
{
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < len; ++i) {
    crc ^= bytes[i];
    for (unsigned j = 0; j < 8U; ++j) crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return crc ^ UINT32_MAX;
}

int main(int argc, char **argv)
{
  GnssNavigation nav;
  GnssSample s;
  assert(argc == 2);
  running(); GnssNavigation_Init(&nav, NULL);
  if (strcmp(argv[1], "projection") == 0) {
    float n = 99, e = 99;
    assert(GnssNavigation_Project(0,0,0,0,&n,&e) && n == 0 && e == 0);
    assert(GnssNavigation_Project(0,0,1,0,&n,&e));
    assert(fabsf(n - 0.011057428f) < 1e-6f && fabsf(e) < 1e-8f);
    assert(GnssNavigation_Project(0,0,0,1,&n,&e));
    assert(fabsf(e - 0.011131949f) < 1e-6f && fabsf(n) < 1e-8f);
    assert(GnssNavigation_Project(0,0,-1,-1,&n,&e) && n < 0 && e < 0);
    assert(GnssNavigation_Project(302500000,1205000000,302500001,1205000001,&n,&e));
    assert(n > .010f && n < .012f && e > .009f && e < .011f);
    assert(GnssNavigation_Project(0,1799999000,0,-1799999000,&n,&e));
    assert(fabsf(e - 22.263898f) < .002f && fabsf(n) < 1e-5f);
    assert(GnssNavigation_Project(0,-1799999000,0,1799999000,&n,&e) && e < -22.26f);
    assert(GnssNavigation_Project(0,0,0,8000000,&n,&e) && e > 89000 && e < 90000);
    assert(!GnssNavigation_Project(0,0,0,10000000,&n,&e));
    assert(!GnssNavigation_Project(0,0,0,1800000000,&n,&e));
    assert(!GnssNavigation_Project(300000000,0,-300000000,1800000000,&n,&e));
    assert(!GnssNavigation_Project(850000000,0,850000000,0,&n,&e));
    assert(!GnssNavigation_Project(0,0,900000001,0,&n,&e));
    assert(!GnssNavigation_Project(0,0,0,1800000001,&n,&e));
    assert(!GnssNavigation_Project(0,0,0,0,NULL,&e));
  } else if (strcmp(argv[1], "first_and_sparse") == 0) {
    filter.vN = 50.0f;
    nav_feed(&nav,1);
    assert(reset_calls == 1U && gps_calls == 0U && velocity_calls == 0U);
    assert(nav.anchored && nav.diag.origin_id == 1U && nav.accepted_streak == 1U);
    assert(nav.diag.state == GNSS_NAV_POSITION_ONLY && !(nav.diag.flags & GNSS_FLAG_AIDING_VALID));
    assert(filter.vN == 0 && filter.Pvn == 25 && !last_velocity_measured);
    assert(last_vn == 0 && last_ve == 0 && last_vvar == 25);
    assert(nav.diag.reset_before_vn == 50 && nav.diag.reset_after_vn == 0);
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_FIRST_FIX);
    /* 间隔四秒、五秒照常更新位置；未知先验不反复把学到的速度归零。 */
    filter.vN = .8f;
    nav_feed(&nav,5); nav_feed(&nav,10);
    assert(reset_calls == 1U && gps_calls == 2U && velocity_calls == 0U);
    assert(filter.vN == .8f && nav.normal_updates == 2U && nav.accepted_streak == 3U);
    assert(nav.diag.accept_count == 3U && nav.diag.vel_accept_count == 0U);
    assert(!nav.have_velocity && !(nav.diag.flags & GNSS_FLAG_VELOCITY_VALID));
    assert(nav.diag.flags & GNSS_FLAG_AIDING_VALID);
    assert(!(nav.diag.flags & GNSS_FLAG_REFERENCE_VERIFIED));
    /* 小速度协方差来自矩阵学习；此桩仅验证状态判据，真实矩阵另在 pipeline 回放。 */
    filter.Pvn = filter.Pve = 1;
    nav_feed(&nav,14);
    assert(nav.diag.velocity_source == 2U && nav.diag.state == GNSS_NAV_TRACKING);
  } else if (strcmp(argv[1], "epochs") == 0) {
    s = sample(1,1000000ULL); s.utc_date_yyyymmdd = 20261231U; s.utc_ms_of_day = 86399999U;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    s.seq++; s.t_rx_us += 100000ULL;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.duplicate_count == 1U && nav.last_rx_us == 1000000ULL);
    assert(nav.diag.sample_seq == 1U && reset_calls == 1U && gps_calls == 0U);
    s.utc_ms_of_day--;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.reason == GNSS_REASON_OUT_OF_ORDER);
    s.utc_date_yyyymmdd = 20270101U; s.utc_ms_of_day = 0U; s.t_rx_us = 2000000ULL;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.utc_date == 20270101U && nav.diag.utc_ms == 0U && gps_calls == 1U);
    /* 同一次 UART 突发，接收时刻相同但 UTC 历元不同，必须继续融合。 */
    s.seq++; s.utc_ms_of_day = 1000U;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 2U && nav.diag.accept_count == 3U);
    s.quality_source = GNSS_QUALITY_REPORTED; s.position_var_m2 = 9;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 2U && nav.diag.duplicate_count == 2U);
  } else if (strcmp(argv[1], "stale") == 0) {
    s = sample(1,1000000ULL);
    nav_step(&nav,&s,2501000ULL,2501000ULL);
    assert(nav.diag.reason == GNSS_REASON_STALE && nav.have_epoch && !nav.have_rx);
    assert(nav.diag.sample_seq == 0U && nav.diag.utc_ms == 0U);
    GnssNavigation_Init(&nav,NULL);
    nav_step(&nav,&s,999000ULL,999000ULL);
    assert(nav.diag.reason == GNSS_REASON_STALE && nav.have_epoch && !nav.have_rx);
    GnssNavigation_Init(&nav,NULL);
    filter.status = UG_EKF_ALIGNING; filter.horizontal_recovery_ready = 0U;
    nav_step(&nav,&s,1000000ULL,1000000ULL);
    assert(nav.diag.reason == GNSS_REASON_NOT_READY && !nav.anchored);
    assert(nav.diag.flags & GNSS_FLAG_MEASUREMENT);
    running(); nav_step(&nav,&s,1100000ULL,1100000ULL);
    assert(nav.diag.reason == GNSS_REASON_DUPLICATE);
    s = sample(2,2000000ULL);
    nav_step(&nav,&s,2000000ULL,1749000ULL);
    assert(nav.diag.reason == GNSS_REASON_IMU_STALE && nav.diag.state == GNSS_NAV_WAIT_EKF);
    assert(reset_calls == 0U && gps_calls == 0U);
  } else if (strncmp(argv[1], "loss_",5) == 0) {
    unsigned minutes = (unsigned)atoi(argv[1]+5);
    nav_feed(&nav,1);
    uint32_t seconds = 1U + 60U * minutes;
    filter.pN = 1000.0f; filter.pE = -1000.0f; filter.vN = 50.0f;
    nav_step(&nav,NULL,(uint64_t)seconds*1000000ULL,(uint64_t)seconds*1000000ULL);
    assert(nav.diag.state == GNSS_NAV_DR_UNTRUSTED && !(nav.diag.flags & GNSS_FLAG_AIDING_VALID));
    nav_feed(&nav,seconds);
    assert(reset_calls == 2U && gps_calls == 0U && velocity_calls == 0U);
    assert(nav.diag.origin_id == 1U && nav.diag.origin_lat_e7 == 302500000);
    assert(last_n == 0 && last_e == 0 && filter.vN == 0 && filter.vE == 0);
    assert(filter.Pvn == 25 && filter.Pve == 25 && !last_velocity_measured);
    assert(nav.diag.reset_before_n == 1000 && nav.diag.reset_before_vn == 50);
    assert(nav.diag.reset_after_n == 0 && nav.diag.reset_after_vn == 0);
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_AID_LOSS);
    assert(nav.diag.vel_accept_count == 0U && nav.diag.vel_fuse_age_ms == UINT32_MAX);
    nav_feed(&nav,seconds+4U);
    assert(reset_calls == 2U && gps_calls == 1U);
  } else if (strcmp(argv[1], "no_fix") == 0) {
    nav_feed(&nav,1);
    s=sample(2,2000000ULL); s.validity_flags=GNSS_VALID_CHECKSUM|GNSS_VALID_UTC|GNSS_VALID_NO_FIX;
    s.lat_deg_e7=s.lon_deg_e7=0;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.no_fix && nav.diag.no_fix_count == 1U && nav.diag.reason == GNSS_REASON_NO_FIX);
    assert(nav.last_fuse_us == 1000000ULL && nav.last_rx_us == 1000000ULL);
    assert(!(nav.diag.flags & GNSS_FLAG_MEASUREMENT) && nav.diag.state == GNSS_NAV_REACQUIRING);
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.no_fix_count == 1U && nav.diag.duplicate_count == 1U);
    nav_feed(&nav,3);
    assert(!nav.no_fix && gps_calls == 1U && reset_calls == 1U);
  } else if (strcmp(argv[1], "velocity") == 0) {
    s=sample(1,1000000ULL); s.validity_flags |= GNSS_VALID_VELOCITY;
    s.vel_n_mps=1; s.velocity_source=GNSS_VELOCITY_RMC;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(reset_calls == 1U && last_velocity_measured && filter.vN == 1 && filter.Pvn == 1);
    assert(velocity_calls == 0U && nav.diag.vel_accept_count == 1U && nav.have_velocity);
    assert(!(nav.diag.flags & GNSS_FLAG_VELOCITY_VALID));
    s=sample(2,2000000ULL); s.validity_flags |= GNSS_VALID_VELOCITY;
    s.vel_e_mps=1; s.velocity_source=GNSS_VELOCITY_RMC;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 1U && velocity_calls == 1U && filter.vE == 1);
    assert(nav.diag.vel_accept_count == 2U && nav.diag.velocity_source == 1U);
    assert(nav.diag.state == GNSS_NAV_TRACKING && (nav.diag.flags & GNSS_FLAG_VELOCITY_VALID));
    s=sample(3,3000000ULL); s.validity_flags |= GNSS_VALID_VELOCITY; s.vel_n_mps=50;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 2U && velocity_calls == 1U && reset_calls == 1U);
    assert(!(nav.diag.flags & GNSS_FLAG_VELOCITY_MEASUREMENT));
    s=sample(4,4000000ULL); s.validity_flags |= GNSS_VALID_VELOCITY; s.vel_n_mps=NAN;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 3U && velocity_calls == 1U);
  } else if (strcmp(argv[1], "candidate") == 0) {
    nav_feed(&nav,1);
    s=sample(2,2000000ULL); s.lat_deg_e7 += 100000;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.have_candidate && nav.diag.reason == GNSS_REASON_JUMP && gps_calls == 0U);
    assert(reset_calls == 1U && nav.diag.meas_n > 1000.0f && filter.pN == 0);
    s.seq++; s.t_rx_us+=1000000ULL; s.utc_ms_of_day+=1000U;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(reset_calls == 2U && !nav.have_candidate && filter.pN > 1000.0f);
    assert(nav.diag.origin_lat_e7 == 302500000 && nav.diag.origin_id == 1U);
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_REJECT);
  } else if (strcmp(argv[1], "candidate_disappears") == 0) {
    nav_feed(&nav,1); s=sample(2,2000000ULL); s.lat_deg_e7+=100000;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us); nav_feed(&nav,3);
    assert(!nav.have_candidate && gps_calls == 1U && reset_calls == 1U && filter.pN == 0);
  } else if (strcmp(argv[1], "nis_recovery") == 0) {
    nav_feed(&nav,1); filter.pN=20; gps_result=UG_EKF_GPS_REJECTED;
    nav_feed(&nav,7);
    assert(gps_calls == 1U && reset_calls == 2U && filter.pN == 0);
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_REJECT && nav.diag.reject_count == 1U);
    gps_result=UG_EKF_GPS_INVALID; nav_feed(&nav,8);
    assert(reset_calls == 2U && nav.diag.reason == GNSS_REASON_NUMERIC);
  } else if (strcmp(argv[1], "horizontal_fault") == 0) {
    nav_feed(&nav,1);
    filter.finite=filter.healthy=0; filter.status=UG_EKF_FAULT; filter.pN=NAN; filter.vN=NAN;
    filter.horizontal_recovery_ready=1;
    nav_feed(&nav,2);
    assert(reset_calls == 2U && isfinite(filter.pN) && isfinite(filter.vN));
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_NUMERIC);
    assert((nav.diag.reset_valid_mask & 5U) == 0 && (nav.diag.reset_valid_mask & 240U) == 240U);
    filter.finite=filter.healthy=0; filter.horizontal_recovery_ready=0;
    nav_feed(&nav,3);
    assert(reset_calls == 2U && nav.diag.state == GNSS_NAV_WAIT_EKF);
  } else if (strcmp(argv[1], "filter_reset") == 0) {
    nav_feed(&nav,1); uint64_t epoch=nav.last_epoch_ms;
    GnssNavigation_OnFilterReset(&nav,2000000ULL);
    assert(nav.diag.origin_id == 1U && nav.last_epoch_ms == epoch && !nav.anchored);
    s=sample(2,1500000ULL); s.utc_ms_of_day=2000;
    nav_step(&nav,&s,2100000ULL,2100000ULL);
    assert(nav.diag.reason == GNSS_REASON_STALE && nav.last_epoch_ms > epoch);
    assert(nav.last_rx_us == 1000000ULL && nav.diag.utc_ms == 1000U);
    s=sample(3,2500000ULL); s.utc_ms_of_day=2000;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.reason == GNSS_REASON_DUPLICATE && !nav.anchored);
    nav_feed(&nav,3);
    assert(reset_calls == 2U && nav.anchored && nav.diag.origin_id == 1U);
    assert(nav.diag.recovery_reason == GNSS_RECOVERY_FILTER_RESET);
  } else if (strcmp(argv[1], "bad_input_covariance") == 0) {
    s=sample(1,1000000ULL); s.validity_flags &= ~GNSS_VALID_CHECKSUM;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.reason == GNSS_REASON_BAD_INPUT && !nav.have_epoch);
    s=sample(1,1000000ULL); s.utc_date_yyyymmdd=20260229U;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.reason == GNSS_REASON_BAD_INPUT && !nav.have_epoch);
    nav_feed(&nav,1);
    s=sample(2,2000000ULL); s.quality_source=GNSS_QUALITY_REPORTED; s.position_var_m2=NAN;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(nav.diag.reason == GNSS_REASON_QUALITY && !(nav.diag.flags & GNSS_FLAG_MEASUREMENT));
    assert(gps_calls == 0U);
    s=sample(3,3000000ULL); s.quality_source=GNSS_QUALITY_REPORTED; s.position_var_m2=9;
    nav_step(&nav,&s,s.t_rx_us,s.t_rx_us);
    assert(gps_calls == 1U && filter.Pnn == 9);
    filter.Pnn=NAN; nav_step(&nav,NULL,3001000ULL,3001000ULL);
    assert(!(nav.diag.flags & GNSS_FLAG_AIDING_VALID) && !(nav.diag.flags & GNSS_FLAG_COVARIANCE));
    filter.Pnn=25; filter.Pee=25; filter.Pne=26;
    nav_step(&nav,NULL,3002000ULL,3002000ULL);
    assert(!(nav.diag.flags & GNSS_FLAG_COVARIANCE));
  } else if (strcmp(argv[1], "bridge_fifo") == 0) {
    StateEstimate st;
    s=sample(1,1000000ULL); EkfBridge_SubmitGnss(&s); /* 初始化前不收。 */
    EkfBridge_Init(); EkfBridge_SubmitGnss(NULL); EkfBridge_SubmitGnss(&s);
    s=sample(2,1000000ULL); s.utc_ms_of_day=2000; EkfBridge_SubmitGnss(&s);
    now_us=1000000ULL; bridge_imu(); input_rejects=7U; EkfBridge_Step(); st=snapshot();
    assert(st.gnss.sample_seq == 2U && st.gnss.drop_count == 0U && st.gnss.input_reject_count == 7U);
    assert(reset_calls == 1U && gps_calls == 1U && st.gnss.accept_count == 2U);
    now_us+=20000ULL; EkfBridge_Step(); st=snapshot();
    assert(st.gnss.duplicate_count == 0U && gps_calls == 1U);
    /* 十帧涌入：容量八帧丢最旧两帧；每次最多处理四帧，保留其余给下一次。 */
    now_us=2000000ULL;
    for (uint32_t i=3;i<=12;i++) {
      s=sample(i,now_us); s.utc_ms_of_day=i*1000U; EkfBridge_SubmitGnss(&s);
    }
    bridge_imu(); EkfBridge_Step(); st=snapshot();
    assert(st.gnss.drop_count == 2U && st.gnss.sample_seq == 8U && gps_calls == 5U);
    now_us+=20000ULL; bridge_imu(); EkfBridge_Step(); st=snapshot();
    assert(st.gnss.sample_seq == 12U && st.gnss.drop_count == 2U && gps_calls == 9U);
    EkfBridge_Step(); assert(gps_calls == 9U && critical_entries > 10U);
  } else if (strcmp(argv[1], "bridge_mag_reset") == 0) {
    float table[16]={0}; uint32_t crc; StateEstimate st;
    table[3]=table[7]=table[11]=1;
    crc=crc32((const uint8_t *)table,sizeof(table));
    EkfBridge_Init(); st=bridge_feed(1); assert(reset_calls == 1U);
    s=sample(2,2000000ULL); EkfBridge_SubmitGnss(&s); now_us=2100000ULL;
    assert(EkfBridge_SubmitMagCalibration((const uint8_t *)table,crc));
    EkfBridge_Step(); st=snapshot();
    assert(init_calls == 2U && g_ekf_mag_cal_apply_count == 1U);
    assert(st.gnss.origin_id == 1U && st.gnss.origin_lat_e7 == 302500000);
    assert(st.gnss.utc_ms == 1000U && st.gnss.reason == GNSS_REASON_STALE);
    assert(!(st.gnss.flags & GNSS_FLAG_ANCHORED));
    now_us=2500000ULL; bridge_imu(); s=sample(3,now_us); s.utc_ms_of_day=2000;
    EkfBridge_SubmitGnss(&s); EkfBridge_Step(); st=snapshot();
    assert(st.gnss.reason == GNSS_REASON_DUPLICATE && reset_calls == 1U);
    st=bridge_feed(3);
    assert(reset_calls == 2U && st.gnss.origin_id == 1U && st.gnss.origin_lat_e7 == 302500000);
    assert(st.gnss.recovery_reason == GNSS_RECOVERY_FILTER_RESET);
    now_us+=20000ULL;
    assert(EkfBridge_SubmitMagCalibration((const uint8_t *)table,crc));
    EkfBridge_Step(); st=snapshot();
    assert(init_calls == 2U && g_ekf_mag_cal_apply_count == 1U && (st.gnss.flags & GNSS_FLAG_ANCHORED));
  } else { assert(0); }
  assert(critical_depth == 0U);
  return 0;
}
"""


class GnssNavigationHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None and Path("D:/msys64/mingw64/bin/gcc.exe").is_file():
            compiler = "D:/msys64/mingw64/bin/gcc.exe"
        if compiler is None:
            raise unittest.SkipTest("需要主机 gcc 执行 GNSS 导航/桥实际源码")
        tmp_root = ROOT.parent / ".buildtmp"
        tmp_root.mkdir(exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="gnss_navigation_", dir=tmp_root)
        cls.addClassCleanup(cls.tmp.cleanup)
        work = Path(cls.tmp.name)
        (work / "FreeRTOS.h").write_text("/* 主机不启动调度器。 */\n", encoding="utf-8")
        (work / "task.h").write_text(TASK_STUB, encoding="utf-8")
        (work / "pressure_sensor_driver.h").write_text(PRESSURE_STUB, encoding="utf-8")
        (work / "glider_core.h").write_text("#include <stdint.h>\nuint32_t GliderCore_GetGnssRejectedCount(void);\n", encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        cls.env = dict(os.environ)
        cls.env["PATH"] = str(Path(compiler).parent) + os.pathsep + cls.env.get("PATH", "")
        cls.env.update(TMP=str(work), TEMP=str(work), TMPDIR=str(work))
        cls.binary = work / ("gnss_navigation.exe" if os.name == "nt" else "gnss_navigation")
        result = subprocess.run(
            [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(work),
             "-I", str(ROOT / "software/Glider-App/common"),
             "-I", str(ROOT / "software/Glider-App/estimation"),
             "-I", str(ROOT / "software/Glider-App/estimation/ug_estimation/include"),
             str(ROOT / "software/Glider-App/estimation/gnss_navigation.c"),
             str(ROOT / "software/Glider-App/estimation/ekf_bridge.c"),
             str(work / "harness.c"), "-lm", "-o", str(cls.binary)],
            capture_output=True, text=True, encoding="utf-8", errors="replace", env=cls.env)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def run_case(self, case):
        result = subprocess.run([str(self.binary), case], capture_output=True, text=True,
                                encoding="utf-8", errors="replace", env=self.env)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_projection_centimeters_directions_dateline_and_far_point_rejection(self):
        self.run_case("projection")

    def test_first_fix_immediate_and_sparse_updates_do_not_repeat_zero_prior(self):
        self.run_case("first_and_sparse")

    def test_duplicate_out_of_order_midnight_stationary_and_same_rx_new_epochs(self):
        self.run_case("epochs")

    def test_old_future_unaligned_and_imu_stale_samples_do_not_fuse(self):
        self.run_case("stale")

    def test_five_twenty_sixty_minute_loss_first_fix_recovers_p_and_v(self):
        for minutes in (5, 20, 60):
            with self.subTest(minutes=minutes):
                self.run_case(f"loss_{minutes}")

    def test_no_fix_event_does_not_refresh_position_or_fusion_age(self):
        self.run_case("no_fix")

    def test_measured_velocity_and_invalid_optional_velocity(self):
        self.run_case("velocity")

    def test_jump_requires_second_epoch_and_preserves_origin(self):
        self.run_case("candidate")

    def test_isolated_jump_followed_by_original_cluster_is_not_reanchored(self):
        self.run_case("candidate_disappears")

    def test_nis_reject_after_aid_loss_can_reanchor_same_epoch(self):
        self.run_case("nis_recovery")

    def test_horizontal_numeric_fault_can_recover_but_retained_fault_waits(self):
        self.run_case("horizontal_fault")

    def test_filter_reset_preserves_origin_and_consumes_stale_epoch(self):
        self.run_case("filter_reset")

    def test_bad_quality_and_covariance_do_not_claim_valid_aiding(self):
        self.run_case("bad_input_covariance")

    def test_actual_bridge_fifo_burst_overflow_budget_and_single_consumption(self):
        self.run_case("bridge_fifo")

    def test_actual_bridge_mag_reset_keeps_origin_and_first_new_fix_reanchors(self):
        self.run_case("bridge_mag_reset")


if __name__ == "__main__":
    unittest.main()
