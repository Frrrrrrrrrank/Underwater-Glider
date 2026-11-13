#include "gnss_navigation.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static uint32_t age_ms(uint64_t now, uint64_t then, uint8_t present)
{
  uint64_t age;
  if (!present || now < then) return UINT32_MAX;
  age = (now - then) / 1000ULL;
  return age >= UINT32_MAX ? UINT32_MAX - 1U : (uint32_t)age;
}

/* 日期序键只用于历元先后比较，不用于计算秒数；实际超时统一用 MCU 单调时钟。 */
static int epoch_key(const GnssSample *s, uint64_t *key)
{
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  uint32_t year = s->utc_date_yyyymmdd / 10000U;
  uint32_t month = s->utc_date_yyyymmdd / 100U % 100U;
  uint32_t day = s->utc_date_yyyymmdd % 100U;
  uint32_t limit;
  if (year < 1980U || year > 2099U || month < 1U || month > 12U ||
      s->utc_ms_of_day >= 86400000U) return 0;
  limit = days[month - 1U];
  if (month == 2U && year % 4U == 0U &&
      (year % 100U != 0U || year % 400U == 0U)) limit++;
  if (day < 1U || day > limit) return 0;
  *key = (uint64_t)s->utc_date_yyyymmdd * 86400000ULL + s->utc_ms_of_day;
  return 1;
}

int GnssNavigation_Project(int32_t origin_lat_e7, int32_t origin_lon_e7,
                           int32_t lat_e7, int32_t lon_e7, float *north, float *east)
{
  /* 两点均投影到 WGS84 椭球面，再作 ECEF 差并旋到原点 NE。
   * RMC 没有高度，不虚构垂直观测。经差 int64 防日期变更线溢出。 */
  const double rad = 0.017453292519943295769;
  const double e2 = 6.6943799901413165e-3;
  int64_t dlon;
  double phi0, phi, dl, sin0, cos0, normal0, normal, dx, dz, n, e;
  if (!north || !east ||
      origin_lat_e7 <= -850000000 || origin_lat_e7 >= 850000000 ||
      lat_e7 < -900000000 || lat_e7 > 900000000 ||
      origin_lon_e7 < -1800000000 || origin_lon_e7 > 1800000000 ||
      lon_e7 < -1800000000 || lon_e7 > 1800000000) return 0;
  dlon = (int64_t)lon_e7 - origin_lon_e7;
  if (dlon > 1800000000LL) dlon -= 3600000000LL;
  if (dlon < -1800000000LL) dlon += 3600000000LL;
  phi0 = (double)origin_lat_e7 * 1.0e-7 * rad;
  phi = (double)lat_e7 * 1.0e-7 * rad;
  dl = (double)dlon * 1.0e-7 * rad;
  sin0 = sin(phi0);
  cos0 = cos(phi0);
  normal0 = 6378137.0 / sqrt(1.0 - e2 * sin0 * sin0);
  normal = 6378137.0 / sqrt(1.0 - e2 * sin(phi) * sin(phi));
  dx = normal * cos(phi) * cos(dl) - normal0 * cos0;
  dz = (1.0 - e2) * (normal * sin(phi) - normal0 * sin0);
  n = -sin0 * dx + cos0 * dz;
  e = normal * cos(phi) * sin(dl);
  /* 必须检查完整 ECEF 弦长：对跖点的 NE 投影接近零，不能据此当作近点。 */
  if (!isfinite(n) || !isfinite(e) || dx * dx + e * e + dz * dz > 1.0e10) return 0;
  *north = (float)n;
  *east = (float)e;
  return 1;
}

void GnssNavigation_DefaultConfig(GnssNavConfig *c)
{
  if (!c) return;
  memset(c, 0, sizeof(*c));
  c->position_sigma_m = 5.0f;
  c->velocity_sigma_mps = 1.0f;
  c->unknown_velocity_sigma_mps = 5.0f;
  c->max_speed_mps = 5.0f;
  c->max_position_std_m = 50.0f;
  c->max_velocity_std_mps = 2.0f;
  c->max_sample_age_ms = 1500U;
  c->fix_timeout_ms = 5000U;
  c->max_imu_age_ms = 250U;
  c->velocity_confirm_ms = 2000U;
  c->long_loss_ms = 30000U;
  c->min_good_fixes = 2U;
}
void GnssNavigation_Init(GnssNavigation *n, const GnssNavConfig *c)
{
  GnssNavConfig d;
  if (!n) return;
  GnssNavigation_DefaultConfig(&d);
  memset(n, 0, sizeof(*n));
  n->config = c ? *c : d;
#define POSITIVE(x) (isfinite(n->config.x) && n->config.x > 0.0f)
  if (!POSITIVE(position_sigma_m) || !POSITIVE(velocity_sigma_mps) ||
      !POSITIVE(unknown_velocity_sigma_mps) || !POSITIVE(max_speed_mps) ||
      !POSITIVE(max_position_std_m) || !POSITIVE(max_velocity_std_mps) ||
      !n->config.max_sample_age_ms || !n->config.fix_timeout_ms ||
      !n->config.max_imu_age_ms || !n->config.velocity_confirm_ms ||
      n->config.long_loss_ms < n->config.fix_timeout_ms || n->config.min_good_fixes < 2U)
    n->config = d;
#undef POSITIVE
  n->diag.r_var = n->config.position_sigma_m * n->config.position_sigma_m;
  n->diag.vel_r_var = n->config.velocity_sigma_mps * n->config.velocity_sigma_mps;
  n->diag.rx_age_ms = n->diag.fuse_age_ms = n->diag.vel_fuse_age_ms = UINT32_MAX;
  n->diag.reset_age_ms = UINT32_MAX;
}
void GnssNavigation_OnFilterReset(GnssNavigation *n, uint64_t now)
{
  if (!n) return;
  n->anchored = n->have_fused = n->have_velocity = 0U;
  n->have_previous = n->have_candidate = n->no_fix = 0U;
  n->accepted_streak = n->normal_updates = 0U;
  n->velocity_evidence_us = 0U;
  n->filter_reset_us = now;
  n->force_recovery = GNSS_RECOVERY_FILTER_RESET;
  n->diag.flags &= GNSS_FLAG_ORIGIN | GNSS_FLAG_REFERENCE_VERIFIED | GNSS_FLAG_RECOVERY;
  n->diag.velocity_source = 0U;
  n->diag.state = GNSS_NAV_WAIT_EKF;
  n->diag.reason = GNSS_REASON_NOT_READY;
  n->diag.fuse_age_ms = n->diag.vel_fuse_age_ms = UINT32_MAX;
}
static int imu_fresh(uint64_t now, uint64_t imu, const GnssNavConfig *c)
{
  return imu && now >= imu && now - imu <= (uint64_t)c->max_imu_age_ms * 1000ULL;
}
static int ready(const UgEkfOutput *o)
{
  return o->status == UG_EKF_RUNNING && o->healthy && o->finite;
}
static int covariance(float nn, float ne, float ee)
{
  return isfinite(nn) && isfinite(ne) && isfinite(ee) && nn >= 0 && ee >= 0 &&
         (double)nn * ee + 1.0e-6 >= (double)ne * ne;
}
static int velocity_sane(const GnssNavigation *n, const UgEkfOutput *o)
{
  return isfinite(o->vN) && isfinite(o->vE) &&
         hypotf(o->vN, o->vE) <= n->config.max_speed_mps;
}
static int velocity_cov_small(const GnssNavigation *n, const UgEkfOutput *o)
{
  const float v = n->config.max_velocity_std_mps * n->config.max_velocity_std_mps;
  return covariance(o->Pvn, o->Pvne, o->Pve) &&
         o->Pvn + fabsf(o->Pvne) <= v && o->Pve + fabsf(o->Pvne) <= v;
}
static int dr_untrusted(const GnssNavigation *n, const UgEkfOutput *o)
{
  /* 大P时严重漂移也可能通过NIS，因此先检查失去绝对约束的时间。 */
  return !ready(o) || !velocity_sane(n, o) ||
         !covariance(o->Pvn, o->Pvne, o->Pve) ||
         n->diag.fuse_age_ms >= n->config.long_loss_ms;
}
static void refresh(GnssNavigation *n, uint64_t now, uint64_t imu, const UgEkfOutput *o)
{
  uint32_t f = n->diag.flags & (GNSS_FLAG_MEASUREMENT | GNSS_FLAG_INNOVATION |
      GNSS_FLAG_VELOCITY_MEASUREMENT | GNSS_FLAG_VELOCITY_INNOVATION | GNSS_FLAG_RECOVERY);
  const float p = n->config.max_position_std_m * n->config.max_position_std_m;
  int pcov = covariance(o->Pnn, o->Pne, o->Pee);
  n->diag.rx_age_ms = age_ms(now, n->last_rx_us, n->have_rx);
  n->diag.fuse_age_ms = age_ms(now, n->last_fuse_us, n->have_fused);
  n->diag.vel_fuse_age_ms = age_ms(now, n->last_velocity_us, n->have_velocity);
  n->diag.reset_age_ms = age_ms(now, n->last_reset_us, n->diag.reset_count != 0U);
  if (n->diag.origin_id) f |= GNSS_FLAG_ORIGIN;
  if (n->anchored) f |= GNSS_FLAG_ANCHORED;
  if (n->have_fused) f |= GNSS_FLAG_FUSED_ONCE;
  if (n->config.reference_verified) f |= GNSS_FLAG_REFERENCE_VERIFIED;
  if (pcov) f |= GNSS_FLAG_COVARIANCE;
  if (covariance(o->Pvn, o->Pvne, o->Pve)) f |= GNSS_FLAG_VELOCITY_COVARIANCE;
  if (ready(o) && velocity_sane(n, o) && velocity_cov_small(n, o) &&
      n->accepted_streak >= n->config.min_good_fixes && n->diag.velocity_source &&
      age_ms(now, n->velocity_evidence_us, n->velocity_evidence_us != 0) < n->config.long_loss_ms)
    f |= GNSS_FLAG_VELOCITY_VALID;
  if (!imu_fresh(now, imu, &n->config) || (!ready(o) && !o->horizontal_recovery_ready))
    n->diag.state = GNSS_NAV_WAIT_EKF;
  else if (!n->anchored) n->diag.state = GNSS_NAV_WAIT_FIX;
  else if (dr_untrusted(n, o)) n->diag.state = GNSS_NAV_DR_UNTRUSTED;
  else if (n->no_fix || n->diag.fuse_age_ms > n->config.fix_timeout_ms)
    n->diag.state = GNSS_NAV_REACQUIRING;
  else {
    n->diag.state = (f & GNSS_FLAG_VELOCITY_VALID) ? GNSS_NAV_TRACKING : GNSS_NAV_POSITION_ONLY;
    if (pcov && o->Pnn + fabsf(o->Pne) <= p && o->Pee + fabsf(o->Pne) <= p &&
        n->accepted_streak >= n->config.min_good_fixes) f |= GNSS_FLAG_AIDING_VALID;
  }
  if (n->diag.state == GNSS_NAV_WAIT_EKF || n->diag.state == GNSS_NAV_DR_UNTRUSTED)
    f &= ~(GNSS_FLAG_AIDING_VALID | GNSS_FLAG_VELOCITY_VALID);
  n->diag.flags = f;
}
static void reject(GnssNavigation *n, uint32_t reason)
{
  n->diag.reason = reason; n->diag.reject_count++;
}
static int nearby(const GnssNavigation *n, int32_t lat, int32_t lon, uint64_t t,
                  const GnssSample *s)
{
  float dn, de, dt, limit;
  if (s->t_rx_us < t) return 0;
  dt = (float)((double)(s->t_rx_us - t) * 1.0e-6);
  limit = 3.0f * sqrtf(2.0f * n->diag.r_var) + n->config.max_speed_mps * dt;
  return GnssNavigation_Project(lat, lon, s->lat_deg_e7, s->lon_deg_e7, &dn, &de) &&
         hypotf(dn, de) <= limit;
}
static void record_reset(GnssNavigation *n, const UgEkfOutput *before,
                         const UgEkfOutput *after, uint64_t t, uint32_t reason)
{
  float values[8] = {before->pN, before->pE, before->vN, before->vE,
                    after->pN, after->pE, after->vN, after->vE};
  float *dest[8] = {&n->diag.reset_before_n, &n->diag.reset_before_e,
    &n->diag.reset_before_vn, &n->diag.reset_before_ve, &n->diag.reset_after_n,
    &n->diag.reset_after_e, &n->diag.reset_after_vn, &n->diag.reset_after_ve};
  n->diag.reset_valid_mask = 0U;
  for (unsigned i = 0; i < 8; ++i) {
    *dest[i] = isfinite(values[i]) ? values[i] : 0.0f;
    if (isfinite(values[i])) n->diag.reset_valid_mask |= 1U << i;
  }
  n->diag.recovery_reason = reason; n->diag.flags |= GNSS_FLAG_RECOVERY;
  n->diag.reset_count++; n->last_reset_us = t;
}
void GnssNavigation_Step(GnssNavigation *n, const GnssSample *s,
                         uint64_t now, uint64_t imu, const UgEkfOutput *out)
{
  uint64_t epoch;
  uint32_t recovery = GNSS_RECOVERY_NONE;
  float dn = 0, de = 0;
  int result, measured_v, confirmed_candidate = 0;
  UgEkfOutput after;
  if (!n || !out) return;
  refresh(n, now, imu, out);
  if (!s) return;
  if ((s->validity_flags & (GNSS_VALID_CHECKSUM | GNSS_VALID_UTC)) !=
      (GNSS_VALID_CHECKSUM | GNSS_VALID_UTC) || !epoch_key(s, &epoch)) {
    reject(n, GNSS_REASON_BAD_INPUT); return;
  }
  if (n->have_epoch && epoch == n->last_epoch_ms) {
    n->diag.duplicate_count++; n->diag.reason = GNSS_REASON_DUPLICATE; return;
  }
  if (n->have_epoch && epoch < n->last_epoch_ms) {
    reject(n, GNSS_REASON_OUT_OF_ORDER); return;
  }
  n->have_epoch = 1U; n->last_epoch_ms = epoch;
  if (now < s->t_rx_us || s->t_rx_us <= n->filter_reset_us ||
      s->t_rx_us < n->last_event_us ||
      now - s->t_rx_us > (uint64_t)n->config.max_sample_age_ms * 1000ULL) {
    reject(n, GNSS_REASON_STALE); return;
  }
  n->last_event_us = s->t_rx_us;
  n->diag.sample_seq = s->seq;
  n->diag.utc_date = s->utc_date_yyyymmdd; n->diag.utc_ms = s->utc_ms_of_day;
  n->diag.flags &= ~(GNSS_FLAG_INNOVATION | GNSS_FLAG_VELOCITY_INNOVATION |
                      GNSS_FLAG_VELOCITY_MEASUREMENT);
  if (s->validity_flags & GNSS_VALID_NO_FIX) {
    n->no_fix = 1U; n->diag.no_fix_count++;
    n->diag.flags &= ~GNSS_FLAG_MEASUREMENT; n->diag.reason = GNSS_REASON_NO_FIX;
    refresh(n, now, imu, out); return;
  }
  if ((s->validity_flags & GNSS_VALID_REQUIRED) != GNSS_VALID_REQUIRED ||
      s->lat_deg_e7 < -900000000 || s->lat_deg_e7 > 900000000 ||
      s->lon_deg_e7 < -1800000000 || s->lon_deg_e7 > 1800000000) {
    reject(n, GNSS_REASON_BAD_INPUT); return;
  }
  n->last_rx_us = s->t_rx_us; n->have_rx = 1U; n->no_fix = 0U;
  n->diag.lat_e7 = s->lat_deg_e7; n->diag.lon_e7 = s->lon_deg_e7;
  n->diag.flags &= ~GNSS_FLAG_MEASUREMENT;
  n->diag.quality_source = s->quality_source;
  n->diag.r_var = n->config.position_sigma_m * n->config.position_sigma_m;
  if (s->quality_source == 1U) {
    if (!isfinite(s->position_var_m2) || s->position_var_m2 <= 0) {
      reject(n, GNSS_REASON_QUALITY); return;
    }
    n->diag.r_var = s->position_var_m2;
  }
  n->diag.flags |= GNSS_FLAG_MEASUREMENT;
  if (n->diag.origin_id) {
    if (!GnssNavigation_Project(n->diag.origin_lat_e7, n->diag.origin_lon_e7,
        s->lat_deg_e7, s->lon_deg_e7, &dn, &de)) {
      n->diag.flags &= ~GNSS_FLAG_MEASUREMENT;
      reject(n, GNSS_REASON_BAD_INPUT); return;
    }
    n->diag.meas_n = dn; n->diag.meas_e = de;
  }
  measured_v = (s->validity_flags & GNSS_VALID_VELOCITY) &&
      isfinite(s->vel_n_mps) && isfinite(s->vel_e_mps) &&
      hypotf(s->vel_n_mps, s->vel_e_mps) <= n->config.max_speed_mps;
  n->diag.vel_r_var = n->config.velocity_sigma_mps * n->config.velocity_sigma_mps;
  if (isfinite(s->velocity_var_m2ps2) && s->velocity_var_m2ps2 > 0)
    n->diag.vel_r_var = s->velocity_var_m2ps2;
  if (measured_v) {
    n->diag.meas_vn = s->vel_n_mps; n->diag.meas_ve = s->vel_e_mps;
    n->diag.flags |= GNSS_FLAG_VELOCITY_MEASUREMENT;
  }
  /* 检查GPS自身时序；单个跳点隔离，下一独立历元与候选一致时允许更换定位簇。 */
  if (n->have_previous && s->t_rx_us >= n->previous_rx_us &&
      s->t_rx_us - n->previous_rx_us < (uint64_t)n->config.long_loss_ms * 1000ULL &&
      !nearby(n, n->previous_lat_e7, n->previous_lon_e7, n->previous_rx_us, s)) {
    if (n->have_candidate && s->t_rx_us >= n->candidate_rx_us &&
        s->t_rx_us - n->candidate_rx_us < (uint64_t)n->config.long_loss_ms * 1000ULL &&
        nearby(n, n->candidate_lat_e7, n->candidate_lon_e7, n->candidate_rx_us, s))
      confirmed_candidate = 1;
    else {
      n->candidate_lat_e7 = s->lat_deg_e7; n->candidate_lon_e7 = s->lon_deg_e7;
      n->candidate_rx_us = s->t_rx_us; n->have_candidate = 1U;
      reject(n, GNSS_REASON_JUMP); refresh(n, now, imu, out); return;
    }
  }
  if (!imu_fresh(now, imu, &n->config) || (!ready(out) && !out->horizontal_recovery_ready)) {
    reject(n, !imu_fresh(now, imu, &n->config) ? GNSS_REASON_IMU_STALE : GNSS_REASON_NOT_READY);
    refresh(n, now, imu, out); return;
  }
  if (!n->diag.origin_id) {
    if (!GnssNavigation_Project(s->lat_deg_e7, s->lon_deg_e7,
        s->lat_deg_e7, s->lon_deg_e7, &dn, &de)) { reject(n, GNSS_REASON_BAD_INPUT); return; }
    n->diag.origin_lat_e7 = s->lat_deg_e7; n->diag.origin_lon_e7 = s->lon_deg_e7;
    n->diag.origin_id = 1U;
  }
  if (!GnssNavigation_Project(n->diag.origin_lat_e7, n->diag.origin_lon_e7,
      s->lat_deg_e7, s->lon_deg_e7, &dn, &de)) {
    n->diag.flags &= ~GNSS_FLAG_MEASUREMENT; reject(n, GNSS_REASON_BAD_INPUT); return;
  }
  n->diag.meas_n = dn; n->diag.meas_e = de;
  if (!n->anchored) recovery = n->force_recovery ? n->force_recovery : GNSS_RECOVERY_FIRST_FIX;
  else if (!ready(out)) recovery = GNSS_RECOVERY_NUMERIC;
  else if (dr_untrusted(n, out)) recovery = GNSS_RECOVERY_AID_LOSS;
  else if (confirmed_candidate) recovery = GNSS_RECOVERY_REJECT;
  if (!recovery) {
    result = UgEkf_PushGpsPosition(dn, de, n->diag.r_var, 0, n->diag.r_var, epoch, s->t_rx_us);
    UgEkf_GetOutput(&after);
    if (isfinite(after.nis_gps) && isfinite(after.gps_innov_n) && isfinite(after.gps_innov_e) &&
        (result == UG_EKF_GPS_ACCEPTED || result == UG_EKF_GPS_REJECTED)) {
      n->diag.innov_n = after.gps_innov_n; n->diag.innov_e = after.gps_innov_e;
      n->diag.nis = after.nis_gps; n->diag.flags |= GNSS_FLAG_INNOVATION;
    }
    /* 短断续先普通更新保留Ppv；失去辅助后NIS拒绝允许直接恢复。 */
    if (result == UG_EKF_GPS_REJECTED && n->diag.fuse_age_ms > n->config.fix_timeout_ms) {
      n->diag.reject_count++; recovery = GNSS_RECOVERY_REJECT;
    }
  }
  if (recovery) {
    float vv = measured_v ? n->diag.vel_r_var :
        n->config.unknown_velocity_sigma_mps * n->config.unknown_velocity_sigma_mps;
    result = UgEkf_ReanchorHorizontal(dn, de, n->diag.r_var, 0, n->diag.r_var,
        measured_v ? s->vel_n_mps : 0.0f, measured_v ? s->vel_e_mps : 0.0f,
        vv, 0, vv, (uint32_t)measured_v, epoch, s->t_rx_us);
    UgEkf_GetOutput(&after);
    /* 队列中较旧RMC可能找不到同刻gyro，不能因此丢掉可用的位置锚点。
     * 这次速度只留作原始诊断，原子恢复降级为未知速度先验。 */
    if (result == UG_EKF_GPS_NOT_READY && measured_v && after.horizontal_recovery_ready) {
      measured_v = 0;
      n->diag.vel_reject_count++;
      vv = n->config.unknown_velocity_sigma_mps * n->config.unknown_velocity_sigma_mps;
      result = UgEkf_ReanchorHorizontal(dn, de, n->diag.r_var, 0, n->diag.r_var,
          0, 0, vv, 0, vv, 0U, epoch, s->t_rx_us);
      UgEkf_GetOutput(&after);
    }
  }
  if (result != UG_EKF_GPS_ACCEPTED) {
    reject(n, result == UG_EKF_GPS_REJECTED ? GNSS_REASON_NIS :
        result == UG_EKF_GPS_NOT_READY ? GNSS_REASON_NOT_READY : GNSS_REASON_NUMERIC);
    refresh(n, now, imu, &after); return;
  }
  n->anchored = n->have_fused = n->have_previous = 1U;
  n->last_fuse_us = n->previous_rx_us = s->t_rx_us;
  n->previous_lat_e7 = s->lat_deg_e7; n->previous_lon_e7 = s->lon_deg_e7;
  n->have_candidate = 0U; n->force_recovery = 0U;
  n->diag.accept_count++;
  n->diag.reason = recovery ? GNSS_REASON_RESET : GNSS_REASON_NONE;
  if (recovery) {
    record_reset(n, out, &after, s->t_rx_us, recovery);
    n->normal_updates = 0U; n->accepted_streak = 1U;
    n->acquisition_start_us = s->t_rx_us;
    n->diag.velocity_source = 0U; n->velocity_evidence_us = 0U;
    /* 未知速度先验不计为GPS速度测量，也不刷新最近速度融合时间。 */
  } else {
    if (n->accepted_streak < UINT32_MAX) n->accepted_streak++;
    if (n->normal_updates < UINT32_MAX) n->normal_updates++;
  }
  if (measured_v) {
    int vr = recovery ? UG_EKF_GPS_ACCEPTED : UgEkf_PushGpsVelocity(s->vel_n_mps,
        s->vel_e_mps, n->diag.vel_r_var, 0, n->diag.vel_r_var, epoch, s->t_rx_us);
    UgEkf_GetOutput(&after);
    if (!recovery && (vr == UG_EKF_GPS_ACCEPTED || vr == UG_EKF_GPS_REJECTED) &&
        isfinite(after.nis_gps_velocity)) {
      n->diag.vel_nis = after.nis_gps_velocity; n->diag.flags |= GNSS_FLAG_VELOCITY_INNOVATION;
    }
    if (vr == UG_EKF_GPS_ACCEPTED) {
      n->diag.vel_accept_count++; n->have_velocity = 1U;
      n->last_velocity_us = n->velocity_evidence_us = s->t_rx_us;
      n->diag.velocity_source = 1U;
    } else {
      n->diag.vel_reject_count++;
      n->diag.reason = vr == UG_EKF_GPS_REJECTED ? GNSS_REASON_VELOCITY_NIS : GNSS_REASON_NUMERIC;
    }
  }
  /* 原始位置通过Ppv学习速度，不将相同位置差分重复包装成独立速度测量。 */
  if (n->normal_updates >= 2U && s->t_rx_us - n->acquisition_start_us >=
      (uint64_t)n->config.velocity_confirm_ms * 1000ULL &&
      velocity_sane(n, &after) && velocity_cov_small(n, &after)) {
    n->velocity_evidence_us = s->t_rx_us;
    if (!measured_v) n->diag.velocity_source = 2U;
  }
  refresh(n, now, imu, &after);
}
