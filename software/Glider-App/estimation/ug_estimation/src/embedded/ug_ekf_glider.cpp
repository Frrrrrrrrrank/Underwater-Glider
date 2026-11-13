// ug_estimation/embedded/ug_ekf_glider.cpp
//
// ug_ekf_glider.h 的实现：C++ 封装 ug_ekf::Eskf，导出 C 接口。
// 无堆分配（静态实例 + 固定对齐缓冲）、无异常、无 iostream。

#include "ug_estimation/embedded/ug_ekf_glider.h"
#include "ug_estimation/core/eskf.h"

#include <cmath>
#include <cstring>

using namespace ug_ekf;

namespace {

//   静态状态（零堆分配） 
Eskf       g_ekf;
UgEkfConfig g_cfg;
bool       g_initialized = false;
bool       g_aligned     = false;
double     g_t0_s        = -1.0;
uint64_t   g_last_imu_us = 0, g_last_gps_epoch = 0;
bool       g_have_imu_time = false, g_have_gps_epoch = false;
int        g_gps_last_result = UG_EKF_GPS_NOT_READY;
uint64_t   g_last_gps_velocity_epoch = 0;
bool       g_have_gps_velocity_epoch = false;
int        g_gps_velocity_last_result = UG_EKF_GPS_NOT_READY;
// 历元消费结果与最近一次API诊断分开：前置NOT_READY不能抹掉NIS拒绝后的恢复资格。
int        g_gps_epoch_result = UG_EKF_GPS_NOT_READY;
int        g_gps_velocity_epoch_result = UG_EKF_GPS_NOT_READY;
double     g_align_window_t0_s = -1.0;
Vec3       g_last_gyro_frd = Vec3::Zero();
Mat3       g_mountR        = Mat3::Identity();

// 增量对齐，累加和 + 计数（O(1) RAM，任意采样率/时长不溢出）
Vec3 g_acc_sum = Vec3::Zero(), g_gyro_sum = Vec3::Zero(), g_mag_sum = Vec3::Zero();
int  g_n_imu = 0, g_n_mag = 0;

// 诊断 / 状态机
UgEkfStatus g_status = UG_EKF_UNINIT;
bool        g_still  = false;
double      g_last_t_s = -1.0;            // 最近喂入的 IMU 时间戳 (s)
uint32_t    g_align_reset_count = 0;      // 对齐中途晃动丢弃窗口次数
int         g_nonstill_streak = 0;        // 当前连续非静止样本数（容忍短暂抖动）

// 健康判据常量
constexpr float kFaultTiltVarSum = 4.0f;  // roll/pitch 协方差和门限 (rad²)，超过判 FAULT
constexpr float kDtMaxS          = 0.1f;  // predict dt 上限 (s)，与 buildParams 一致
constexpr int   kDefaultNonstillReset = 5;// 连续非静止样本数达此值才丢弃对齐窗口

inline double us_to_s(uint64_t t_us) { return double(t_us) * 1e-6; }

bool recentGyroForVelocity(uint64_t t_us) {
  if (!g_have_imu_time || !g_last_gyro_frd.allFinite()) return false;
  const uint64_t age = t_us >= g_last_imu_us ? t_us - g_last_imu_us : g_last_imu_us - t_us;
  const uint32_t limit = g_cfg.gps_velocity_gyro_max_age_us != 0 ?
      g_cfg.gps_velocity_gyro_max_age_us : 250000u;
  return age <= limit;
}

bool canResetEpoch(bool have_time, uint64_t previous, int result, uint64_t candidate) {
  return !have_time || candidate > previous ||
      (candidate == previous && result == UG_EKF_GPS_REJECTED);
}

inline Vec3 toFrd(float x, float y, float z) {
  return g_mountR * Vec3(Scalar(x), Scalar(y), Scalar(z));
}

// 取默认或配置值（cfg 字段为 0 时回退默认）
inline Scalar pick(float v, double def) {
  return Scalar(v != 0.0f ? double(v) : def);
}
// 重置对齐累加器,如果中途晃动，就清空 IMU/mag 对齐累加，重新等静止窗口
void resetAlignAccum() {
  g_acc_sum = Vec3::Zero();
  g_gyro_sum = Vec3::Zero();
  g_mag_sum = Vec3::Zero();
  g_n_imu = 0;
  g_n_mag = 0;
  g_align_window_t0_s = -1.0;
}
//判断是否静态的阈值
bool isStillForAlign(const Vec3& gyro, const Vec3& acc) {
  const Scalar g = pick(g_cfg.gravity, 9.81);
  const Scalar gate_a = pick(g_cfg.align_gate_acc, 0.6);
  const Scalar gate_w = pick(g_cfg.align_gate_gyro, 0.05);
  return std::abs(acc.norm() - g) < gate_a && gyro.norm() < gate_w;
}

InitParams buildParams() {
  InitParams p{};
  p.g = pick(g_cfg.gravity, 9.81);
  p.mag_declination_rad = Scalar(g_cfg.mag_declination_rad);   // 0 合法
  p.init_yaw_ned_rad    = Scalar(g_cfg.init_yaw_ned_rad);      // 0 合法
  p.r_pressure_FRD = Vec3(Scalar(g_cfg.r_pressure_frd[0]),
                          Scalar(g_cfg.r_pressure_frd[1]),
                          Scalar(g_cfg.r_pressure_frd[2]));
  p.dt_max = Scalar(kDtMaxS);
  p.nis_gate_depth = pick(g_cfg.nis_gate_depth, 6.635);
  p.nis_gate_tilt  = pick(g_cfg.nis_gate_tilt, 11.345);
  p.nis_gate_mag   = pick(g_cfg.nis_gate_mag, 6.635);
  p.depth_reset_streak = pick(g_cfg.depth_reset_streak, 10.0);
  p.r_gps_FRD = Vec3(Scalar(g_cfg.r_gps_frd[0]), Scalar(g_cfg.r_gps_frd[1]),
                     Scalar(g_cfg.r_gps_frd[2]));
  p.nis_gate_gps = pick(g_cfg.nis_gate_gps, 9.21034);
  p.gps_reset_velocity_var = pick(g_cfg.gps_reset_velocity_var, 4.0);
  p.nis_gate_gps_velocity = pick(g_cfg.nis_gate_gps_velocity, 9.21034);
  p.gps_velocity_gyro_var = pick(g_cfg.gps_velocity_gyro_var, 1e-4);
  p.noise.sigma_g  = pick(g_cfg.sigma_g, 1e-3);
  p.noise.sigma_a  = pick(g_cfg.sigma_a, 1e-2);
  p.noise.sigma_bg = pick(g_cfg.sigma_bg, 1e-5);
  p.noise.sigma_ba = pick(g_cfg.sigma_ba, 1e-4);
  p.p0_pos = pick(g_cfg.p0_pos, 100.0);
  p.p0_vel = pick(g_cfg.p0_vel, 0.01);
  p.p0_att = pick(g_cfg.p0_att, 0.01);
  p.p0_bg  = pick(g_cfg.p0_bg, 1e-4);
  p.p0_ba  = pick(g_cfg.p0_ba, 1e-3);
  return p;
}

Mat15 buildP0(const InitParams& p) {
  Mat15 P = Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}

void finishAlign() {
  InitParams p = buildParams();
  g_ekf.initialize(p, State{}, buildP0(p));
  if (g_n_imu >= 10) {
    const Vec3 acc_mean  = g_acc_sum  / Scalar(g_n_imu);
    const Vec3 gyro_mean = g_gyro_sum / Scalar(g_n_imu);
    const Vec3 mag_mean  = (g_n_mag >= 10) ? Vec3(g_mag_sum / Scalar(g_n_mag)) : Vec3::Zero();
    g_ekf.staticAlignFromMeans(acc_mean, gyro_mean,
                               (g_n_mag >= 10) ? &mag_mean : nullptr);
  }
  g_aligned = true;
  g_status  = UG_EKF_RUNNING;
}

}  // namespace

extern "C" {
//默认参数，后面补充修改
void UgEkf_DefaultConfig(UgEkfConfig* cfg) {
  if (!cfg) return;
  std::memset(cfg, 0, sizeof(*cfg));
  cfg->gravity = 9.81f;
  cfg->p_atm_pa = 101325.0f;
  cfg->water_rho = 1028.0f;
  cfg->mount_R[0] = 1; cfg->mount_R[4] = 1; cfg->mount_R[8] = 1;  // 单位阵
  cfg->r_pressure_frd[2] = -0.1f;
  cfg->static_align_seconds = 8.0f;
  cfg->static_align_delay_seconds = 1.0f;
  cfg->align_gate_acc = 0.6f;
  cfg->align_gate_gyro = 0.05f;
  cfg->align_nonstill_reset = float(kDefaultNonstillReset);
  cfg->depth_var = 0.01f;
  cfg->depth_reset_streak = 10.0f;
  cfg->tilt_var = 0.25f;
  cfg->mag_yaw_var = 0.01f;
  cfg->nis_gate_gps = 9.21034f;
  cfg->gps_reset_velocity_var = 4.0f;
  cfg->nis_gate_gps_velocity = 9.21034f;
  cfg->gps_velocity_gyro_var = 1e-4f;
  cfg->gps_velocity_gyro_max_age_us = 250000u;
  cfg->tilt_gate_acc = 0.5f;
  cfg->tilt_gate_gyro = 0.05f;
}

void UgEkf_Init(const UgEkfConfig* cfg) {
  if (cfg) g_cfg = *cfg;
  else UgEkf_DefaultConfig(&g_cfg);

  // mount_R 全 0 → 单位阵
  bool allzero = true;
  for (int i = 0; i < 9; ++i) if (g_cfg.mount_R[i] != 0.0f) { allzero = false; break; }
  if (allzero) g_mountR = Mat3::Identity();
  else {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        g_mountR(r, c) = Scalar(g_cfg.mount_R[r * 3 + c]);
  }

  g_aligned = false; g_t0_s = -1.0;
  g_have_imu_time = false; g_have_gps_epoch = false;
  g_last_imu_us = 0; g_last_gps_epoch = 0;
  g_gps_last_result = UG_EKF_GPS_NOT_READY;
  g_gps_velocity_last_result = UG_EKF_GPS_NOT_READY;
  g_gps_epoch_result = g_gps_velocity_epoch_result = UG_EKF_GPS_NOT_READY;
  g_have_gps_velocity_epoch = false;
  g_last_gps_velocity_epoch = 0;
  resetAlignAccum();
  g_last_gyro_frd = Vec3::Zero();
  g_status = UG_EKF_WAIT_BOOT_DELAY;
  g_still  = false;
  g_last_t_s = -1.0;
  g_align_reset_count = 0;
  g_nonstill_streak = 0;

  InitParams p = buildParams();
  g_ekf.initialize(p, State{}, buildP0(p));
  g_initialized = true;
}

void UgEkf_PushImu(float gx, float gy, float gz,
                   float ax, float ay, float az, uint64_t t_us) {
  if (!g_initialized) return;
  if (g_have_imu_time && t_us <= g_last_imu_us) return;
  g_last_imu_us = t_us;
  g_have_imu_time = true;
  const double t = us_to_s(t_us);
  g_last_t_s = t;
  const Vec3 gyro = toFrd(gx, gy, gz);
  const Vec3 acc  = toFrd(ax, ay, az);

  if (!g_aligned) {
    if (g_t0_s < 0) g_t0_s = t;
    const double delay_s = double(pick(g_cfg.static_align_delay_seconds, 1.0));
    g_still = isStillForAlign(gyro, acc);
    if ((t - g_t0_s) < delay_s) { g_status = UG_EKF_WAIT_BOOT_DELAY; return; }

    if (!g_still) {
      // 容忍短暂抖动：仅当连续非静止样本数达到门限才丢弃已收集窗口
      const int reset_n = (g_cfg.align_nonstill_reset > 0.0f)
                            ? int(g_cfg.align_nonstill_reset) : kDefaultNonstillReset;
      if (g_align_window_t0_s >= 0) {
        if (++g_nonstill_streak >= reset_n) {
          ++g_align_reset_count;          // 持续晃动，丢弃整段对齐窗口
          resetAlignAccum();
          g_nonstill_streak = 0;
          g_status = UG_EKF_WAIT_STILL;
        } else {
          g_status = UG_EKF_ALIGNING;     // 短暂抖动：保留窗口，仅跳过该帧（不累加）
        }
      } else {
        g_nonstill_streak = 0;            // 窗口尚未开始，继续等静止
        g_status = UG_EKF_WAIT_STILL;
      }
      return;
    }

    // still 样本：清零连续非静止计数，正常累加
    g_nonstill_streak = 0;
    if (g_align_window_t0_s < 0) g_align_window_t0_s = t;
    g_acc_sum += acc; g_gyro_sum += gyro; ++g_n_imu;
    g_status = UG_EKF_ALIGNING;

    const double align_s = double(pick(g_cfg.static_align_seconds, 8.0));
    if ((t - g_align_window_t0_s) >= align_s && g_n_imu >= 10) finishAlign();
    return;
  }

  g_ekf.predictImu(t, gyro, acc);
  g_last_gyro_frd = gyro;

  const Scalar g = pick(g_cfg.gravity, 9.81);
  const Scalar gate_a = pick(g_cfg.tilt_gate_acc, 0.5);
  const Scalar gate_w = pick(g_cfg.tilt_gate_gyro, 0.05);
  g_still = (std::abs(acc.norm() - g) < gate_a && gyro.norm() < gate_w);
  if (g_still) {
    g_ekf.updateAccelTilt(Scalar(t), acc, pick(g_cfg.tilt_var, 0.25));
  }
}

void UgEkf_PushMag(float mx, float my, float mz, uint64_t t_us) {
  if (!g_initialized) return;
  const Vec3 mag = toFrd(mx, my, mz);
  if (!g_aligned) {
    if (g_align_window_t0_s >= 0) {
      g_mag_sum += mag; ++g_n_mag;
    }
    return;
  }
  g_ekf.updateMag(Scalar(us_to_s(t_us)), mag, pick(g_cfg.mag_yaw_var, 0.01));
}

void UgEkf_PushDepth(float depth_m, uint64_t t_us) {
  if (!g_initialized || !g_aligned) return;
  g_ekf.updateDepth(Scalar(us_to_s(t_us)), Scalar(depth_m), pick(g_cfg.depth_var, 0.01));
}

void UgEkf_PushPressureBar(float pressure_bar, uint64_t t_us) {
  const Scalar p_atm = pick(g_cfg.p_atm_pa, 101325.0);
  const Scalar rho   = pick(g_cfg.water_rho, 1028.0);
  const Scalar g     = pick(g_cfg.gravity, 9.81);
  const Scalar depth = (Scalar(pressure_bar) * Scalar(1e5) - p_atm) / (rho * g);
  UgEkf_PushDepth(float(depth), t_us);
}

int UgEkf_PushGpsPosition(float north, float east, float var_n, float cov_ne,
                         float var_e, uint64_t epoch_id, uint64_t t_us) {
  UgEkfOutput current;
  UgEkf_GetOutput(&current);
  if (!g_initialized || !g_aligned || !current.valid_for_control) {
    return g_gps_last_result = UG_EKF_GPS_NOT_READY;
  }
  if (g_have_gps_epoch && epoch_id <= g_last_gps_epoch) {
    return g_gps_last_result = UG_EKF_GPS_INVALID;
  }
  g_last_gps_epoch = epoch_id;
  g_have_gps_epoch = true;
  Mat2 R;
  R << Scalar(var_n), Scalar(cov_ne), Scalar(cov_ne), Scalar(var_e);
  g_ekf.updateGps(us_to_s(t_us), Vec2(Scalar(north), Scalar(east)), R);
  return g_gps_last_result = g_gps_epoch_result = g_ekf.diag().gps_last_result;
}

int UgEkf_ResetHorizontalPosition(float north, float east, float var_n, float cov_ne,
                                 float var_e, uint64_t epoch_id, uint64_t t_us) {
  (void)t_us;  // 此操作不推进IMU时间，也不借接收时刻代替测量身份。
  UgEkfOutput current;
  UgEkf_GetOutput(&current);
  if (!g_initialized || !g_aligned || !current.valid_for_control) {
    return g_gps_last_result = UG_EKF_GPS_NOT_READY;
  }
  // 本次正常更新被 NIS 拒绝时，可用同一历元进入调用方明确选择的重锚定。
  if (g_have_gps_epoch && (epoch_id < g_last_gps_epoch ||
      (epoch_id == g_last_gps_epoch && g_gps_epoch_result != UG_EKF_GPS_REJECTED))) {
    return g_gps_last_result = UG_EKF_GPS_INVALID;
  }
  g_last_gps_epoch = epoch_id;
  g_have_gps_epoch = true;
  Mat2 R;
  R << Scalar(var_n), Scalar(cov_ne), Scalar(cov_ne), Scalar(var_e);
  g_ekf.resetHorizontalPosition(Vec2(Scalar(north), Scalar(east)), R);
  return g_gps_last_result = g_gps_epoch_result = g_ekf.diag().gps_last_result;
}

int UgEkf_PushGpsVelocity(float vn, float ve, float var_n, float cov_ne,
                         float var_e, uint64_t epoch_id, uint64_t t_us) {
  UgEkfOutput current;
  UgEkf_GetOutput(&current);
  if (!g_initialized || !g_aligned || !current.valid_for_control || !recentGyroForVelocity(t_us)) {
    return g_gps_velocity_last_result = UG_EKF_GPS_NOT_READY;
  }
  if (g_have_gps_velocity_epoch && epoch_id <= g_last_gps_velocity_epoch) {
    return g_gps_velocity_last_result = UG_EKF_GPS_INVALID;
  }
  g_last_gps_velocity_epoch = epoch_id;
  g_have_gps_velocity_epoch = true;
  Mat2 R;
  R << Scalar(var_n), Scalar(cov_ne), Scalar(cov_ne), Scalar(var_e);
  g_ekf.updateGpsVelocity(us_to_s(t_us), Vec2(Scalar(vn), Scalar(ve)), R, g_last_gyro_frd);
  return g_gps_velocity_last_result = g_gps_velocity_epoch_result = g_ekf.diag().gps_velocity_last_result;
}

int UgEkf_ReanchorHorizontal(float n, float e, float var_n, float cov_ne, float var_e,
                            float vn, float ve, float vvar_n, float vcov_ne, float vvar_e,
                            uint32_t velocity_measured, uint64_t epoch_id, uint64_t t_us) {
  UgEkfOutput current;
  UgEkf_GetOutput(&current);
  if (!current.horizontal_recovery_ready ||
      (velocity_measured != 0 && !recentGyroForVelocity(t_us))) {
    return g_gps_last_result = UG_EKF_GPS_NOT_READY;
  }
  if (velocity_measured > 1u ||
      !canResetEpoch(g_have_gps_epoch, g_last_gps_epoch, g_gps_epoch_result, epoch_id) ||
      !canResetEpoch(g_have_gps_velocity_epoch, g_last_gps_velocity_epoch,
                      g_gps_velocity_epoch_result, epoch_id)) {
    return g_gps_last_result = UG_EKF_GPS_INVALID;
  }
  Mat2 positionR, velocityR;
  positionR << Scalar(var_n), Scalar(cov_ne), Scalar(cov_ne), Scalar(var_e);
  velocityR << Scalar(vvar_n), Scalar(vcov_ne), Scalar(vcov_ne), Scalar(vvar_e);
  const bool accepted = g_ekf.reanchorHorizontal(Vec2(Scalar(n), Scalar(e)), positionR,
      Vec2(Scalar(vn), Scalar(ve)), velocityR, velocity_measured != 0, g_last_gyro_frd);
  g_gps_last_result = g_ekf.diag().gps_last_result;
  if (accepted) {
    g_last_gps_epoch = g_last_gps_velocity_epoch = epoch_id;
    g_have_gps_epoch = g_have_gps_velocity_epoch = true;
    g_gps_velocity_last_result = g_ekf.diag().gps_velocity_last_result;
    g_gps_epoch_result = g_gps_last_result;
    g_gps_velocity_epoch_result = g_gps_velocity_last_result;
  }
  return g_gps_last_result;
}

void UgEkf_GetOutput(UgEkfOutput* out) {
  if (!out) return;
  std::memset(out, 0, sizeof(*out));
  out->gps_last_result = g_gps_last_result;
  out->gps_velocity_last_result = g_gps_velocity_last_result;
  if (!g_initialized) { out->status = uint8_t(UG_EKF_UNINIT); return; }

  const State& x = g_ekf.state();
  const Mat3 R = x.q_NB.toRotationMatrix();
  out->pitch = float(std::asin(std::max(Scalar(-1), std::min(Scalar(1), -R(2, 0)))));
  out->roll  = float(std::atan2(R(2, 1), R(2, 2)));
  out->yaw   = float(std::atan2(R(1, 0), R(0, 0)));
  out->pN = float(x.p_NED.x()); out->pE = float(x.p_NED.y());
  out->depth_m = float(x.p_NED.z());
  out->vN = float(x.v_NED.x()); out->vE = float(x.v_NED.y()); out->vD = float(x.v_NED.z());
  for (int i = 0; i < 3; ++i) { out->b_g[i] = float(x.b_g[i]); out->b_a[i] = float(x.b_a[i]); }

  // 协方差迹（发散指示），P_trace_pos≈水平发散（D 项因深度可观很小）
  const Mat15& P = g_ekf.covariance();
  out->P_trace_pos = float(P.block<3, 3>(0, 0).trace());
  out->P_trace_vel = float(P.block<3, 3>(3, 3).trace());
  out->P_trace_att = float(P.block<3, 3>(6, 6).trace());

  const Diagnostics& d = g_ekf.diag();
  out->nis_depth = float(d.last_nis_depth);
  out->nis_tilt  = float(d.last_nis_tilt);
  out->nis_mag   = float(d.last_nis_mag);
  out->nis_gps = float(d.last_nis_gps);
  out->accept_gps = d.accept_gps;
  out->reject_gps = d.reject_gps;
  out->gps_reset_count = d.gps_reset_count;
  out->gps_innov_n = float(d.gps_innov_n);
  out->gps_innov_e = float(d.gps_innov_e);
  out->Pnn = float(P(0, 0)); out->Pne = float(P(0, 1)); out->Pee = float(P(1, 1));
  out->Pvn = float(P(3, 3)); out->Pvne = float(P(3, 4)); out->Pve = float(P(4, 4));
  out->nis_gps_velocity = float(d.last_nis_gps_velocity);
  out->accept_gps_velocity = d.accept_gps_velocity;
  out->reject_gps_velocity = d.reject_gps_velocity;
  out->gps_velocity_innov_n = float(d.gps_velocity_innov_n);
  out->gps_velocity_innov_e = float(d.gps_velocity_innov_e);
  out->horizontal_velocity_reset_count = d.horizontal_velocity_reset_count;
  out->horizontal_velocity_source = d.horizontal_velocity_source;
  out->gps_last_result = g_gps_last_result;
  out->accept_depth = d.accept_depth; out->reject_depth = d.reject_depth;
  out->accept_tilt  = d.accept_tilt;  out->reject_tilt  = d.reject_tilt;
  out->accept_mag   = d.accept_mag;   out->reject_mag   = d.reject_mag;
  out->reset_count  = d.reset_count;
  out->depth_reject_streak = d.depth_reject_streak;
  out->depth_reset_count   = d.depth_reset_count;
  out->dt_last = float(d.dt_last);
  out->t_last  = float(g_last_t_s);

  // 对齐诊断
  out->aligned            = g_aligned ? 1 : 0;
  out->still              = g_still ? 1 : 0;
  out->align_sample_count = uint32_t(g_n_imu);
  out->align_reset_count  = g_align_reset_count;
  out->align_elapsed_s    = (g_align_window_t0_s >= 0 && g_last_t_s >= 0)
                              ? float(g_last_t_s - g_align_window_t0_s) : 0.0f;

  // finiteness（含速度）→ finite 标志 + FAULT 判据
  const float chk[] = { out->roll, out->pitch, out->yaw, out->pN, out->pE,
                        out->depth_m, out->vN, out->vE, out->vD,
                        out->b_g[0], out->b_g[1], out->b_g[2],
                        out->b_a[0], out->b_a[1], out->b_a[2] };
  bool finite = true;
  for (float v : chk) { if (!std::isfinite(v)) { finite = false; break; } }
  out->finite = finite ? 1 : 0;

  // 状态机 + 健康判定
  UgEkfStatus st = g_status;
  bool healthy = false;
  if (st == UG_EKF_RUNNING) {
    // 仅以"数值失效"判 FAULT：NaN/Inf，或 roll/pitch 协方差爆掉。
    // 缺少水平位置或航向观测时相应协方差会增长；水平可用性由外层独立判定。
    const float tilt_var_sum = float(P(6, 6) + P(7, 7));
    if (!finite || !P.allFinite() || tilt_var_sum > kFaultTiltVarSum) {
      st = UG_EKF_FAULT;
    } else {
      // dt 正常 = 最近 predict 步长在 (0, dt_max]。注：数据中断时 dt_last 不更新，
      // 上位机须另用 (now - t_last) 判断喂数是否停止。
      healthy = (out->dt_last > 0.0f) && (out->dt_last <= kDtMaxS);
    }
  }
  out->status            = uint8_t(st);
  out->healthy           = healthy ? 1 : 0;
  out->valid_for_control = (st == UG_EKF_RUNNING && healthy) ? 1 : 0;
  // 整体FAULT可能仅来自水平NaN；仅保留块健康时放行水平恢复，外层另查实时年龄。
  const float tilt_var_sum = float(P(6, 6) + P(7, 7));
  out->horizontal_recovery_ready = (g_aligned && g_status == UG_EKF_RUNNING &&
      out->dt_last > 0.0f && out->dt_last <= kDtMaxS &&
      std::isfinite(tilt_var_sum) && tilt_var_sum <= kFaultTiltVarSum &&
      g_ekf.horizontalRecoveryReady()) ? 1 : 0;
}

}  // extern "C"
