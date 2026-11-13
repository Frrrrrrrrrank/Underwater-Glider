// ug_estimation/src/core/eskf.cpp
//
// IMU 预测、压力深度、倾角/航向与 GNSS 二维位置更新；DVL 接口待实现。
// 平台无关；无 ROS；无堆分配；无异常；无 RTTI。

#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/quaternion_utils.h"
#include "ug_estimation/core/imu_propagator.h"
#include "ug_estimation/core/measurements/depth.h"
#include "ug_estimation/core/measurements/accel_tilt.h"
#include "ug_estimation/core/measurements/mag.h"
#include "ug_estimation/core/measurements/gps.h"
#include <Eigen/Cholesky>

#include <cmath>
#include <limits>

namespace ug_ekf {
namespace {
// 测量方差必须有限、对称且正定；不能把坏 R 当成高可信定位。
bool validGpsCovariance(const Mat2& R) {
  if (!R.allFinite()) return false;
  const Scalar scale = std::max(Scalar(1), R.cwiseAbs().maxCoeff());
  if (std::abs(R(0, 1) - R(1, 0)) > Scalar(1e-6) * scale) return false;
  Eigen::LLT<Mat2> llt(R);
  return llt.info() == Eigen::Success;
}

bool finiteState(const State& x) {
  return x.p_NED.allFinite() && x.v_NED.allFinite() &&
         x.q_NB.coeffs().allFinite() && x.q_NB.squaredNorm() > Scalar(1e-12) &&
         x.b_g.allFinite() && x.b_a.allFinite();
}

using Mat11 = Eigen::Matrix<Scalar, 11, 11>;
// 留下 pD/vD/姿态误差/bg/ba；不要读取待丢弃的水平行列，0*NaN也会污染结果。
constexpr int kRetained[11] = {2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
constexpr int kHorizontal[4] = {0, 1, 3, 4};
bool retainedCovariance(const Mat15& P, Mat11& retained) {
  for (int i = 0; i < 11; ++i)
    for (int j = 0; j < 11; ++j) retained(i, j) = P(kRetained[i], kRetained[j]);
  if (!retained.allFinite()) return false;
  for (int i = 0; i < 11; ++i) {
    for (int j = i + 1; j < 11; ++j) {
      const Scalar scale = std::max(Scalar(1), std::max(std::abs(retained(i, j)),
                                                       std::abs(retained(j, i))));
      if (std::abs(retained(i, j) - retained(j, i)) > Scalar(1e-5) * scale) return false;
    }
  }
  Eigen::LLT<Mat11> llt(retained);
  return llt.info() == Eigen::Success;
}

// 罕见的水平P损坏分支：保留块动力学不依赖水平p/v，不让0*NaN扩散到姿态和深度。
void propagateRetainedCovariance(Mat15& P, const Mat15& F, const Mat15& Q) {
  Mat11 retained, transition, noise;
  for (int i = 0; i < 11; ++i) {
    for (int j = 0; j < 11; ++j) {
      retained(i, j) = P(kRetained[i], kRetained[j]);
      transition(i, j) = F(kRetained[i], kRetained[j]);
      noise(i, j) = Q(kRetained[i], kRetained[j]);
    }
  }
  const Mat11 next = transition * retained * transition.transpose() + noise;
  for (int i = 0; i < 11; ++i)
    for (int j = 0; j < 11; ++j) P(kRetained[i], kRetained[j]) = next(i, j);
}
}  // namespace


Eskf::Eskf() = default;

void Eskf::initialize(const InitParams& params, const State& x0, const Mat15& P0) {
  params_ = params;
  x_ = x0;
  P_ = P0;
  diag_ = Diagnostics{};
  t_prev_ = -1.0;
  initialized_ = true;
  enforceSymmetry();
}

void Eskf::predictImu(double t, const Vec3& gyro_FRD, const Vec3& accel_FRD) {
  if (!initialized_ || !std::isfinite(t)) return;

  // 首帧，仅记录时间戳，不做传播
  if (t_prev_ < Scalar(0)) {
    t_prev_ = t;
    diag_.t_last = t;
    diag_.dt_last = Scalar(0);
    return;
  }

  const Scalar dt = Scalar(t - t_prev_);
  if (dt <= Scalar(0)) {
    // 时间戳倒退或重复，跳过
    return;
  }
  if (dt > params_.dt_max) {
    // 跳帧过大：重置 t_prev，不传播
    diag_.reset_count++;
    t_prev_ = t;
    diag_.t_last = t;
    diag_.dt_last = dt;
    return;
  }

  const bool isolate_horizontal = !P_.allFinite();
  if (isolate_horizontal && !horizontalRecoveryReady()) {
    // 保留块也已失效，停止传播；交由外层独立故障处理，不重启整个滤波器。
    t_prev_ = t;
    diag_.t_last = t;
    diag_.dt_last = Scalar(0);
    return;
  }
  // 1) 用传播前的状态构造 F/Q（线性化点）
  Mat15 F;
  Mat15 Q;
  BuildErrorJacobians(x_, gyro_FRD, accel_FRD, params_.noise, dt, F, Q);

  // 2) 名义传播
  PropagateNominal(x_, gyro_FRD, accel_FRD, params_.g, dt);

  // 3) 协方差传播 P = F P F^T + Q，动力学传播
  if (isolate_horizontal) propagateRetainedCovariance(P_, F, Q);
  else P_ = F * P_ * F.transpose() + Q;
  enforceSymmetry();

  t_prev_ = t;
  diag_.t_last = t;
  diag_.dt_last = dt;
}

bool Eskf::updateDepth(Scalar t, Scalar depth_m, Scalar R) {
  if (!initialized_) return false;
  (void)t;  // 深度测量时间戳由 caller 用于诊断；预测协方差已由 predict 推到当前

  RowVec15 H;
  BuildDepthH(x_, params_.r_pressure_FRD, H);

  const Scalar z_pred = PredictDepth(x_, params_.r_pressure_FRD);
  const Scalar y = depth_m - z_pred;              // innovation

  // S = H P H^T + R (标量)
  const Scalar S = (H * P_ * H.transpose())(0, 0) + R;
  if (S <= Scalar(0)) {
    diag_.reject_depth++;
    diag_.depth_reject_streak++;
    return false;
  }

  const Scalar nis = y * y / S;
  diag_.last_nis_depth = nis;

  if (nis > params_.nis_gate_depth) {
    diag_.reject_depth++;
    diag_.depth_reject_streak++;
    // 失锁恢复：连续拒绝次数达门限，判定垂直通道预测量发散，把深度对齐到测量、垂直速度清零、
    // 解相关并把深度/垂直速度方差恢复到初值（PX4 resetHeight 思路），使后续量测重新被信任
    if (params_.depth_reset_streak > Scalar(0) &&
        Scalar(diag_.depth_reject_streak) >= params_.depth_reset_streak) {
      const Scalar pressure_lever_arm_D =
          (x_.q_NB.toRotationMatrix() * params_.r_pressure_FRD).z();
      x_.p_NED.z() = depth_m - pressure_lever_arm_D;
      x_.v_NED.z() = Scalar(0);
      P_.row(kIdxDeltaP + 2).setZero();  P_.col(kIdxDeltaP + 2).setZero();
      P_.row(kIdxDeltaV + 2).setZero();  P_.col(kIdxDeltaV + 2).setZero();
      P_(kIdxDeltaP + 2, kIdxDeltaP + 2) = params_.p0_pos;
      P_(kIdxDeltaV + 2, kIdxDeltaV + 2) = params_.p0_vel;
      enforceSymmetry();
      diag_.depth_reset_count++;
      diag_.depth_reject_streak = 0;
    }
    return false;
  }

  // Kalman gain K = P H^T / S (15x1)
  const Eigen::Matrix<Scalar, 15, 1> K = (P_ * H.transpose()) / S;

  const Vec15 dx = K * y;
  const Mat15 IKH = Mat15::Identity() - K * H;
  const Mat15 joseph = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  if (!applyCorrection(dx, joseph)) {
    diag_.reject_depth++;
    return false;
  }

  diag_.accept_depth++;
  diag_.depth_reject_streak = 0;
  return true;
}
//加速度计作为观测量，在机体近似静止情况下修正delt_theta
bool Eskf::updateAccelTilt(Scalar /*t*/, const Vec3& accel_FRD, Scalar R_tilt) {
  if (!initialized_) return false;

  Mat3x15 H;
  BuildAccelTiltH(x_, params_.g, H);

  const Vec3 h = PredictAccelSpecificForce(x_, params_.g);
  const Vec3 y = accel_FRD - h;                       // innovation (3x1)

  // S = H P H^T + R·I3
  Mat3 S = H * P_ * H.transpose();
  S.diagonal().array() += R_tilt;

  // NIS = y^T S^-1 y
  const Mat3 S_inv = S.inverse();
  const Scalar nis = (y.transpose() * S_inv * y)(0, 0);
  diag_.last_nis_tilt = nis;

  if (!(nis == nis) || nis > params_.nis_gate_tilt) {  // NaN 或超门限
    diag_.reject_tilt++;
    return false;
  }

  // K = P H^T S^-1  (15x3)
  const Eigen::Matrix<Scalar, 15, 3> K = P_ * H.transpose() * S_inv;

  const Vec15 dx = K * y;
  const Mat15 IKH = Mat15::Identity() - K * H;
  const Mat15 joseph = IKH * P_ * IKH.transpose() + (K * R_tilt) * K.transpose();
  if (!applyCorrection(dx, joseph)) {
    diag_.reject_tilt++;
    return false;
  }

  diag_.accept_tilt++;
  return true;
}

bool Eskf::updateMag(Scalar /*t*/, const Vec3& mag_FRD, Scalar R_yaw) {
  if (!initialized_) return false;
  if (mag_FRD.norm() < Scalar(1e-12)) { diag_.reject_mag++; return false; }

  RowVec15 H;
  BuildMagYawH(x_, H);

  const Scalar y = MagYawInnovation(x_, mag_FRD, params_.mag_declination_rad);

  // S = H P H^T + R (标量)
  const Scalar S = (H * P_ * H.transpose())(0, 0) + R_yaw;
  if (S <= Scalar(0)) { diag_.reject_mag++; return false; }

  const Scalar nis = y * y / S;
  diag_.last_nis_mag = nis;
  if (nis > params_.nis_gate_mag) { diag_.reject_mag++; return false; }

  const Eigen::Matrix<Scalar, 15, 1> K = (P_ * H.transpose()) / S;
  const Mat15 IKH = Mat15::Identity() - K * H;
  const Mat15 joseph = IKH * P_ * IKH.transpose() + K * R_yaw * K.transpose();
  if (!applyCorrection(K * y, joseph)) {
    diag_.reject_mag++;
    return false;
  }

  diag_.accept_mag++;
  return true;
}

bool Eskf::updateGps(double t, const Vec2& antennaNE, const Mat2& R) {
  if (!initialized_) { diag_.gps_last_result = -2; return false; }
  diag_.gps_last_result = -1;
  if (!std::isfinite(t) || !antennaNE.allFinite() || !validGpsCovariance(R) ||
      !params_.r_gps_FRD.allFinite() || !std::isfinite(params_.nis_gate_gps) ||
      params_.nis_gate_gps <= Scalar(0) || !finiteState(x_) || !P_.allFinite()) {
    ++diag_.reject_gps;
    return false;
  }
  Mat2x15 H;
  BuildGpsPositionH(x_, params_.r_gps_FRD, H);
  const Vec2 y = antennaNE - PredictGpsPosition(x_, params_.r_gps_FRD);
  diag_.gps_innov_n = y.x();
  diag_.gps_innov_e = y.y();
  const Mat2 S = H * P_ * H.transpose() + R;
  if (!S.allFinite()) { ++diag_.reject_gps; return false; }
  Eigen::LLT<Mat2> llt(S);
  if (llt.info() != Eigen::Success) { ++diag_.reject_gps; return false; }
  const Scalar nis = y.dot(llt.solve(y));
  diag_.last_nis_gps = nis;
  if (!std::isfinite(nis) || nis < Scalar(0)) { ++diag_.reject_gps; return false; }
  if (nis > params_.nis_gate_gps) {
    diag_.gps_last_result = 0;
    ++diag_.reject_gps;
    return false;
  }
  const Eigen::Matrix<Scalar, 15, 2> PHt = P_ * H.transpose();
  const Eigen::Matrix<Scalar, 15, 2> K = llt.solve(PHt.transpose()).transpose();
  const Mat15 IKH = Mat15::Identity() - K * H;
  const Mat15 joseph = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  if (!applyCorrection(K * y, joseph)) { ++diag_.reject_gps; return false; }
  ++diag_.accept_gps;
  diag_.gps_last_result = 1;
  return true;
}

bool Eskf::resetHorizontalPosition(const Vec2& antennaNE, const Mat2& R) {
  if (!initialized_) { diag_.gps_last_result = -2; return false; }
  diag_.gps_last_result = -1;
  if (!antennaNE.allFinite() || !validGpsCovariance(R) ||
      !params_.r_gps_FRD.allFinite() || !finiteState(x_) || !P_.allFinite() ||
      !std::isfinite(params_.gps_reset_velocity_var) ||
      params_.gps_reset_velocity_var < Scalar(0)) {
    ++diag_.reject_gps;
    return false;
  }
  const Mat3 R_NB = x_.q_NB.toRotationMatrix();
  const Vec2 innovation = antennaNE - PredictGpsPosition(x_, params_.r_gps_FRD);
  // 位置重置来自天线测量和当前姿态；保留杆臂引起的姿态交叉协方差。
  const Eigen::Matrix<Scalar, 2, 3> J =
      (R_NB * Hat(params_.r_gps_FRD)).topRows<2>();
  const Mat2x15 cross = J * P_.block<3, 15>(kIdxDeltaTh, 0);
  Mat15 nextP = P_;
  nextP.topRows<2>() = cross;
  nextP.leftCols<2>() = cross.transpose();
  nextP.block<2, 2>(0, 0) =
      J * P_.block<3, 3>(kIdxDeltaTh, kIdxDeltaTh) * J.transpose() + R;
  // 没有速度观测不能强置零；增加不确定度，让后续独立位置历元校正旧速度。
  nextP(kIdxDeltaV, kIdxDeltaV) += params_.gps_reset_velocity_var;
  nextP(kIdxDeltaV + 1, kIdxDeltaV + 1) += params_.gps_reset_velocity_var;
  State next = x_;
  next.p_NED.head<2>() = antennaNE - (R_NB * params_.r_gps_FRD).head<2>();
  if (!finiteState(next) || !nextP.allFinite()) { ++diag_.reject_gps; return false; }
  x_ = next;
  P_ = nextP;
  enforceSymmetry();
  diag_.gps_innov_n = innovation.x();
  diag_.gps_innov_e = innovation.y();
  diag_.last_nis_gps = Scalar(0); // 显式重置不伪装为一次正常 NIS 更新。
  ++diag_.gps_reset_count;
  diag_.gps_last_result = 1;
  return true;
}

bool Eskf::updateGpsVelocity(double t, const Vec2& antennaVelocityNE, const Mat2& R,
                             const Vec3& gyro_FRD) {
  if (!initialized_) { diag_.gps_velocity_last_result = -2; return false; }
  diag_.gps_velocity_last_result = -1;
  if (!std::isfinite(t) || !antennaVelocityNE.allFinite() || !validGpsCovariance(R) ||
      !gyro_FRD.allFinite() || !params_.r_gps_FRD.allFinite() ||
      !std::isfinite(params_.gps_velocity_gyro_var) || params_.gps_velocity_gyro_var < Scalar(0) ||
      !std::isfinite(params_.nis_gate_gps_velocity) || params_.nis_gate_gps_velocity <= Scalar(0) ||
      !finiteState(x_) || !P_.allFinite()) {
    ++diag_.reject_gps_velocity;
    return false;
  }
  Mat2x15 H;
  BuildGpsVelocityH(x_, params_.r_gps_FRD, gyro_FRD, H);
  const Mat2 effectiveR = R +
      GpsVelocityGyroCovariance(x_, params_.r_gps_FRD, params_.gps_velocity_gyro_var);
  const Vec2 y = antennaVelocityNE - PredictGpsVelocity(x_, params_.r_gps_FRD, gyro_FRD);
  diag_.gps_velocity_innov_n = y.x();
  diag_.gps_velocity_innov_e = y.y();
  const Mat2 S = H * P_ * H.transpose() + effectiveR;
  if (!S.allFinite()) { ++diag_.reject_gps_velocity; return false; }
  Eigen::LLT<Mat2> llt(S);
  if (llt.info() != Eigen::Success) { ++diag_.reject_gps_velocity; return false; }
  const Scalar nis = y.dot(llt.solve(y));
  diag_.last_nis_gps_velocity = nis;
  if (!std::isfinite(nis) || nis < Scalar(0)) { ++diag_.reject_gps_velocity; return false; }
  if (nis > params_.nis_gate_gps_velocity) {
    diag_.gps_velocity_last_result = 0;
    ++diag_.reject_gps_velocity;
    return false;
  }
  const Eigen::Matrix<Scalar, 15, 2> PHt = P_ * H.transpose();
  const Eigen::Matrix<Scalar, 15, 2> K = llt.solve(PHt.transpose()).transpose();
  const Mat15 IKH = Mat15::Identity() - K * H;
  const Mat15 joseph = IKH * P_ * IKH.transpose() + K * effectiveR * K.transpose();
  if (!applyCorrection(K * y, joseph)) { ++diag_.reject_gps_velocity; return false; }
  ++diag_.accept_gps_velocity;
  diag_.horizontal_velocity_source = 2;
  diag_.gps_velocity_last_result = 1;
  return true;
}

bool Eskf::horizontalRecoveryReady() const {
  if (!initialized_ || !std::isfinite(x_.p_NED.z()) || !std::isfinite(x_.v_NED.z()) ||
      !x_.q_NB.coeffs().allFinite() ||
      std::abs(x_.q_NB.squaredNorm() - Scalar(1)) > Scalar(1e-3) ||
      !x_.b_g.allFinite() || !x_.b_a.allFinite() || !params_.r_gps_FRD.allFinite()) return false;
  Mat11 retained;
  return retainedCovariance(P_, retained);
}

bool Eskf::reanchorHorizontal(const Vec2& antennaNE, const Mat2& positionR,
                               const Vec2& antennaVelocityNE, const Mat2& velocityR,
                               bool velocity_measured, const Vec3& gyro_FRD) {
  if (!initialized_) { diag_.gps_last_result = -2; return false; }
  diag_.gps_last_result = -1;
  if (!antennaNE.allFinite() || !validGpsCovariance(positionR) ||
      !validGpsCovariance(velocityR) || !horizontalRecoveryReady() ||
      (velocity_measured && (!antennaVelocityNE.allFinite() || !gyro_FRD.allFinite() ||
       !std::isfinite(params_.gps_velocity_gyro_var) || params_.gps_velocity_gyro_var < Scalar(0)))) {
    ++diag_.reject_gps;
    return false;
  }
  Mat11 retained;
  if (!retainedCovariance(P_, retained)) { ++diag_.reject_gps; return false; }
  // 新水平误差只来自保留状态和新观测/先验，彻底移除旧水平信息。
  Eigen::Matrix<Scalar, 4, 11> J = Eigen::Matrix<Scalar, 4, 11>::Zero();
  const Mat3 R_NB = x_.q_NB.toRotationMatrix();
  J.block<2, 3>(0, 2) = (R_NB * Hat(params_.r_gps_FRD)).topRows<2>();
  if (velocity_measured) {
    Mat2x15 H;
    BuildGpsVelocityH(x_, params_.r_gps_FRD, gyro_FRD, H);
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 11; ++j) J(i + 2, j) = -H(i, kRetained[j]);
  }
  const Eigen::Matrix<Scalar, 4, 11> cross = J * retained;
  Eigen::Matrix<Scalar, 4, 4> horizontal = cross * J.transpose();
  // 输入位置/速度接收机噪声按不相关建模；共同姿态/bias相关性由J保留。
  horizontal.topLeftCorner<2, 2>() += positionR;
  horizontal.bottomRightCorner<2, 2>() += velocityR;
  if (velocity_measured) horizontal.bottomRightCorner<2, 2>() +=
      GpsVelocityGyroCovariance(x_, params_.r_gps_FRD, params_.gps_velocity_gyro_var);
  Mat15 nextP = Mat15::Zero();
  for (int i = 0; i < 11; ++i)
    for (int j = 0; j < 11; ++j) nextP(kRetained[i], kRetained[j]) = retained(i, j);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 11; ++j) {
      nextP(kHorizontal[i], kRetained[j]) = cross(i, j);
      nextP(kRetained[j], kHorizontal[i]) = cross(i, j);
    }
    for (int j = 0; j < 4; ++j) nextP(kHorizontal[i], kHorizontal[j]) = horizontal(i, j);
  }
  const Vec2 oldPosition = PredictGpsPosition(x_, params_.r_gps_FRD);
  State next = x_;
  next.p_NED.head<2>() = antennaNE - (R_NB * params_.r_gps_FRD).head<2>();
  next.v_NED.head<2>().setZero(); // 未知速度的先验均值，不代表观测到静止。
  if (velocity_measured) next.v_NED.head<2>() = antennaVelocityNE -
      (R_NB * (gyro_FRD - x_.b_g).cross(params_.r_gps_FRD)).head<2>();
  if (!finiteState(next) || !nextP.allFinite()) { ++diag_.reject_gps; return false; }
  x_ = next;
  P_ = nextP;
  enforceSymmetry();
  const Scalar missing = std::numeric_limits<Scalar>::quiet_NaN();
  diag_.gps_innov_n = oldPosition.allFinite() ? antennaNE.x() - oldPosition.x() : missing;
  diag_.gps_innov_e = oldPosition.allFinite() ? antennaNE.y() - oldPosition.y() : missing;
  diag_.last_nis_gps = Scalar(0);
  diag_.last_nis_gps_velocity = Scalar(0);
  ++diag_.gps_reset_count;
  ++diag_.horizontal_velocity_reset_count;
  diag_.horizontal_velocity_source = velocity_measured ? 2u : 1u;
  diag_.gps_last_result = 1;
  diag_.gps_velocity_last_result = velocity_measured ? 1 : -2;
  return true;
}

bool Eskf::updateDvl(Scalar /*t*/, const Vec3& /*v_FRD*/, const Mat3& /*R*/) {
  //  后续可补充DVL
  return false;
}

// 静态对齐，水平部署假设捕获 b_a
//
// 背景：MEMS 加速度计有 ~40mg 常值零偏，单测静止 accel 无法把"零偏"和"倾角"
//   分开。Phase 1 深度更新下水平 b_a 不可观，若按 accel→tilt 反推姿态会把
//   零偏误当成 ~2.3° 倾角， roll/pitch≤1° 阈值
//
// 真机标准做法（放平标定）：入水/部署时滑翔机近似水平，于是
//   - 姿态：roll=pitch=0，yaw=先验 init_yaw_ned_rad
//   - 加计零偏：静止时 a_NED 应=0 ⇒ R_NB·(a_mean - b_a) + g_NED = 0
//     水平(绕 Down 轴 yaw 旋转不改 [0,0,-g]) ⇒ a_mean - b_a = [0,0,-g]
//     ⇒ b_a = a_mean - [0,0,-g] = a_mean + [0,0,+g]
//   - 陀螺零偏：b_g = mean(gyro)（静止真角速度=0）
//
// 代价若部署时真有安装倾角，会被误记进 b_a；对水平部署的滑翔机可接受
//   （本仿真 ground_truth 静止姿态 roll=0/pitch=-0.48°，残差<0.5°）。
//   后续如需建模安装角，可加 init_roll/init_pitch 先验参数扣除。
//
// 准入条件由 wrapper 端筛样本，core 假设 caller 已确保静止。
void Eskf::staticAlign(const Vec3* acc_buf, const Vec3* gyro_buf,
                       const Vec3* mag_buf, std::size_t n) {
  if (!initialized_ || n == 0) return;

  Vec3 a_sum = Vec3::Zero();
  Vec3 g_sum = Vec3::Zero();
  Vec3 m_sum = Vec3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    a_sum += acc_buf[i];
    g_sum += gyro_buf[i];
    if (mag_buf) m_sum += mag_buf[i];
  }
  const Scalar inv_n = Scalar(1) / static_cast<Scalar>(n);
  const Vec3 a_mean = a_sum * inv_n;
  const Vec3 g_mean = g_sum * inv_n;
  const Vec3 m_mean = m_sum * inv_n;
  staticAlignFromMeans(a_mean, g_mean, mag_buf ? &m_mean : nullptr);
}

void Eskf::staticAlignFromMeans(const Vec3& acc_mean, const Vec3& gyro_mean,
                                const Vec3* mag_mean) {
  if (!initialized_) return;

  // yaw，有 mag 则用 level 假设从磁场恢复，否则回退先验 init_yaw_ned_rad
  Scalar yaw = params_.init_yaw_ned_rad;
  if (mag_mean && mag_mean->norm() > Scalar(1e-12)) {
    yaw = MagInitYaw(*mag_mean, params_.mag_declination_rad);
  }

  // 姿态：水平 + yaw（仅绕 Down 轴）
  x_.q_NB = Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitZ());
  x_.q_NB.normalize();

  // 陀螺零偏 = 静止角速度均值
  x_.b_g = gyro_mean;

  // 加计零偏 = a_mean - [0,0,-g]（水平静止下的期望比力为 [0,0,-g]_FRD）
  const Vec3 expected_specific_force(Scalar(0), Scalar(0), -params_.g);
  x_.b_a = acc_mean - expected_specific_force;
}

bool Eskf::applyCorrection(const Vec15& dx, const Mat15& joseph_covariance) {
  if (!dx.allFinite() || !joseph_covariance.allFinite()) return false;
  State next = x_;
  next.p_NED += dx.segment<3>(kIdxDeltaP);
  next.v_NED += dx.segment<3>(kIdxDeltaV);
  next.q_NB = InjectDeltaTheta(next.q_NB, dx.segment<3>(kIdxDeltaTh));
  next.b_g += dx.segment<3>(kIdxDeltaBg);
  next.b_a += dx.segment<3>(kIdxDeltaBa);
  const Mat3 J = RightJacobianSO3(dx.segment<3>(kIdxDeltaTh));
  // G 只有姿态块不是 I；只更新这三行/列，与 G*P*G^T 等价并节省板端栈。
  Mat15 nextP = joseph_covariance;
  nextP.block<3, 15>(kIdxDeltaTh, 0).noalias() =
      J * joseph_covariance.block<3, 15>(kIdxDeltaTh, 0);
  nextP.block<15, 3>(0, kIdxDeltaTh) =
      (nextP.block<15, 3>(0, kIdxDeltaTh) * J.transpose()).eval();
  // 先验证候选值再提交；拒绝更新不会留下修改了一半的名义状态。
  if (!finiteState(next) || !nextP.allFinite()) return false;
  x_ = next;
  P_ = nextP;
  enforceSymmetry();
  return true;
}

void Eskf::enforceSymmetry() {
  // 显式按对处理，不产生额外 15x15 临时矩阵，也避免转置表达式别名。
  for (int row = 0; row < kErrDim; ++row) {
    for (int col = row + 1; col < kErrDim; ++col) {
      const Scalar value = Scalar(0.5) * P_(row, col) + Scalar(0.5) * P_(col, row);
      P_(row, col) = value;
      P_(col, row) = value;
    }
  }
}

}  // namespace ug_ekf
