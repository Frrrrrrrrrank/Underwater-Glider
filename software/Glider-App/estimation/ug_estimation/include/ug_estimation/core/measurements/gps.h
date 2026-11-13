// GNSS 天线二维位置模型；经纬度、原点和数据新鲜度由平台入口处理。
#pragma once

#include "ug_estimation/core/types.h"
#include "ug_estimation/core/quaternion_utils.h"

namespace ug_ekf {

using Mat2x15 = Eigen::Matrix<Scalar, 2, 15>;

inline Vec2 PredictGpsPosition(const State& x, const Vec3& r_gps_FRD) {
  return (x.p_NED + x.q_NB.toRotationMatrix() * r_gps_FRD).head<2>();
}

inline void BuildGpsPositionH(const State& x, const Vec3& r_gps_FRD,
                              Mat2x15& H) {
  H.setZero();
  H(0, kIdxDeltaP) = Scalar(1);
  H(1, kIdxDeltaP + 1) = Scalar(1);
  // q_true=q_nom*Exp(delta_theta)，d(Rr)=-R*Hat(r)*delta_theta。
  H.block<2, 3>(0, kIdxDeltaTh) =
      -(x.q_NB.toRotationMatrix() * Hat(r_gps_FRD)).topRows<2>();
}

inline Vec2 PredictGpsVelocity(const State& x, const Vec3& r_gps_FRD,
                               const Vec3& gyro_FRD) {
  const Vec3 omega = gyro_FRD - x.b_g;
  return (x.v_NED + x.q_NB.toRotationMatrix() * omega.cross(r_gps_FRD)).head<2>();
}

inline void BuildGpsVelocityH(const State& x, const Vec3& r_gps_FRD,
                              const Vec3& gyro_FRD, Mat2x15& H) {
  const Mat3 R_NB = x.q_NB.toRotationMatrix();
  const Vec3 lever_velocity = (gyro_FRD - x.b_g).cross(r_gps_FRD);
  H.setZero();
  H(0, kIdxDeltaV) = Scalar(1);
  H(1, kIdxDeltaV + 1) = Scalar(1);
  H.block<2, 3>(0, kIdxDeltaTh) = -(R_NB * Hat(lever_velocity)).topRows<2>();
  H.block<2, 3>(0, kIdxDeltaBg) = (R_NB * Hat(r_gps_FRD)).topRows<2>();
}

inline Mat2 GpsVelocityGyroCovariance(const State& x, const Vec3& r_gps_FRD,
                                     Scalar gyro_variance) {
  const Eigen::Matrix<Scalar, 2, 3> J =
      (x.q_NB.toRotationMatrix() * Hat(r_gps_FRD)).topRows<2>();
  return gyro_variance * J * J.transpose();
}

}  // namespace ug_ekf
