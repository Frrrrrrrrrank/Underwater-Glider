// ug_estimation/core/quaternion_utils.h
//
// 四元数与 SO(3) 工具：Exp_q, Log_q, hat, 旋转矩阵互转。
// 全部 inline / constexpr 友好；无堆分配。

#pragma once

#include "ug_estimation/core/types.h"

namespace ug_ekf {

// 反对称矩阵 (^∧ 算子)
inline Mat3 Hat(const Vec3& v) {
  Mat3 m;
  m <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return m;
}

// SO(3) 旋转向量 → 单位四元数：q = [cos(|φ|/2), sin(|φ|/2)·φ/|φ|]
//   注意：约定 Exp_q(φ) 使得旋转角 = |φ|，与 ESKF 反馈注入 δθ = φ 直接相容。
//   小角度展开避免除零。
inline Quat ExpQ(const Vec3& phi) {
  const Scalar theta = phi.norm();
  if (theta < Scalar(1e-8)) {
    Quat q;
    q.w() = Scalar(1);
    q.vec() = Scalar(0.5) * phi;       // 二阶以下精度
    q.normalize();
    return q;
  }
  const Scalar half = theta * Scalar(0.5);
  const Scalar s = std::sin(half) / theta;
  Quat q;
  q.w() = std::cos(half);
  q.vec() = phi * s;
  return q;
}

// 单位四元数 → SO(3) 旋转向量
inline Vec3 LogQ(const Quat& q_in) {
  Quat q = q_in;
  if (q.w() < Scalar(0)) q.coeffs() = -q.coeffs();   // 选 w≥0 半球
  const Scalar nv = q.vec().norm();
  if (nv < Scalar(1e-8)) {
    return Scalar(2) * q.vec();
  }
  const Scalar theta = Scalar(2) * std::atan2(nv, q.w());
  return q.vec() * (theta / nv);
}

// 将旋转向量误差 δθ 注入名义四元数：q_new = q ⊗ Exp_q(δθ)
inline Quat InjectDeltaTheta(const Quat& q, const Vec3& dtheta) {
  Quat qn = q * ExpQ(dtheta);
  qn.normalize();
  return qn;
}

// 右乘误差注入后的协方差变换；小角度级数避免 FP32 下 1-cos 的消差。
inline Mat3 RightJacobianSO3(const Vec3& phi) {
  const Scalar theta2 = phi.squaredNorm();
  const Mat3 A = Hat(phi);
  Scalar a, b;
  if (theta2 < Scalar(1e-4)) {
    a = Scalar(0.5) - theta2 / Scalar(24) + theta2 * theta2 / Scalar(720);
    b = Scalar(1.0 / 6.0) - theta2 / Scalar(120) + theta2 * theta2 / Scalar(5040);
  } else {
    const Scalar theta = std::sqrt(theta2);
    a = (Scalar(1) - std::cos(theta)) / theta2;
    b = (theta - std::sin(theta)) / (theta2 * theta);
  }
  return Mat3::Identity() - a * A + b * A * A;
}

}  // namespace ug_ekf
