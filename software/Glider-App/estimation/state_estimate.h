/* state_estimate.h
 * EKF 的输出状态快照定义，供控制器/上位机观察。
 * estimation → control / telemetry 的约定：状态估计输出快照。
 * 内嵌 ESKF wrapper 的 UgEkfOutput（姿态/位置/速度/状态/健康/诊断）+ 自增序号。
 *
 * 约定：
 *   - out.valid_for_control 是原有姿态/数值健康；水平位置另看 gnss 有效位和年龄。
 *   - 散读全局 g_state_estimate 仅供 Ozone 观察；闭环必须走 EkfBridge_CopyState()。
 */
#ifndef GLIDER_STATE_ESTIMATE_H
#define GLIDER_STATE_ESTIMATE_H

#include "ug_estimation/embedded/ug_ekf_glider.h"
#include <stdint.h>
#include "gnss_diagnostics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  UgEkfOutput out;          /* 滤波器输出，字段见 ug_ekf_glider.h */
  uint32_t    update_seq;   /* 每次 EKF 刷新自增；读侧判新旧/校验一致性 */
  /* 只在一帧新压力被实际投喂给 EKF 时变化，供低速深度外环判新样本。 */
  uint32_t    depth_measurement_seq;
  uint32_t    depth_measurement_tick_ms;
  uint32_t    update_tick_ms; /* 本快照生成时刻；与遥测发送时刻区分 */
  GnssNavDiagnostics gnss;    /* 原点、鲜度、接受/拒绝和重捕获，随 out 一致发布 */
} StateEstimate;

#ifdef __cplusplus
}
#endif

#endif /* GLIDER_STATE_ESTIMATE_H */
