/* state_estimate.h
 * EKF 的输出状态快照定义，供控制器/上位机观察。
 * estimation → control / telemetry 的约定：状态估计输出快照。
 * 内嵌 ESKF wrapper 的 UgEkfOutput（姿态/位置/速度/状态/健康/诊断）+ 自增序号。
 *
 * 约定：
 *   - 控制器只认 out.valid_for_control（== status==RUNNING && healthy）。
 *   - 散读全局 g_state_estimate 仅供 Ozone 观察；闭环必须走 EkfBridge_CopyState()。
 */
#ifndef GLIDER_STATE_ESTIMATE_H
#define GLIDER_STATE_ESTIMATE_H

#include "ug_estimation/embedded/ug_ekf_glider.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  UgEkfOutput out;          /* 滤波器输出，字段见 ug_ekf_glider.h */
  uint32_t    update_seq;   /* 每次 EKF 刷新自增；读侧判新旧/校验一致性 */
} StateEstimate;

#ifdef __cplusplus
}
#endif

#endif /* GLIDER_STATE_ESTIMATE_H */
