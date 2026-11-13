/* GNSS 水平定位管理：仅 EKF task 写；不依赖 HAL/RTOS，供主机回放复用。 */
#ifndef GNSS_NAVIGATION_H
#define GNSS_NAVIGATION_H
#include <stdint.h>
#include "gnss_sample.h"
#include "gnss_diagnostics.h"
#include "ug_estimation/embedded/ug_ekf_glider.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float position_sigma_m;       /* 无设备精度字段时的保守配置，不代表模块指标 */
  float max_speed_mps;          /* 连续定位间跳变检查的运动上界 */
  float max_position_std_m;
  float velocity_sigma_mps, unknown_velocity_sigma_mps, max_velocity_std_mps;
  uint32_t max_sample_age_ms, fix_timeout_ms, max_imu_age_ms;
  uint32_t velocity_confirm_ms, long_loss_ms;
  uint32_t min_good_fixes;
  uint8_t reference_verified;   /* 安装杆臂与真北参考实测通过后才可置 1 */
} GnssNavConfig;

typedef struct {
  GnssNavConfig config;
  GnssNavDiagnostics diag;
  uint64_t last_epoch_ms, last_rx_us, last_fuse_us, filter_reset_us;
  uint64_t acquisition_start_us, previous_rx_us, last_event_us;
  uint64_t last_velocity_us, velocity_evidence_us, last_reset_us, candidate_rx_us;
  int32_t previous_lat_e7, previous_lon_e7;
  int32_t candidate_lat_e7, candidate_lon_e7;
  uint32_t accepted_streak, normal_updates;
  uint8_t have_epoch, have_rx, anchored, have_fused, have_previous;
  uint8_t have_candidate, have_velocity, no_fix, force_recovery;
} GnssNavigation;

void GnssNavigation_DefaultConfig(GnssNavConfig *config);
void GnssNavigation_Init(GnssNavigation *nav, const GnssNavConfig *config);
/* 保留任务地理原点和已消费 UTC；禁止标定重启后重用旧定位。 */
void GnssNavigation_OnFilterReset(GnssNavigation *nav, uint64_t now_us);
/* WGS84 椭球面 ECEF→当地 NE，double 消差后转 float；最大适用半径 100 km。 */
int GnssNavigation_Project(int32_t origin_lat_e7, int32_t origin_lon_e7,
                           int32_t lat_e7, int32_t lon_e7, float *north, float *east);
/* sample=NULL 时仅刷新失锁/年龄；调用前先处理同轮 IMU/深度。 */
void GnssNavigation_Step(GnssNavigation *nav, const GnssSample *sample,
                         uint64_t now_us, uint64_t last_imu_us,
                         const UgEkfOutput *output);

#ifdef __cplusplus
}
#endif
#endif
