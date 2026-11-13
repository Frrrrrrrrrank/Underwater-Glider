/* 只读 GNSS 诊断类型；控制/遥测读快照不依赖输入邮箱或导航管理器。 */
#ifndef GNSS_DIAGNOSTICS_H
#define GNSS_DIAGNOSTICS_H
#include <stdint.h>

typedef enum {
  GNSS_NAV_WAIT_FIX = 0, GNSS_NAV_POSITION_ONLY, GNSS_NAV_TRACKING,
  GNSS_NAV_DR_UNTRUSTED, GNSS_NAV_REACQUIRING, GNSS_NAV_WAIT_EKF,
  GNSS_NAV_ACQUIRING = GNSS_NAV_POSITION_ONLY,
  GNSS_NAV_AIDING = GNSS_NAV_TRACKING,
  GNSS_NAV_COASTING = GNSS_NAV_DR_UNTRUSTED
} GnssNavState;
typedef enum {
  GNSS_REASON_NONE = 0, GNSS_REASON_BAD_INPUT, GNSS_REASON_DUPLICATE,
  GNSS_REASON_OUT_OF_ORDER, GNSS_REASON_STALE, GNSS_REASON_NOT_READY,
  GNSS_REASON_ACQUIRING, GNSS_REASON_JUMP, GNSS_REASON_NIS,
  GNSS_REASON_NUMERIC, GNSS_REASON_RESET, GNSS_REASON_IMU_STALE,
  GNSS_REASON_NO_FIX, GNSS_REASON_VELOCITY_NIS, GNSS_REASON_QUALITY
} GnssNavReason;
enum {
  GNSS_FLAG_ORIGIN = 1U, GNSS_FLAG_ANCHORED = 2U,
  GNSS_FLAG_AIDING_VALID = 4U, GNSS_FLAG_REFERENCE_VERIFIED = 8U,
  GNSS_FLAG_COVARIANCE = 16U, GNSS_FLAG_MEASUREMENT = 32U,
  GNSS_FLAG_INNOVATION = 64U, GNSS_FLAG_FUSED_ONCE = 128U,
  GNSS_FLAG_VELOCITY_VALID = 256U, GNSS_FLAG_VELOCITY_MEASUREMENT = 512U,
  GNSS_FLAG_RECOVERY = 1024U, GNSS_FLAG_VELOCITY_COVARIANCE = 2048U,
  GNSS_FLAG_VELOCITY_INNOVATION = 4096U
};

enum {
  GNSS_RECOVERY_NONE = 0, GNSS_RECOVERY_FIRST_FIX, GNSS_RECOVERY_AID_LOSS,
  GNSS_RECOVERY_FILTER_RESET, GNSS_RECOVERY_NUMERIC, GNSS_RECOVERY_REJECT
};

/* SI 单位；遥测层转定点。年龄 UINT32_MAX 表示从未有过。 */
typedef struct {
  uint32_t state, reason, flags, sample_seq, rx_age_ms, fuse_age_ms;
  uint32_t utc_date, utc_ms, origin_id;
  int32_t lat_e7, lon_e7, origin_lat_e7, origin_lon_e7;
  float meas_n, meas_e, innov_n, innov_e, nis, r_var;
  uint32_t accept_count, reject_count, reset_count, duplicate_count;
  uint32_t drop_count, input_reject_count;
  uint32_t quality_source, velocity_source, vel_fuse_age_ms, no_fix_count;
  float meas_vn, meas_ve, vel_nis, vel_r_var;
  uint32_t vel_accept_count, vel_reject_count, recovery_reason;
  float reset_before_n, reset_before_e, reset_before_vn, reset_before_ve;
  float reset_after_n, reset_after_e, reset_after_vn, reset_after_ve;
  uint32_t reset_age_ms, reset_valid_mask;
} GnssNavDiagnostics;
#endif
