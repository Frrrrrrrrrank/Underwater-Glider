#ifndef GNSS_SAMPLE_H
#define GNSS_SAMPLE_H

#include <stdint.h>

/* 这些位表示输入检查通过，不代表定位精度或已被 EKF 接受。 */
#define GNSS_VALID_CHECKSUM    (1UL << 0)
#define GNSS_VALID_ACTIVE      (1UL << 1)
#define GNSS_VALID_COORDINATES (1UL << 2)
#define GNSS_VALID_UTC         (1UL << 3)
#define GNSS_VALID_MODE        (1UL << 4)
#define GNSS_VALID_VELOCITY    (1UL << 5)
#define GNSS_VALID_NO_FIX      (1UL << 6)
#define GNSS_VALID_REQUIRED    (GNSS_VALID_CHECKSUM | GNSS_VALID_ACTIVE | GNSS_VALID_COORDINATES | GNSS_VALID_UTC | GNSS_VALID_MODE)

#define GNSS_QUALITY_UNKNOWN   0U /* 未报告精度；消费方使用配置方差并注明来源。 */
#define GNSS_QUALITY_REPORTED  1U
#define GNSS_VELOCITY_NONE     0U
#define GNSS_VELOCITY_RMC      1U /* 接收机 SOG/COG，不代表已确认使用多普勒测速。 */

typedef struct
{
  uint32_t seq;                 /* 通过串口输入检查的发布序号；UTC 去重由消费方负责。 */
  uint64_t t_rx_us;             /* 主控处理完整行的单调时间，非卫星测量时刻。 */
  uint32_t utc_date_yyyymmdd;    /* RMC 两位年份按 2000..2099 解释。 */
  uint32_t utc_ms_of_day;       /* 0..86399999；UTC 仅标识历元，不替代主控时基。 */
  int32_t lat_deg_e7;           /* 十进制度乘 1e7，南纬/西经为负。 */
  int32_t lon_deg_e7;           /* 十进制度乘 1e7，南纬/西经为负。 */
  uint32_t validity_flags;
  float vel_n_mps, vel_e_mps;  /* 地面 N/E 速度；仅 VELOCITY 位有效时使用，不是艇体艏向。 */
  uint32_t quality_source;     /* RMC 不含位置精度，当前始终为 UNKNOWN。 */
  uint32_t velocity_source;
  float position_var_m2;       /* 0=未报告，绝不是零误差；消费方用配置替代。 */
  float velocity_var_m2ps2;    /* 0=未报告；保留质量扩展合同，当前不拼 GGA/GST。 */
} GnssSample;

/* NO_FIX 是带合法 UTC/校验的无定位事件；不置 ACTIVE/COORDINATES/VELOCITY，
 * 坐标和速度清零。消费方先处理事件，再检查位置用的 VALID_REQUIRED。 */

#endif
