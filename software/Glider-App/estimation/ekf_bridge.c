/* ekf_bridge.c 定义见头文件 ekf_bridge.h */
#include "ekf_bridge.h"
#include "glider_time.h"
#include "ug_estimation/embedded/ug_ekf_glider.h"
#include "pressure_sensor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <math.h>

//本板深度计为【表压】传感器（空气里≈0）：depth = P_gauge/(ρg) = bar*1e5/(ρg)，不减大气压。
//系数与 wrapper 默认一致；空气中 gauge≈0 故 depth≈0，不受 rho 取值影响
static const float kWaterRho = 1028.0f;   /* 水密度 kg/m^3 */
static const float kGravity  = 9.81f;     /* 重力 m/s^2 */

/* 磁力计硬铁/软铁标定（模块系/FLU，uT）
 * 目前版本为 2026-07-17 14:29采样标定版本, 样本 7966, station/tools/mag_calib_fit.py（logs/csv/mag_20260717_141522）。
 * 校正算法：m_cal = A * (m_raw - b)，【必须在模块系、mount_R 之前】算，
 *       再交给 UgEkf_PushMag（其内部 toFrd 做 FLU→FRD）。
 * 质量: 校正后 |m_cal| std/mean≈2.9%，机体带磁部件变动后必须重标。
 * 下列值仅作固件内参考；上电 valid=0，不会参与融合，必须由参数事务成功提交后才启用。 */
static float s_mag_hard_iron_ut[3] = { 5.5920f, 2.4283f, 10.0223f };
static float s_mag_soft_iron[3][3] = {
  { 1.091314f, -0.004086f,  0.161096f },
  { -0.004086f, 1.319577f, -0.013531f },
  {  0.161096f, -0.013531f, 0.787296f },
};

//IMU 磁力计量程为 ±800uT；正常软铁矩阵元素在 1 附近
static const float kMagHardIronAbsMaxUt = 800.0f;
static const float kMagSoftIronAbsMax = 5.0f;
static const float kMagSoftIronDetMin = 1.0e-4f;
static const float kMagSoftIronSymmetryTol = 1.0e-3f;
static const float kMagSoftIronEigenMin = 5.0e-2f;
static const float kMagSoftIronEigenMax = 5.0f;
static UgEkfConfig s_ekf_config;

typedef struct {
  uint8_t  table_bytes[EKF_MAG_CAL_TABLE_BYTES];
  uint32_t expected_crc;
  uint32_t seq;
} EkfMagCalPending;

/* Ethernet task 写、EKF task 读的 latest-wins 单槽邮箱。 */
static EkfMagCalPending s_mag_cal_pending;
static uint32_t s_mag_cal_pending_seq = 0U;
static uint32_t s_mag_cal_handled_seq = 0U;
/* BSS 上电初值为 0。只有 EKF task 完成完整初始化后才置 1；
 * Ethernet task 据此避免把参数写进即将被初始化清零的邮箱。 */
static volatile uint8_t s_ekf_bridge_ready = 0U;

/*  受保护输入缓存（IMU task 写，EKF task 读，单槽 latest-wins）  */
typedef struct {
  float    gx, gy, gz, ax, ay, az;
  float    mx, my, mz;          /* 模块系原始 mag（与 IMU 同帧到达，未校正） */
  uint64_t t_us;
  uint32_t seq;
} EkfImuInput;

static EkfImuInput s_imu_in;             /* 临界区保护 */
static uint32_t    s_imu_in_seq = 0;     /* 最新投喂序号 */
static uint32_t    s_imu_last_seq = 0;   /* EKF 已处理序号 */
static uint32_t    s_pressure_last_seq = 0;
static uint32_t    s_depth_measurement_tick_ms = 0;

// 暴露给OZone的输出快照，观察参数用
StateEstimate     g_state_estimate;
static uint32_t   s_snap_seq = 0;
volatile uint32_t g_ekf_imu_push_count = 0;
volatile uint32_t g_ekf_imu_drop_count = 0;
volatile uint32_t g_ekf_pressure_push_count = 0;
volatile uint32_t g_ekf_mag_push_count = 0;   // 已 push 给 EKF 的校正 mag 帧数(Ozone 观察)
volatile float    g_ekf_depth_meas = 0.0f;   // 换算后的深度测量值(m)，供 Ozone 对比 out.depth_m
volatile float    g_ekf_mag_cal[3] = {0.0f, 0.0f, 0.0f};  // 校正后 mag(模块系,uT)；转动时 |·| 应≈常数
volatile uint32_t g_ekf_mag_cal_crc = 0U;
volatile uint8_t  g_ekf_mag_cal_valid = 0U;
volatile uint32_t g_ekf_mag_cal_request_count = 0U;
volatile uint32_t g_ekf_mag_cal_apply_count = 0U;
volatile uint32_t g_ekf_mag_cal_reject_count = 0U;

/* CRC-32/ISO-HDLC，与 Python zlib.crc32 一致。这里只在参数提交时计算一次，
 * 64 字节的运算量放在 EKF 任务上下文不会拉长 UART 中断。 */
static uint32_t magCalCrc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;

  for (i = 0U; i < length; i++) {
    uint32_t bit;
    crc ^= (uint32_t)data[i];

    for (bit = 0U; bit < 8U; bit++) {
      const uint32_t lsb_mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & lsb_mask);
    }
  }

  return crc ^ 0xFFFFFFFFUL;
}

static uint32_t magCalReadLeU32(const uint8_t *bytes)
{
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8U) |
         ((uint32_t)bytes[2] << 16U) |
         ((uint32_t)bytes[3] << 24U);
}

/* 先拼成整数位模式再 memcpy，避免未对齐 float 访问和 strict-aliasing 问题。 */
static float magCalReadLeFloat(const uint8_t *bytes)
{
  const uint32_t bits = magCalReadLeU32(bytes);
  float value;

  (void)memcpy(&value, &bits, sizeof(value));
  return value;
}

static int takePendingMagCalibration(EkfMagCalPending *dst)
{
  int got = 0;

  taskENTER_CRITICAL();
  if (s_mag_cal_pending_seq != s_mag_cal_handled_seq) {
    *dst = s_mag_cal_pending;
    s_mag_cal_handled_seq = dst->seq;
    got = 1;
  }
  taskEXIT_CRITICAL();

  return got;
}

/* 所有结果先写局部数组；任何字段失败都不触碰上一份 active 参数。 */
static int decodeMagCalibration(
    const uint8_t table_bytes[EKF_MAG_CAL_TABLE_BYTES],
    float hard_iron_ut[3],
    float soft_iron[3][3])
{
  uint32_t i;
  float det;

  /* float[12..15] 是 v1 保留区，必须为 +0.0f 的全零位模式。 */
  for (i = 12U * 4U; i < EKF_MAG_CAL_TABLE_BYTES; i++) {
    if (table_bytes[i] != 0U) {
      return 0;
    }
  }

  for (i = 0U; i < 3U; i++) {
    const float value = magCalReadLeFloat(&table_bytes[i * 4U]);
    if (!isfinite(value) || fabsf(value) > kMagHardIronAbsMaxUt) {
      return 0;
    }
    hard_iron_ut[i] = value;
  }

  for (i = 0U; i < 9U; i++) {
    const float value = magCalReadLeFloat(&table_bytes[(3U + i) * 4U]);
    if (!isfinite(value) || fabsf(value) > kMagSoftIronAbsMax) {
      return 0;
    }
    soft_iron[i / 3U][i % 3U] = value;
  }

  /* 本协议的 A 是椭球拟合所得对称正定平方根，不接受一般旋转/翻转矩阵。
   * 对称矩阵的特征值落在 Gershgorin 圆盘内；把每个圆盘都限制在
   * [0.05, 5]，可同时拒绝负增益、近奇异和增益过大的病态矩阵。 */
  for (i = 0U; i < 3U; i++) {
    uint32_t j;
    float radius = 0.0f;

    for (j = 0U; j < 3U; j++) {
      if (j == i) {
        continue;
      }
      if (fabsf(soft_iron[i][j] - soft_iron[j][i]) >
          kMagSoftIronSymmetryTol) {
        return 0;
      }
      radius += fabsf(soft_iron[i][j]);
    }

    if ((soft_iron[i][i] - radius) < kMagSoftIronEigenMin ||
        (soft_iron[i][i] + radius) > kMagSoftIronEigenMax) {
      return 0;
    }
  }

  det =
      soft_iron[0][0] *
          (soft_iron[1][1] * soft_iron[2][2] -
           soft_iron[1][2] * soft_iron[2][1]) -
      soft_iron[0][1] *
          (soft_iron[1][0] * soft_iron[2][2] -
           soft_iron[1][2] * soft_iron[2][0]) +
      soft_iron[0][2] *
          (soft_iron[1][0] * soft_iron[2][1] -
           soft_iron[1][1] * soft_iron[2][0]);

  /* 行列式作为第二道完整性检查；上面的特征值界已经保证正定且不过度病态。 */
  if (!isfinite(det) || det < kMagSoftIronDetMin) {
    return 0;
  }

  return 1;
}

/* 仅由 EKF task 在 Step 开头调用：校验完整事务并在帧边界一次切换。 */
static void applyPendingMagCalibration(void)
{
  EkfMagCalPending pending;
  float hard_iron_ut[3];
  float soft_iron[3][3];
  uint32_t i;

  if (!takePendingMagCalibration(&pending)) {
    return;
  }

  if ((magCalCrc32(pending.table_bytes,
                   EKF_MAG_CAL_TABLE_BYTES) != pending.expected_crc) ||
      !decodeMagCalibration(pending.table_bytes, hard_iron_ut, soft_iron)) {
    g_ekf_mag_cal_reject_count++;
    return;
  }

  // PC 超时会整组重发；同一 active CRC 视为幂等成功，不能反复重启对齐
  if (g_ekf_mag_cal_valid != 0U &&
      g_ekf_mag_cal_crc == pending.expected_crc) {
    return;
  }

  for (i = 0U; i < 3U; i++) {
    uint32_t j;
    s_mag_hard_iron_ut[i] = hard_iron_ut[i];
    for (j = 0U; j < 3U; j++) {
      s_mag_soft_iron[i][j] = soft_iron[i][j];
    }
  }

  /* 标定坐标发生变化后必须清除旧的 mag 对齐均值和旧 yaw 状态。
   * 这里只重启算法 wrapper，不清参数邮箱/诊断；随后本 Step 的最新 IMU
   * 会成为新一轮静止对齐的第一帧。 */
  UgEkf_Init(&s_ekf_config);
  g_ekf_mag_cal_crc = pending.expected_crc;
  /* 最后发布 valid，观察侧和下一帧 mag 不会看到半成品。 */
  g_ekf_mag_cal_valid = 1U;
  g_ekf_mag_cal_apply_count++;
}

//填数据
void EkfBridge_Init(void)
{
  UgEkfConfig cfg;

  /* 本函数只能由 EKF task 调用；先关闭参数入口，再建立算法和全部缓存。 */
  s_ekf_bridge_ready = 0U;
  UgEkf_DefaultConfig(&cfg);
  /* IMU 原始静止读数 az≈+g，模块坐标按“前-左-上”处理；
   * 转换到 EKF 使用的“前-右-下”机体坐标系。
   * 行主序矩阵 diag(+1,-1,-1)：前轴不变，左轴取反变为右轴，上轴取反变为下轴。 */
  cfg.mount_R[0] =  1.0f; cfg.mount_R[1] =  0.0f; cfg.mount_R[2] =  0.0f;
  cfg.mount_R[3] =  0.0f; cfg.mount_R[4] = -1.0f; cfg.mount_R[5] =  0.0f;
  cfg.mount_R[6] =  0.0f; cfg.mount_R[7] =  0.0f; cfg.mount_R[8] = -1.0f;

  /* 压力计(深度计)相对 IMU 的杠杆臂，FRD(前-右-下)，单位米。
   * 量测模型 z = e_D·(p_NED + R_NB·r_pressure_FRD)，方向为 IMU→压力计。
   * 由实测推算（均相对车体原点、FLU 前-左-上，单位 mm）：
   *   压力计 = (-300, 0, -25)，IMU = (24.29, 19.5, 0)
   *   差值(压力计-IMU)_FLU = (-324.29, -19.5, -25)
   *   FLU→FRD (x,-y,-z)    = (-324.29, 19.5, 25) mm = 下方各值(米)
   * 注：量得不很准，量级 ~cm，对深度影响很小（仅大姿态角时显现）。 */
  cfg.r_pressure_frd[0] = -0.32429f;  /* 前：压力计在 IMU 后方 324.29mm */
  cfg.r_pressure_frd[1] =  0.01950f;  /* 右：压力计在 IMU 右侧 19.5mm */
  cfg.r_pressure_frd[2] =  0.02500f;  /* 下：压力计在 IMU 下方 25mm */

  /* 磁力计 yaw 融合配置（硬/软铁标定值由参数事务写入 active b/A，Step 里应用）。
   * declination(磁偏角=磁北→真北)与硬/软铁标定无关，是地理量：
   *   - 台架验证阶段填 0：yaw 参考的是"磁北"，验证"yaw 是否跟随转动、可复现"足够；
   *   - 真实导航前按下水点磁偏角填（NOAA / magnetic-declination.com），单位 rad。 */
  cfg.mag_declination_rad = 0.0f;   /* 台架先 0；下水前实测填 */
  /* mag_yaw_var：yaw 量测方差(rad²)。bring-up 先保守放大到 ~0.04(std≈11°)，
   * 让 mag 慢慢修 yaw、不过度信任；确认 accept 率稳定后可收回默认 0.01。 */
  cfg.mag_yaw_var = 0.04f;
  s_ekf_config = cfg;
  UgEkf_Init(&s_ekf_config);

  memset(&s_imu_in, 0, sizeof(s_imu_in));
  s_imu_in_seq = 0;
  s_imu_last_seq = 0;
  s_pressure_last_seq = 0;
  s_depth_measurement_tick_ms = 0;
  g_ekf_depth_meas = 0.0f;
  memset(&s_mag_cal_pending, 0, sizeof(s_mag_cal_pending));
  s_mag_cal_pending_seq = 0U;
  s_mag_cal_handled_seq = 0U;
  g_ekf_mag_cal_crc = 0U;
  g_ekf_mag_cal_valid = 0U;
  g_ekf_mag_cal_request_count = 0U;
  g_ekf_mag_cal_apply_count = 0U;
  g_ekf_mag_cal_reject_count = 0U;
  memset(&g_state_estimate, 0, sizeof(g_state_estimate));
  s_snap_seq = 0;
  /* 最后发布 ready，避免其他任务观察到半初始化状态。 */
  taskENTER_CRITICAL();
  s_ekf_bridge_ready = 1U;
  taskEXIT_CRITICAL();
}
//把一帧IMU(含同帧原始mag)暂存起来，在IMUtask里
void EkfBridge_SubmitImu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz, uint64_t t_us)
{
  taskENTER_CRITICAL();//写快照，这是用于防止读写冲突的地方
  /* 上一帧若尚未被 EKF 使用就被覆盖，记一次丢弃（观察是否跟不上 IMU 率） */
  if (s_imu_in_seq != s_imu_last_seq) {
    g_ekf_imu_drop_count++;
  }
  s_imu_in.gx = gx; s_imu_in.gy = gy; s_imu_in.gz = gz;
  s_imu_in.ax = ax; s_imu_in.ay = ay; s_imu_in.az = az;
  s_imu_in.mx = mx; s_imu_in.my = my; s_imu_in.mz = mz;  /* 原始mag，随IMU同帧latest-wins */
  s_imu_in.t_us = t_us;
  s_imu_in.seq = ++s_imu_in_seq;
  taskEXIT_CRITICAL();
}

uint8_t EkfBridge_SubmitMagCalibration(
    const uint8_t table_bytes[EKF_MAG_CAL_TABLE_BYTES],
    uint32_t expected_crc)
{
  uint8_t accepted = 0U;

  if (table_bytes == NULL) {
    return 0U;
  }

  //与 ready 的发布放在同一个临界区；未 ready 时让 PC 整组重试
  //跨任务仍只做固定 64 字节拷贝和发布序号，不做 CRC 或矩阵计算
  taskENTER_CRITICAL();
  if (s_ekf_bridge_ready != 0U) {
    (void)memcpy(s_mag_cal_pending.table_bytes,
                 table_bytes,
                 EKF_MAG_CAL_TABLE_BYTES);
    s_mag_cal_pending.expected_crc = expected_crc;
    s_mag_cal_pending.seq = ++s_mag_cal_pending_seq;
    g_ekf_mag_cal_request_count++;
    accepted = 1U;
  }
  taskEXIT_CRITICAL();

  return accepted;
}

/* 取一帧待处理输入；无新样本返回 0 */
static int takeImuInput(EkfImuInput *dst)
{
  int got = 0;
  taskENTER_CRITICAL();//避免撕裂读取
  if (s_imu_in_seq != s_imu_last_seq) {
    *dst = s_imu_in;
    s_imu_last_seq = s_imu_in_seq;
    got = 1;
  }
  taskEXIT_CRITICAL();
  return got;
}

void EkfBridge_Step(void)//在EKF的task里调用
{
  /* CRC 和参数切换必须在任务上下文、且发生在本帧 mag 计算之前。 */
  applyPendingMagCalibration();

  /* 1) IMU：单槽 latest-wins，每次只取最新一帧，处理完即返回，保证 Step 一定走到 publish，
   *    不会被高速输入拖住活锁，输入快于 predict 时只丢帧，drop_count 增长 */
  // 后续可能需要实现ring-buffer
  EkfImuInput in;
  if (takeImuInput(&in)) {
    //EKF的Task，算法层
    UgEkf_PushImu(in.gx, in.gy, in.gz, in.ax, in.ay, in.az, in.t_us);
    g_ekf_imu_push_count++;

    /* 1b) 磁力计 yaw 融合：先在【模块系】做硬/软铁校正 m_cal = A*(m_raw - b)，
     *     再 PushMag（wrapper 内 toFrd 才把 FLU→FRD，故校正必须在此、mount_R 之前）。
     *     mag 与 IMU 同帧、同 latest-wins 槽，故随 IMU 一起 push（对齐阶段也 push，
     *     finishAlign 用累计 mag 均值算初始 yaw）。updateMag 只约束 yaw，不碰 roll/pitch。 */
    if (g_ekf_mag_cal_valid != 0U) {
      const float dx = in.mx - s_mag_hard_iron_ut[0];
      const float dy = in.my - s_mag_hard_iron_ut[1];
      const float dz = in.mz - s_mag_hard_iron_ut[2];
      const float mcx = s_mag_soft_iron[0][0]*dx + s_mag_soft_iron[0][1]*dy + s_mag_soft_iron[0][2]*dz;
      const float mcy = s_mag_soft_iron[1][0]*dx + s_mag_soft_iron[1][1]*dy + s_mag_soft_iron[1][2]*dz;
      const float mcz = s_mag_soft_iron[2][0]*dx + s_mag_soft_iron[2][1]*dy + s_mag_soft_iron[2][2]*dz;
      g_ekf_mag_cal[0] = mcx; g_ekf_mag_cal[1] = mcy; g_ekf_mag_cal[2] = mcz;  /* Ozone: 转动时|·|应≈常数 */
      UgEkf_PushMag(mcx, mcy, mcz, in.t_us);
      g_ekf_mag_push_count++;
    }
  }

  /* 2) 压力：仅在 valid 且 pressure_update_seq 变化时 push（对齐前 wrapper 内部会忽略）。
   *    本板深度计是【表压】(空气≈0)：depth = bar*1e5/(ρg)，不减大气压，
   *    故绕过 UgEkf_PushPressureBar（它按绝对压减 p_atm，会多出 -10m 偏置）。
   *    先在临界区拷出同一版 pressure/value/tick/seq，避免任务切换造成字段跨帧。 */
  uint32_t pressure_valid;
  uint32_t pressure_seq;
  uint32_t pressure_tick_ms;
  float pressure_bar;
  taskENTER_CRITICAL();
  pressure_valid = g_pressure_sensor_readout.pressure_valid;
  pressure_seq = g_pressure_sensor_readout.pressure_update_seq;
  pressure_tick_ms = g_pressure_sensor_readout.pressure_update_tick;
  pressure_bar = g_pressure_sensor_readout.pressure_bar;
  taskEXIT_CRITICAL();

  if (pressure_valid != 0U && pressure_seq != s_pressure_last_seq) {
    s_pressure_last_seq = pressure_seq;
    s_depth_measurement_tick_ms = pressure_tick_ms;
    g_ekf_depth_meas = pressure_bar * 1.0e5f / (kWaterRho * kGravity);
    UgEkf_PushDepth(g_ekf_depth_meas, GliderTime_NowUs());
    g_ekf_pressure_push_count++;
  }

  /* 3) mag 已在 1b 随 IMU 帧 push（校正+yaw更新）。 */

  /* 4) 拉输出，然后发布受保护快照 */
  UgEkfOutput out;
  UgEkf_GetOutput(&out);
  taskENTER_CRITICAL();//嵌入式的保护读写
  g_state_estimate.out = out;
  g_state_estimate.update_seq = ++s_snap_seq;
  g_state_estimate.depth_measurement_seq = g_ekf_pressure_push_count;
  g_state_estimate.depth_measurement_tick_ms = s_depth_measurement_tick_ms;
  taskEXIT_CRITICAL();
}

uint32_t EkfBridge_CopyState(StateEstimate *out)
{
  uint32_t seq;
  if (out == NULL) {
    return 0;
  }
  taskENTER_CRITICAL();
  *out = g_state_estimate;
  seq = g_state_estimate.update_seq;
  taskEXIT_CRITICAL();
  return seq;
}
