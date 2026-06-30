/* ekf_bridge.c 定义见头文件 ekf_bridge.h */
#include "ekf_bridge.h"
#include "glider_time.h"
#include "ug_estimation/embedded/ug_ekf_glider.h"
#include "pressure_sensor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* 本板深度计为【表压】传感器（空气里≈0）：depth = P_gauge/(ρg) = bar*1e5/(ρg)，不减大气压。
   系数与 wrapper 默认一致；空气中 gauge≈0 故 depth≈0，不受 rho 取值影响。 */
static const float kWaterRho = 1028.0f;   /* 水密度 kg/m^3 */
static const float kGravity  = 9.81f;     /* 重力 m/s^2 */

/*  受保护输入缓存（IMU task 写，EKF task 读，单槽 latest-wins）  */
typedef struct {
  float    gx, gy, gz, ax, ay, az;
  uint64_t t_us;
  uint32_t seq;
} EkfImuInput;

static EkfImuInput s_imu_in;             /* 临界区保护 */
static uint32_t    s_imu_in_seq = 0;     /* 最新投喂序号 */
static uint32_t    s_imu_last_seq = 0;   /* EKF 已处理序号 */
static uint32_t    s_pressure_last_tick = 0;

// 暴露给OZone的输出快照，观察参数用
StateEstimate     g_state_estimate;
static uint32_t   s_snap_seq = 0;
volatile uint32_t g_ekf_imu_push_count = 0;
volatile uint32_t g_ekf_imu_drop_count = 0;
volatile uint32_t g_ekf_pressure_push_count = 0;
volatile float    g_ekf_depth_meas = 0.0f;   // 换算后的深度测量值(m)，供 Ozone 对比 out.depth_m
//填数据
void EkfBridge_Init(void)
{
  UgEkfConfig cfg;
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

  /* TODO 上控制前实测填写 cfg.mag_declination_rad 等。 */
  UgEkf_Init(&cfg);

  memset(&s_imu_in, 0, sizeof(s_imu_in));
  s_imu_in_seq = 0;
  s_imu_last_seq = 0;
  s_pressure_last_tick = 0;
  g_ekf_depth_meas = 0.0f;
  memset(&g_state_estimate, 0, sizeof(g_state_estimate));
  s_snap_seq = 0;
}
//把一帧IMU暂存起来，在IMUtask里
void EkfBridge_SubmitImu(float gx, float gy, float gz,
                         float ax, float ay, float az, uint64_t t_us)
{
  taskENTER_CRITICAL();//写快照，这是用于防止读写冲突的地方
  /* 上一帧若尚未被 EKF 使用就被覆盖，记一次丢弃（观察是否跟不上 IMU 率） */
  if (s_imu_in_seq != s_imu_last_seq) {
    g_ekf_imu_drop_count++;
  }
  s_imu_in.gx = gx; s_imu_in.gy = gy; s_imu_in.gz = gz;
  s_imu_in.ax = ax; s_imu_in.ay = ay; s_imu_in.az = az;
  s_imu_in.t_us = t_us;
  s_imu_in.seq = ++s_imu_in_seq;
  taskEXIT_CRITICAL();
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
  /* 1) IMU：单槽 latest-wins，每次只取最新一帧，处理完即返回，保证 Step 一定走到 publish，
   *    不会被高速输入拖住活锁，输入快于 predict 时只丢帧，drop_count 增长 */
  // 后续可能需要实现ring-buffer
  EkfImuInput in;
  if (takeImuInput(&in)) {
    //EKF的Task，算法层
    UgEkf_PushImu(in.gx, in.gy, in.gz, in.ax, in.ay, in.az, in.t_us);
    g_ekf_imu_push_count++;
  }

  /* 2) 压力：仅在 valid 且 last_update_tick 变化时 push（对齐前 wrapper 内部会忽略）。
   *    本板深度计是【表压】(空气≈0)：depth = bar*1e5/(ρg)，不减大气压，
   *    故绕过 UgEkf_PushPressureBar（它按绝对压减 p_atm，会多出 -10m 偏置）。 */
  if (g_pressure_sensor_readout.pressure_valid &&
      g_pressure_sensor_readout.last_update_tick != s_pressure_last_tick) {
    s_pressure_last_tick = g_pressure_sensor_readout.last_update_tick;
    g_ekf_depth_meas = g_pressure_sensor_readout.pressure_bar * 1.0e5f / (kWaterRho * kGravity);
    UgEkf_PushDepth(g_ekf_depth_meas, GliderTime_NowUs());
    g_ekf_pressure_push_count++;
  }

  /* 3) v1 不 push mag，在等 hard/soft iron 标定；raw mag 仅在 task 层记录观察。 */

  /* 4) 拉输出，然后发布受保护快照 */
  UgEkfOutput out;
  UgEkf_GetOutput(&out);
  taskENTER_CRITICAL();//嵌入式的保护读写
  g_state_estimate.out = out;
  g_state_estimate.update_seq = ++s_snap_seq;
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
