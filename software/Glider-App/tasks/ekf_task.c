/* ekf_task.c 定义见 ekf_task.h */
#include "ekf_task.h"
#include "ekf_bridge.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* 任务句柄在 main.c 定义（osThreadNew 返回值） */
extern osThreadId_t ekfTaskHandle;
//暴露给OZone调试用，分别为EKF task loop计数器，栈水位，原始mag数据
volatile uint32_t g_ekf_task_loop_count = 0;
volatile uint32_t g_ekf_stack_high_water_words = 0;
volatile float    g_ekf_mag_raw[3] = {0.0f, 0.0f, 0.0f};
volatile float    g_ekf_imu_hz = 0.0f;   //实测 IMU 真实新帧率(Hz)，每秒刷新一次

//IMU 帧率测量窗口（仅 EkfTask_PublishImuRaw 单写）
static uint32_t   s_imu_hz_frames = 0;
static uint64_t   s_imu_hz_win_start_us = 0;

void EkfTask_PublishImuRaw(const IMU_RawData *raw, uint64_t t_us)
{
  if (raw == NULL) {
    return;
  }
  EkfBridge_SubmitImu(raw->gyro.gx, raw->gyro.gy, raw->gyro.gz,
                      raw->accel.ax, raw->accel.ay, raw->accel.az, t_us);

  /* 测量 IMU 真实新帧率：每累计 1 秒用帧数算一次 Hz。本函数仅在真正新帧时被调
     （main.c 已用 raw.updated 门控），t_us 为帧时间戳，统一时基。 */
  s_imu_hz_frames++;
  if (s_imu_hz_win_start_us == 0) {
    s_imu_hz_win_start_us = t_us;
  } else if (t_us - s_imu_hz_win_start_us >= 1000000ULL) {
    g_ekf_imu_hz = (float)s_imu_hz_frames * 1000000.0f / (float)(t_us - s_imu_hz_win_start_us);
    s_imu_hz_frames = 0;
    s_imu_hz_win_start_us = t_us;
  }

  /* 目前raw mag 仅记录观察，不进 EKF，等 hard/soft iron 标定后再启用 yaw 更新 */
  g_ekf_mag_raw[0] = raw->mag.mx;
  g_ekf_mag_raw[1] = raw->mag.my;
  g_ekf_mag_raw[2] = raw->mag.mz;

  if (ekfTaskHandle != NULL) {
    (void)osThreadFlagsSet(ekfTaskHandle, EKF_FLAG_IMU);//标志位，唤醒EKFTask
  }
}

void StartEkfTask(void *argument)
{
  (void)argument;

  EkfBridge_Init();

  for (;;) {
    /* 事件触发，以及心跳包兜底，即使无 IMU 帧也周期性输入压力 / 刷新快照 / 更新栈水位 */
    (void)osThreadFlagsWait(EKF_FLAG_IMU, osFlagsWaitAny, EKF_TASK_FALLBACK_MS);

    g_ekf_task_loop_count++;
    EkfBridge_Step();
    g_ekf_stack_high_water_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  }
}
