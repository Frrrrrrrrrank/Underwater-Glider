/*
*********************************************************************************************************
*
*	模块名称 : SD 卡日志任务
*	文件名称 : sd_log_task.c
*	说    明 : 每隔一分钟刷新系统快照 g_ethernet_system_snapshot 并把其内容与时间信息
*			   追加写入 SD 卡根目录的 armfly.txt。每条记录单独占一行（以 \r\n 结束）。
*			   本任务是唯一访问 FatFS 的任务（ffconf.h 中 _FS_REENTRANT = 0，非线程安全），
*			   每次落盘都完整 mount → open(append) → write → close → unmount。
*
*********************************************************************************************************
*/
#include "bsp.h"              /* stm32h7xx_hal.h(ALIGN_32BYTES) + SD 底层 BSP */
#include "ff.h"               /* FatFS 文件系统 */
#include "ff_gen_drv.h"       /* FATFS_LinkDriver */
#include "sd_diskio_dma.h"    /* SD_Driver */
#include "cmsis_os2.h"        /* osDelayUntil / osKernelGetTickCount */
#include "ethernet_uart_driver.h" /* g_ethernet_system_snapshot / 刷新函数 */
#include "sd_log_task.h"
#include <stdio.h>
#include <string.h>

/*
 * FatFS 的工作区 / 文件对象需放在 AXI SRAM(RAM_D1)，SDMMC1 的 DMA 仅能访问该区域，
 * 且按 32 字节对齐配合 Cache。与 demo_sd_fatfs.c 中 fs/file 的放置方式一致。
 */
ALIGN_32BYTES(__attribute__((section(".RAM_D1"))) static FATFS s_sd_log_fs);
ALIGN_32BYTES(__attribute__((section(".RAM_D1"))) static FIL   s_sd_log_file);
ALIGN_32BYTES(__attribute__((section(".RAM_D1"))) static char  s_sd_log_line[768]);

static char    s_sd_log_path[4] = {0};  /* SD 卡逻辑盘符路径，如 "0:/" */
static uint8_t s_sd_log_linked  = 0U;   /* 驱动是否已注册 */

/*
*********************************************************************************************************
*	函 数 名: SdLog_LinkOnce
*	功能说明: 只注册一次 SD 卡 FatFS 驱动，拿到逻辑盘符路径 s_sd_log_path。
*********************************************************************************************************
*/
static void SdLog_LinkOnce(void)
{
  if (s_sd_log_linked == 0U)
  {
    if (FATFS_LinkDriver(&SD_Driver, s_sd_log_path) == 0U)
    {
      s_sd_log_linked = 1U;
    }
  }
}

/*
*********************************************************************************************************
*	函 数 名: SdLog_WriteSnapshotRecord
*	功能说明: 刷新系统快照，拼一整行（时间信息 + 结构体全部字段），追加写入 armfly.txt。
*	形    参: 无
*	返 回 值: 无（写盘失败时静默返回，不影响任务继续运行）
*********************************************************************************************************
*/
static void SdLog_WriteSnapshotRecord(void)
{
  FRESULT res;
  UINT    bw = 0U;
  int     n;
  size_t  wlen;
  uint32_t tick_ms, ms, ss, mm, hh, dd;
  char    path[16];
  const volatile EthernetUartSystemSnapshot *s = &g_ethernet_system_snapshot;

  if (s_sd_log_linked == 0U)
  {
    return;
  }

  /* 1) 刷新系统快照（函数内部只是散读各全局镜像，线程上下文调用安全） */
  EthernetUartDriver_UpdateSystemSnapshot();

  /* 2) 时间信息：系统运行时间(ms)，换算为 天 时:分:秒.毫秒 */
  tick_ms = osKernelGetTickCount();   /* configTICK_RATE_HZ = 1000，tick 即 ms */
  ms = tick_ms % 1000U;
  ss = (tick_ms / 1000U) % 60U;
  mm = (tick_ms / 60000U) % 60U;
  hh = (tick_ms / 3600000U) % 24U;
  dd = tick_ms / 86400000U;

  /* 3) 挂载文件系统 */
  res = f_mount(&s_sd_log_fs, s_sd_log_path, 0);
  if (res != FR_OK)
  {
    return;
  }

  /* 4) 以追加方式打开（不存在则自动创建），保证每次写入接在文件末尾 */
  (void)snprintf(path, sizeof(path), "%s%s", s_sd_log_path, SD_LOG_FILE_NAME);
  res = f_open(&s_sd_log_file, path, FA_OPEN_APPEND | FA_WRITE);
  if (res != FR_OK)
  {
    (void)f_mount(NULL, s_sd_log_path, 0);
    return;
  }

  /* 5) 拼一整行：时间戳 + 快照全部字段，末尾 \r\n，保证每条记录单独分行 */
  n = snprintf(s_sd_log_line, sizeof(s_sd_log_line),
    "[%lud %02lu:%02lu:%02lu.%03lu tick=%lums] "
    "alive=%lu alarm=0x%04lX "
    "gnrmc(lat_e6=%ld lon_e6=%ld valid=%u) "
    "disp_mm_x10=%d(valid=%u) "
    "imu(roll_x10=%d pitch_x10=%d yaw_x10=%d height_dm=%d temp_x10=%d) "
    "press(bar_x100=%u temp_x10=%d valid=%u) "
    "pump_fb_rpm=%d umotor0_rpm=%d umotor1_rpm=%d "
    "servo(pulse_us=%u mode=%u) "
    "limit_bits=0x%02X pump(spd_trig=%u dir_rev=%u) "
    "ekf(pN_cm=%ld pE_cm=%ld depth_mm=%ld roll_x10=%d pitch_x10=%d yaw_x10=%d status=%u valid=%u)"
    "\r\n",
    (unsigned long)dd, (unsigned long)hh, (unsigned long)mm,
    (unsigned long)ss, (unsigned long)ms, (unsigned long)tick_ms,
    (unsigned long)s->alive_counter, (unsigned long)s->alarm_bits,
    (long)s->gnrmc_latitude_e6, (long)s->gnrmc_longitude_e6, (unsigned)s->gnrmc_valid,
    (int)s->displacement_mm_x10, (unsigned)s->displacement_valid,
    (int)s->imu_roll_x10, (int)s->imu_pitch_x10, (int)s->imu_yaw_x10,
    (int)s->imu_height_dm, (int)s->imu_temp_c_x10,
    (unsigned)s->pressure_bar_x100, (int)s->pressure_temp_c_x10, (unsigned)s->pressure_valid,
    (int)s->pump_feedback_rpm, (int)s->umotor0_feedback_rpm, (int)s->umotor1_feedback_rpm,
    (unsigned)s->servo_pulse_us, (unsigned)s->servo_mode,
    (unsigned)s->limit_switch_bits, (unsigned)s->pump_speed_triggered,
    (unsigned)s->pump_direction_reversed,
    (long)s->ekf_pN_cm, (long)s->ekf_pE_cm, (long)s->ekf_depth_mm,
    (int)s->ekf_roll_x10, (int)s->ekf_pitch_x10, (int)s->ekf_yaw_x10,
    (unsigned)s->ekf_status, (unsigned)s->ekf_valid);

  if (n > 0)
  {
    /* snprintf 返回“本应写入”的长度，可能大于缓冲区；截断到实际内容长度 */
    wlen = ((size_t)n >= sizeof(s_sd_log_line)) ? (sizeof(s_sd_log_line) - 1U) : (size_t)n;
    (void)f_write(&s_sd_log_file, s_sd_log_line, (UINT)wlen, &bw);
    (void)f_sync(&s_sd_log_file);   /* 立即刷 FAT，掉电也不丢已写记录 */
  }

  /* 6) 关闭文件并卸载文件系统 */
  (void)f_close(&s_sd_log_file);
  (void)f_mount(NULL, s_sd_log_path, 0);
}

/*
*********************************************************************************************************
*	函 数 名: StartSdLogTask
*	功能说明: SD 卡日志任务入口。注册驱动后进入循环，每分钟落盘一次系统快照。
*	形    参: argument - 未使用
*	返 回 值: 无（任务函数不返回）
*********************************************************************************************************
*/
void StartSdLogTask(void *argument)
{
  uint32_t wake;
  (void)argument;

  /* 注册 SD 卡 FatFS 驱动，得到逻辑盘符路径（如 "0:/"） */
  SdLog_LinkOnce();

  /* 用绝对唤醒时刻做无累积漂移的一分钟周期 */
  wake = osKernelGetTickCount();
  for (;;)
  {
    wake += SD_LOG_TASK_PERIOD_MS;
    (void)osDelayUntil(wake);
    SdLog_WriteSnapshotRecord();
  }
}

/***************************** (END OF FILE) *********************************/
