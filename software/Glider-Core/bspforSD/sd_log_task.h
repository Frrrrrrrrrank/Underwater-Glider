/*
*********************************************************************************************************
*
*	模块名称 : SD 卡日志任务
*	文件名称 : sd_log_task.h
*	说    明 : FreeRTOS 任务，每隔一分钟调用 EthernetUartDriver_UpdateSystemSnapshot()
*			   刷新 g_ethernet_system_snapshot，并把该快照与时间信息追加写入 SD 卡根目录
*			   的 armfly.txt，每条记录单独占一行。
*
*********************************************************************************************************
*/
#ifndef SD_LOG_TASK_H
#define SD_LOG_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 落盘周期：每分钟一次（configTICK_RATE_HZ = 1000，tick == ms） */
#define SD_LOG_TASK_PERIOD_MS   (60U * 1000U)

/* 写入的目标文件（SD 卡根目录下） */
#define SD_LOG_FILE_NAME        "armfly.txt"

/* FreeRTOS 任务入口，在 main.c 用 osThreadNew 注册。
 * 内部先注册 SD 卡驱动，然后进入周期循环：每分钟刷新一次系统快照并追加落盘。 */
void StartSdLogTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SD_LOG_TASK_H */
