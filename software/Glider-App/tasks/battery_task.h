#ifndef BATTERY_TASK_H
#define BATTERY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 由 main.c 注册；本任务独占电池 ADC 采样和状态更新。 */
void StartBatteryTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
