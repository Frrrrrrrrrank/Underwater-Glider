#include "battery_task.h"
#include "battery_monitor.h"
#include "FreeRTOS.h"
#include "task.h"

void StartBatteryTask(void *argument)
{
  const TickType_t period = pdMS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS);
  TickType_t last_wake = xTaskGetTickCount();
  (void)argument;
  for (;;)
  {
    TickType_t now;
    /* 与网线连接和遥测订阅无关，始终按约 10Hz 获取电池总压。 */
    BatteryMonitor_Poll();
    now = xTaskGetTickCount();
    /* 长校准或抢占后重新对齐周期，避免追赶旧时间表而瞬间重复采样。 */
    if ((TickType_t)(now - last_wake) >= period)
    {
      last_wake = now;
    }
    vTaskDelayUntil(&last_wake, period);
  }
}
