#ifndef RUDDER_DRIVER_H
#define RUDDER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* DS820MG 的单位统一为 0.01 度；标称总行程 90 度，中位两侧各 45 度。 */
#define RUDDER_FULL_SCALE_CDEG  4500
#define RUDDER_LIMIT_CDEG       RUDDER_FULL_SCALE_CDEG
#define RUDDER_MIN_PULSE_US     1000U
#define RUDDER_CENTER_PULSE_US  1500U
#define RUDDER_MAX_PULSE_US     2000U
#define RUDDER_PERIOD_US        3003U

/* 安装标定项：确认实物方向/机械中位后改常量，不用调试器直接改输出寄存器。 */
#define RUDDER_DIRECTION        1
#define RUDDER_CENTER_TRIM_US   0

typedef enum
{
  RUDDER_SOURCE_NOT_STARTED = 0,
  RUDDER_SOURCE_CENTER = 1,
  RUDDER_SOURCE_ETHERNET = 2,
  RUDDER_SOURCE_DEBUG = 3
} RudderSource;

typedef struct
{
  uint32_t enabled;             /* Ozone 写 1 请求调试；写 0 释放，其他值不接受。 */
  int32_t target_angle_cdeg;     /* 先写目标再使能：1000=+10 度，-4500=-45 度。 */
} RudderDebugCommand;

typedef struct
{
  uint32_t pwm_started;
  uint32_t source;
  int16_t requested_angle_cdeg;
  int16_t applied_angle_cdeg;   /* 限幅后的下发目标，不是舵机实测反馈。 */
  uint16_t applied_pulse_us;
  uint8_t limit_active;
  uint32_t timer_clock_hz;
  uint32_t update_count;
} RudderState;

/* Ozone 观察入口；只有本驱动写入，其他任务通过 CopyState 读取一致快照。 */
extern volatile RudderState g_rudder_state;

/* Ozone 命令邮箱；默认关闭，不直接写 CCR。使用 int32 避免大输入窄化后反号。 */
extern volatile RudderDebugCommand g_rudder_debug;

/* 调度器启动前调用一次；TIM4_CH4 必须已由 CubeMX 配置为向上计数 PWM1。 */
HAL_StatusTypeDef RudderDriver_Init(TIM_HandleTypeDef *htim);

/* 只由 StartServoTask 调用。active=0 回中；active!=0 接受网线绝对角度目标。 */
void RudderDriver_Apply(int16_t angle_cdeg, uint8_t active);

/* 只由 StartServoTask 仲裁后调用，与 Apply 共用同一限幅和硬件写入路径。 */
void RudderDriver_ApplyDebug(int16_t angle_cdeg);

/* 只由 StartServoTask 每拍调用。allow=0 清除使能，防止冲突结束后旧调试命令续动。
   allow!=0 且 enabled==1 时返回目标；本函数只取命令，不产生 PWM 输出。 */
bool RudderDriver_GetDebugCommand(uint8_t allow, int16_t *out_angle_cdeg);

/* 任务上下文读取；false 表示空指针，pwm_started 表示 PWM 是否启动成功。 */
bool RudderDriver_CopyState(RudderState *out_state);

#endif /* RUDDER_DRIVER_H */
