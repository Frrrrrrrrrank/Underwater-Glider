/*
 * worm_axis.c —— 职责与设计取舍见 worm_axis.h 顶部说明。
 */
#include "worm_axis.h"
#include "motor_driver.h"

#include "FreeRTOS.h"
#include "task.h"

volatile WormAxisState g_worm_axis = {
  0, 0.0f, 0.0f, 0.0f,
  (uint8_t)WORM_REF_NONE,
  0U, 0U, 0U, 0U,
  0U,
  0, 0, 0,
  0U
};

/* 只有 ServoTask 调用 WormAxis_Step()，单一调用者，内部量用 static 即可。 */
static uint32_t s_last_valid_ms = 0U;
static uint8_t  s_have_valid    = 0U;

void WormAxis_Step(uint32_t now_ms)
{
  uint32_t valid;
  int32_t  pulse;
  uint16_t status_word;
  uint32_t silence_ms;
  uint8_t  fresh;
  int32_t  span;
  float    ratio;

  /*
   * 多字段快照必须一次取齐：Usart2Task 随时可能在两次读之间刷新整组数据，
   * 分开读会得到"旧 pulse 配新 status_word"的撕裂组合。做法与 main.c 取位移快照
   * 完全一致；临界区只有 3 次赋值，微秒级，不会影响 100Hz IMU 中断链。
   */
  taskENTER_CRITICAL();
  valid       = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].valid;
  status_word = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].status_word;
  pulse       = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].debug.pos_signed_32;
  taskEXIT_CRITICAL();

  if (valid != 0U)
  {
    s_last_valid_ms = now_ms;
    s_have_valid = 1U;
  }

  /* 无符号相减天然处理 HAL_GetTick 的 32 位回绕；开机尚未取得数据时直接按超时算。 */
  silence_ms = (s_have_valid != 0U) ? (now_ms - s_last_valid_ms)
                                    : WORM_AXIS_REF_LOST_MS;
  fresh = (silence_ms < WORM_AXIS_STALE_MS) ? 1U : 0U;

  /*
   * 零点参考作废判据。
   * 驱动器一旦掉电重启，0x000B/0x000C 从 0 重新计数，而质量块并没有移动 ——
   * 此时任何"跨上电比较位置"的用法都是错的。我们无法直接观测重启，只能观测通信中断：
   *     驱动器重启  =>  通信必然中断   (必要条件，不会漏报)
   *     通信中断    =/=> 驱动器重启    (ORE / 线缆抖动也会，故会误报)
   * 判据刻意保守：误报的代价只是要求重新建立一次参考，方向朝安全。
   */
  if (silence_ms >= WORM_AXIS_REF_LOST_MS)
  {
    if (g_worm_axis.reference_kind != (uint8_t)WORM_REF_NONE)
    {
      g_worm_axis.reference_kind = (uint8_t)WORM_REF_NONE;
      g_worm_axis.reference_lost_count++;
    }
  }
  else if ((fresh != 0U) &&
           (g_worm_axis.reference_kind == (uint8_t)WORM_REF_NONE))
  {
    /* 重新拿到数据：只敢声明"以驱动器上电点为参考"。极值必须跟着重置，
       否则会把两个不同参考系下的脉冲混在一起比大小。 */
    g_worm_axis.reference_kind = (uint8_t)WORM_REF_POWER_ON;
    g_worm_axis.pulse_min_seen = pulse;
    g_worm_axis.pulse_max_seen = pulse;
  }
  else
  {
    /* 参考系维持不变；已回零(WORM_REF_HOMED)不会被降级成 POWER_ON。 */
  }

  g_worm_axis.pulse = pulse;
  /* rev 只是给人核对细分用的诊断量：实测行程 145mm/33.577转 = 4.32mm/转 对不上
     任何标准导程，说明细分或导程与假设不符 —— 所以位置换算不走 rev 这条路。 */
  g_worm_axis.rev = (float)pulse / WORM_PULSES_PER_REV;
  g_worm_axis.data_fresh = fresh;
  /*
   * 0x0004 Bit1=静止/运动, Bit6=报警。
   * 判"电机停了没有"只能看 Bit1：0x0005/0x0006 转速寄存器在模式退出后不清零，
   * 会冻结在断电前最后一次采样值(实测 work_mode=0/静止 却读到 199/209，
   * 见 AGENT_STATUS 风险 9-quater)。
   */
  g_worm_axis.moving = ((status_word & 0x0002U) != 0U) ? 1U : 0U;
  g_worm_axis.alarm  = ((status_word & 0x0040U) != 0U) ? 1U : 0U;

  /* 极值先更新，下面的百分比才用得上本拍的最新端点。 */
  if ((fresh != 0U) && (g_worm_axis.reference_kind != (uint8_t)WORM_REF_NONE))
  {
    if (pulse < g_worm_axis.pulse_min_seen) { g_worm_axis.pulse_min_seen = pulse; }
    if (pulse > g_worm_axis.pulse_max_seen) { g_worm_axis.pulse_max_seen = pulse; }
  }

  /*
   * 行程百分比 —— 本模块的主输出，也是"电机现在处于总行程哪里"的直接答案。
   * 纯粹是脉冲比例，不含导程/细分的任何假设，因此比 rev/mm 都可靠。
   *
   * 端点直接取本次上电见过的极值：开机后把质量块跑一遍两端即完成自标定，
   * 不需要等下一轮的回零功能。跨度不足全行程 90% 时判为"还没跑过两端"，
   * 输出置 0 并由 span_valid=0 明示不可用，避免把半程当成全程。
   */
  span = g_worm_axis.pulse_max_seen - g_worm_axis.pulse_min_seen;
  g_worm_axis.span_pulses = span;
  if (span >= WORM_SPAN_MIN_PULSES)
  {
    ratio = (float)(pulse - g_worm_axis.pulse_min_seen) / (float)span;
    g_worm_axis.span_valid = 1U;
    g_worm_axis.stroke_percent = 100.0f * ratio;
    g_worm_axis.mm_from_front = WORM_STROKE_MM * ratio;
  }
  else
  {
    g_worm_axis.span_valid = 0U;
    g_worm_axis.stroke_percent = 0.0f;
    g_worm_axis.mm_from_front = 0.0f;
  }

  g_worm_axis.update_count++;
}

void WormAxis_CopyState(WormAxisState *out_state)
{
  if (out_state == NULL)
  {
    return;
  }

  /* 整体复制，避免与 ServoTask 的写入交错。结构体仅 40 字节，临界区极短。 */
  taskENTER_CRITICAL();
  *out_state = *(const WormAxisState *)&g_worm_axis;
  taskEXIT_CRITICAL();
}
