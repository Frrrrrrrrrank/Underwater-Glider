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
  0U,
  /* 回零 */
  (uint8_t)WORM_HOME_IDLE, (uint8_t)WORM_HOME_FAIL_NONE, 0, 0U
};

/* main.c 里定义为 volatile（Usart3Task 每 10ms 写、本模块读），此处必须同型。 */
extern volatile uint8_t g_limit_switch_states[4];

/* 只有 ServoTask 调用 WormAxis_Step()，单一调用者，内部量用 static 即可。 */
static uint32_t s_last_valid_ms = 0U;
static uint8_t  s_have_valid    = 0U;

/* 回零：请求/取消由以太网任务置位、ServoTask 消费。单字节读写在 M7 上原子，
   且这两个标志只是"边沿触发一次"的性质，不需要临界区。 */
static volatile uint8_t s_home_request = 0U;
static volatile uint8_t s_home_cancel  = 0U;
/* 本拍回零想要的转速，只由 WormAxis_HomingStep 写、WormAxis_GetHomingWormRpm 读，
   两者同在 ServoTask 上下文，无并发。 */
static int32_t  s_home_rpm = 0;
static uint32_t s_home_phase_start_ms = 0U;

static uint8_t WormAxis_HomingRunning(void)
{
  return ((g_worm_axis.home_state == (uint8_t)WORM_HOME_SEEK) ||
          (g_worm_axis.home_state == (uint8_t)WORM_HOME_BACKOFF)) ? 1U : 0U;
}

static void WormAxis_HomingFail(WormHomingFail reason)
{
  g_worm_axis.home_state = (uint8_t)WORM_HOME_FAILED;
  g_worm_axis.home_fail = (uint8_t)reason;
  s_home_rpm = 0;
}

/*
 * 两段式回零状态机。刻意写成【线性】的：SEEK -> BACKOFF -> DONE，只有超时/取消/
 * 数据失效三条支线通向 FAILED，没有任何回环。
 * 2026-07-31 那次事故的教训是"别把大状态机塞进收发路径"，这里的做法是：
 * 状态机完全不发帧、不写执行器，只根据 GPIO 电平和时间推进，产出一个建议转速。
 *
 * 依赖 motor_driver 已有的限位保护：撞到前限位时 Motor_IsLimitBlocked() 会把速度
 * 钳成 0 并写 0x0039=0 释放电机，所以本状态机【不需要】自己做撞限位急停。
 */
static void WormAxis_HomingStep(uint32_t now_ms, int32_t pulse, uint8_t fresh)
{
  /* g_limit_switch_states 约定：0 = 压下(触发)，1 = 未触发。 */
  uint8_t front_hit =
      (g_limit_switch_states[WORM_FRONT_LIMIT_INDEX] == 0U) ? 1U : 0U;

  /* --- 取消优先：任何时刻都能立即中止并停机 --- */
  if (s_home_cancel != 0U)
  {
    s_home_cancel = 0U;
    s_home_request = 0U;
    if (WormAxis_HomingRunning() != 0U)
    {
      WormAxis_HomingFail(WORM_HOME_FAIL_CANCELLED);
    }
    s_home_rpm = 0;
  }

  /* --- 启动请求：从 IDLE/DONE/FAILED 任何状态都能重新开始 --- */
  if (s_home_request != 0U)
  {
    s_home_request = 0U;
    g_worm_axis.home_state = (uint8_t)WORM_HOME_SEEK;
    g_worm_axis.home_fail = (uint8_t)WORM_HOME_FAIL_NONE;
    s_home_phase_start_ms = now_ms;
    s_home_rpm = 0; /* 本拍先不动，下一拍才开始走 */
    return;
  }

  /* --- 运行途中位置数据失效就立刻停：绝不摸黑往限位上撞 --- */
  if ((WormAxis_HomingRunning() != 0U) && (fresh == 0U))
  {
    WormAxis_HomingFail(WORM_HOME_FAIL_DATA_STALE);
    return;
  }

  switch ((WormHomingState)g_worm_axis.home_state)
  {
    case WORM_HOME_SEEK:
      if (front_hit != 0U)
      {
        /* 已在前限位内（含"启动时就压着开关"的情况），直接进退出段。 */
        g_worm_axis.home_state = (uint8_t)WORM_HOME_BACKOFF;
        s_home_phase_start_ms = now_ms;
        s_home_rpm = WORM_HOME_BACKOFF_RPM;
      }
      else if ((now_ms - s_home_phase_start_ms) >= WORM_HOME_SEEK_TIMEOUT_MS)
      {
        WormAxis_HomingFail(WORM_HOME_FAIL_SEEK_TIMEOUT);
      }
      else
      {
        /* 负向 = 朝前限位（实测：-RPM 使 pulse 递减、走向 PD7）。 */
        s_home_rpm = -WORM_HOME_SEEK_RPM;
      }
      break;

    case WORM_HOME_BACKOFF:
      if (front_hit == 0U)
      {
        /*
         * 开关刚刚释放 —— 以【释放沿】为零点。
         * 释放沿比触发沿重复性好得多：触发点受撞击速度和过冲影响，
         * 释放点只由开关自身回差决定，而且是慢速通过的。
         * 手册方式 17(负限位回零)用的就是这个策略。
         */
        g_worm_axis.pulse_home_front = pulse;
        /* 合成行程端点：一端实测、另一端由已知全行程常数推出，
           于是 stroke_percent 立刻可用，不必再跑另一端。 */
        g_worm_axis.pulse_min_seen = pulse;
        g_worm_axis.pulse_max_seen = pulse + WORM_STROKE_PULSES;
        g_worm_axis.reference_kind = (uint8_t)WORM_REF_HOMED;
        g_worm_axis.home_state = (uint8_t)WORM_HOME_DONE;
        g_worm_axis.home_done_count++;
        s_home_rpm = 0;
      }
      else if ((now_ms - s_home_phase_start_ms) >= WORM_HOME_BACKOFF_TIMEOUT_MS)
      {
        WormAxis_HomingFail(WORM_HOME_FAIL_BACKOFF_TIMEOUT);
      }
      else
      {
        s_home_rpm = WORM_HOME_BACKOFF_RPM;
      }
      break;

    default:
      /* IDLE / DONE / FAILED：不产生任何转速。 */
      s_home_rpm = 0;
      break;
  }
}

void WormAxis_RequestHoming(void)
{
  s_home_cancel = 0U;
  s_home_request = 1U;
}

void WormAxis_CancelHoming(void)
{
  s_home_request = 0U;
  s_home_cancel = 1U;
}

uint8_t WormAxis_GetHomingWormRpm(int32_t *out_rpm)
{
  if (WormAxis_HomingRunning() == 0U)
  {
    if (out_rpm != NULL) { *out_rpm = 0; }
    return 0U;
  }
  if (out_rpm != NULL) { *out_rpm = s_home_rpm; }
  return 1U;
}

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
      /* 参考系没了，之前那次回零的结果也就不成立了，退回未回零。
         注意不要动 FAILED：那是给人看失败原因的，等下次回零请求时才清。 */
      if (g_worm_axis.home_state == (uint8_t)WORM_HOME_DONE)
      {
        g_worm_axis.home_state = (uint8_t)WORM_HOME_IDLE;
      }
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

  /*
   * 回零状态机。放在位置换算之前，这样本拍刚完成的回零立刻生效。
   * 它只产出一个建议转速，由 StartServoTask 决定要不要采纳。
   */
  WormAxis_HomingStep(now_ms, pulse, fresh);

  /*
   * 极值跟踪【只在 POWER_ON 自标定模式下做】。
   * 一旦回零成功进入 HOMED，端点就由"回零点 + 已知全行程常数"唯一确定，
   * 不能再让它随实际走位漂移 —— 否则质量块稍微越过释放沿一点，
   * 零点就被拉走了，绝对位置的意义也就没了。
   */
  if ((fresh != 0U) &&
      (g_worm_axis.reference_kind == (uint8_t)WORM_REF_POWER_ON))
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
