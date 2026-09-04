/*
 * worm_axis.h
 *
 * 蜗杆(移动质量块)轴的位置读出层。职责只有两件：
 *  1) 把 motor_driver 轮询回来的原始脉冲(0x000B/0x000C)翻译成圈数/毫米；
 *  2) 回答"这个数现在可不可信" —— 数据是否新鲜、零点参考是否还成立。
 *
 * 本模块【不】发任何 MODBUS 帧、【不】写任何执行器、【不】含状态机。
 * 由 StartServoTask 每 20ms 调一次 WormAxis_Step()，只读 g_umotor 与系统时基。
 *
 * 关于零点：UMLR35 是增量式闭环步进，0x000B/0x000C 的零点是"驱动器上电那一刻电机在哪"
 * 或"回原点完成后的位置"(手册 4.2.1)。因此本轮(尚未实现回零)只能给出【相对上电点】的
 * 位移，reference_kind 把这件事显式表达出来，禁止上层把它当绝对位置用。
 */
#ifndef WORM_AXIS_H
#define WORM_AXIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 蜗杆电机在 g_umotor.nodes[] 里的槽位，须与 main.c 的 RC_WORM_MOTOR_SLOT 一致。 */
#define WORM_AXIS_MOTOR_SLOT 0U

/*
 * 驱动器闭环细分(手册 0x0028)，出厂默认 10000 Pul/rev。本轮不读该寄存器；
 * 若现场改过细分，rev 会整体缩放 —— 台架上"实测导程 = 量得行程 / Δrev"与图纸对不上，
 * 就是这里出了问题。
 */
#define WORM_PULSES_PER_REV 10000.0f

/*
 * 【2026-08-20 台架实测】行程标定对：两限位触发点间距 145mm，对应脉冲差 335772
 * (前限位 -2 ~ 后限位 +335770；该次驱动器恰好在前限位上电)。
 *
 * 位置换算直接用这一对实测值，【不依赖】丝杆导程与驱动器细分这两个未知量：
 * 145mm / 33.577转 = 4.32mm/转，对不上任何标准导程(2/4/5/8/10)，说明导程或细分
 * 至少有一个与假设不符，因此绕开它们。日后量准 L_mm 或读到 0x0028，只需改这两个数。
 *
 * 注意 stroke_percent 连 WORM_STROKE_MM 都不需要 —— 纯脉冲比例，是最可靠的输出；
 * WORM_STROKE_MM 只用来把百分比换成人看的毫米。
 */
#define WORM_STROKE_MM     145.0f
#define WORM_STROKE_PULSES 335772

/* 已探明跨度达到全行程 90% 才认为两端都跑到过，min/max 方可当行程端点使用。 */
#define WORM_SPAN_MIN_PULSES ((int32_t)((WORM_STROKE_PULSES / 10) * 9))

/*
 * 数据新鲜 / 参考作废的两级超时(ms)。必须显著大于 100ms 轮询周期：
 * USART2 目前有约 1.4% 的 ORE 丢帧(AGENT_STATUS 风险 9-ter)，每次丢帧都会把
 * node->valid/comm_ok 打到 0 一拍。若按电平或下降沿判定会每隔几秒误报一次，
 * 因此一律用超时判据。
 */
#define WORM_AXIS_STALE_MS    300U   /* 连续 3 个轮询周期无成功数据 => 不新鲜 */
#define WORM_AXIS_REF_LOST_MS 1000U  /* 连续 1 秒无成功数据 => 零点参考作废 */

typedef enum
{
  WORM_REF_NONE     = 0U, /* 无参考：通信没起来，或刚判定驱动器可能重启过 */
  WORM_REF_POWER_ON = 1U, /* 参考=驱动器上电点，只能在本次上电内做相对比较 */
  WORM_REF_HOMED    = 2U  /* 参考=回零点，绝对位置有效。本轮不会出现，留给下一轮 */
} WormReferenceKind;

typedef struct
{
  int32_t  pulse;                /* 0x000B/0x000C 拼出的有符号脉冲，原始真值 */
  float    rev;                  /* pulse/10000，仅作细分交叉核对用的诊断量 */
  float    stroke_percent;       /* 【主输出】0=前限位, 100=后限位; span_valid=0 时为 0 */
  float    mm_from_front;        /* 距前限位毫米 = stroke_percent/100 × WORM_STROKE_MM */
  uint8_t  reference_kind;       /* WormReferenceKind */
  uint8_t  data_fresh;           /* 0 = 轮询已陈旧，pulse 是老值 */
  uint8_t  span_valid;           /* 1 = 本次上电已跑过两端，min/max 可当行程端点 */
  uint8_t  moving;               /* 0x0004 Bit1。判"停了没有"只能看它，不能看转速寄存器 */
  uint8_t  alarm;                /* 0x0004 Bit6。撞限位后置位，下条指令的使能沿会自清 */
  uint32_t reference_lost_count; /* 参考系被作废的次数 */
  int32_t  pulse_min_seen;       /* 本参考系内见过的极值，台架标定直接读这两个数 */
  int32_t  pulse_max_seen;
  int32_t  span_pulses;          /* pulse_max_seen - pulse_min_seen，已探明的行程跨度 */
  uint32_t update_count;         /* Step 调用次数，确认本模块真的在跑 */
} WormAxisState;

/* Ozone 观察点；上层若要用位置，必须同时判 data_fresh 与 reference_kind。 */
extern volatile WormAxisState g_worm_axis;

/* 由 StartServoTask 每 20ms 调用一次；now_ms 用 HAL_GetTick()。 */
void WormAxis_Step(uint32_t now_ms);

/*
 * 取一致性快照。遥测/控制等其它任务【必须】走这里，不要散读 g_worm_axis：
 * ServoTask 每 20ms 会整组刷新，分字段散读可能拿到"旧 pulse 配新 span_valid"
 * 这种撕裂组合。与 EkfBridge_CopyState 是同一条约定。
 */
void WormAxis_CopyState(WormAxisState *out_state);

#ifdef __cplusplus
}
#endif

#endif /* WORM_AXIS_H */
