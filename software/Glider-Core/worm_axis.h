/*
 * worm_axis.h
 *
 * 蜗杆(移动质量块)轴的位置与回零层：
 *  1) 保留驱动器原始脉冲，换算机头端 0%、机尾端 100% 的行程位置；
 *  2) 判断数据新鲜度与零点参考，并产生回零所需的逻辑转速。
 *
 * 本模块不发 MODBUS 帧、不写执行器。StartServoTask 每 20ms 调用 WormAxis_Step()，
 * 再仲裁回零/控制请求；pitch 安装方向只在该任务的最终下发处转换一次。
 *
 * 关于零点：UMLR35 是增量式闭环步进，0x000B/0x000C 的零点是"驱动器上电那一刻电机在哪"
 * 或"回原点完成后的位置"(手册 4.2.1)。本模块软件回零不清驱动器计数，而是记录前限位
 * 释放沿的原始脉冲；reference_kind 区分上电参考和已回零参考。
 */
#ifndef WORM_AXIS_H
#define WORM_AXIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 蜗杆电机在 g_umotor.nodes[] 里的槽位，须与 main.c 的 RC_WORM_MOTOR_SLOT 一致。 */
#define WORM_AXIS_MOTOR_SLOT 0U

/* 新机构：逻辑负 RPM 向机头、正 RPM 向机尾；驱动器实际方向相反。
 * 仅供 StartServoTask 最终发送速度时使用，PID/手动/回零不得再次取反。
 * 原始脉冲不取反：新机头端脉冲较大，向机尾运动时脉冲递减。 */
#define WORM_INSTALL_SIGN (-1)

/*
 * 驱动器闭环细分(手册 0x0028)，出厂默认 10000 Pul/rev。本轮不读该寄存器；
 * 若现场改过细分，rev 会整体缩放 —— 台架上"实测导程 = 量得行程 / Δrev"与图纸对不上，
 * 就是这里出了问题。
 */
#define WORM_PULSES_PER_REV 10000.0f

/*
 * 【2026-10-16 新机构】用户确认完整行程：原始极值 -281323 ~ -2，跨度 281321。
 * 极值只属于该次驱动器上电参考，不作为固定坐标写入程序。
 *
 * WORM_STROKE_MM 暂保留旧机构实测 145mm，新机构长度尚待实测；此时毫米量仅为估计。
 * 回零以释放沿为零点，触发点间的跨度在实机复核后可再微调。
 *
 * 注意 stroke_percent 连 WORM_STROKE_MM 都不需要 —— 纯脉冲比例，是最可靠的输出；
 * WORM_STROKE_MM 只用来把百分比换成人看的毫米。
 */
#define WORM_STROKE_MM     58.8f
#define WORM_STROKE_PULSES 281321 //v1的机械设计原来是 335772

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
  WORM_REF_HOMED    = 2U  /* 参考=回零点，绝对位置有效（回零成功后进入） */
} WormReferenceKind;

/* ================= 回零(Homing) ================= */

/* 新机头限位为 PB4，压下时 states[1]=0，挡驱动器实际正 RPM。
 * 机尾 PD7/states[3] 挡实际负 RPM；Motor_IsLimitBlocked 的条件保持原样。 */
#define WORM_FRONT_LIMIT_INDEX 1U

/*
 * 两段式回零速度幅值：SEEK 输出逻辑 -350，BACKOFF 输出逻辑 +35。
 * 经 StartServoTask 唯一安装转换后，驱动器实际收到 +350 / -35 RPM。
 * 慢速退出用于减小位置轮询延迟带来的零点误差；新机构重复性仍待实测。
 */
#define WORM_HOME_SEEK_RPM    350
#define WORM_HOME_BACKOFF_RPM  35

/* 分段超时沿用原设置：趋近 30s、退出 15s。超时即判失败，回零请求转速归零。 */
#define WORM_HOME_SEEK_TIMEOUT_MS    30000U
#define WORM_HOME_BACKOFF_TIMEOUT_MS 15000U

typedef enum
{
  WORM_HOME_IDLE    = 0U, /* 未在回零 */
  WORM_HOME_SEEK    = 1U, /* 朝机头限位趋近，等 PB4 触发 */
  WORM_HOME_BACKOFF = 2U, /* 反向慢速退出，等 PB4 释放 */
  WORM_HOME_DONE    = 3U, /* 回零完成，零点已记录 */
  WORM_HOME_FAILED  = 4U  /* 失败，见 home_fail */
} WormHomingState;

typedef enum
{
  WORM_HOME_FAIL_NONE            = 0U,
  WORM_HOME_FAIL_SEEK_TIMEOUT    = 1U, /* 一直没撞到前限位 */
  WORM_HOME_FAIL_BACKOFF_TIMEOUT = 2U, /* 退不出前限位 */
  WORM_HOME_FAIL_DATA_STALE      = 3U, /* 回零途中位置数据失效，不敢再动 */
  WORM_HOME_FAIL_CANCELLED       = 4U, /* 被上位机或 PID 接管取消 */
  WORM_HOME_FAIL_POSITION_RANGE  = 5U  /* 回零推算的端点超出 int32 脉冲范围 */
} WormHomingFail;

typedef struct
{
  int32_t  pulse;                /* 0x000B/0x000C 拼出的有符号脉冲，原始真值 */
  float    rev;                  /* pulse/10000，仅作细分交叉核对用的诊断量 */
  float    stroke_percent;       /* 【主输出】0=前限位, 100=后限位; span_valid=0 时为 0 */
  float    mm_from_front;        /* 距前限位毫米 = stroke_percent/100 × WORM_STROKE_MM */
  uint8_t  reference_kind;       /* WormReferenceKind */
  uint8_t  data_fresh;           /* 0 = 轮询已陈旧，pulse 是老值 */
  uint8_t  span_valid;           /* 1 = 已探明足够跨度或回零已合成有效端点 */
  uint8_t  moving;               /* 0x0004 Bit1。判"停了没有"只能看它，不能看转速寄存器 */
  uint8_t  alarm;                /* 0x0004 Bit6。撞限位后置位，下条指令的使能沿会自清 */
  uint32_t reference_lost_count; /* 参考系被作废的次数 */
  int32_t  pulse_min_seen;       /* 本参考系内见过的极值，台架标定直接读这两个数 */
  int32_t  pulse_max_seen;
  int32_t  span_pulses;          /* pulse_max_seen - pulse_min_seen，已探明的行程跨度 */
  uint32_t update_count;         /* Step 调用次数，确认本模块真的在跑 */
  /* --- 回零 --- */
  uint8_t  home_state;           /* WormHomingState */
  uint8_t  home_fail;            /* WormHomingFail，仅在 home_state=FAILED 时有意义 */
  int32_t  pulse_home_front;     /* 回零点：前限位释放沿处的脉冲值 */
  uint32_t home_done_count;      /* 成功回零次数 */
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

/*
 * 请求 / 取消一次回零。以太网任务收到上位机命令后调用，只置标志，不做任何动作，
 * 真正的状态推进全部发生在 ServoTask 的 WormAxis_Step() 里。
 * 重复请求会重新开始；DONE/FAILED 状态下再次请求也能重来。
 */
void WormAxis_RequestHoming(void);
void WormAxis_CancelHoming(void);

/*
 * 回零期间由 StartServoTask 查询：返回 1 表示回零正在进行、需要接管蜗杆，
 * 此时 *out_rpm 是逻辑转速（负向机头、正向机尾），最终下发时才转换安装方向。
 *
 * 本模块【绝不】自己调 UMotor_* —— 执行器唯一写点仍然是 StartServoTask，
 * 否则就出现两个写者，违反本仓库"唯一仲裁点"的核心约定。
 * 必须在同一个任务里紧跟 WormAxis_Step() 之后调用，取的才是本拍刚算出的结果。
 */
uint8_t WormAxis_GetHomingWormRpm(int32_t *out_rpm);

#ifdef __cplusplus
}
#endif

#endif /* WORM_AXIS_H */
