/*
 * zigzag_control.h
 *
 * 深度—浮力/俯仰并行外环与俯仰内环控制器的纯计算层。
 *
 * 单位约定：
 *   - depth、vertical_velocity：m、m/s，NED 向下为正；
 *   - 油囊线位移：mm，数值越大表示浮力越大；
 *   - pitch：rad，低头为负、抬头为正；
 *   - 输出蜗杆命令：有符号 RPM；
 *   - 推进器：有符号百分比，仅作为开环前馈，不进入 PID 公式。
 *
 * 该模块不读取 EKF 全局变量，也不调用任何电机、泵或推进器驱动。
 * 调用者负责提供状态快照，并由唯一的执行器仲裁层应用输出。
 */
#ifndef ZIGZAG_CONTROL_H
#define ZIGZAG_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ZIGZAG_MODE_OFF = 0U,
  ZIGZAG_MODE_PITCH_HOLD = 1U,
  ZIGZAG_MODE_DEPTH_CASCADE = 2U
} ZigzagControlMode;

typedef enum
{
  ZIGZAG_PHASE_DIVE = 0U,
  ZIGZAG_PHASE_CLIMB = 1U
} ZigzagPhase;

typedef enum
{
  ZIGZAG_FAULT_NONE = 0U,
  ZIGZAG_FAULT_DISARMED = 1U,
  ZIGZAG_FAULT_EKF_INVALID = 2U,
  ZIGZAG_FAULT_BAD_CONFIG = 3U,
  ZIGZAG_FAULT_BAD_COMMAND = 4U,
  ZIGZAG_FAULT_BAD_DT = 5U,
  ZIGZAG_FAULT_DEPTH_SAMPLE_INVALID = 6U,
  ZIGZAG_FAULT_DISPLACEMENT_INVALID = 7U
} ZigzagControlFault;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float derivative_tau_s;
} ZigzagPidConfig;

typedef struct
{
  ZigzagPidConfig depth_outer;
  ZigzagPidConfig pitch_inner;
  /* 深度误差直接生成俯仰目标；正系数配合 Down 正方向显式取负号。 */
  float depth_to_pitch_alpha_rad_per_m;
  // 用 pump_driver.h 的硬宏填写 min/max/deadzone；只允许上位机传参修改 neutral
  float displacement_min_mm;
  float displacement_neutral_mm;
  float displacement_max_mm;
  float displacement_deadzone_mm;
  float max_pitch_rad;
  float max_worm_rpm;
  float max_thrust_percent;
  float surface_depth_m;
  float depth_switch_threshold_m;
  /* 逻辑蜗杆命令与俯仰修正方向的极性，只允许 +1 或 -1。
   * 安装翻转已由 StartServoTask 统一处理，不要为同次安装在此重复反号。 */
  float pitch_actuator_sign;
} ZigzagControlConfig;

typedef struct
{
  ZigzagControlMode mode;
  uint8_t armed;
  uint16_t command_id;
  // DEPTH_CASCADE：本轮最大下潜深度；PITCH_HOLD 中忽略
  float target_depth_m;
  // PITCH_HOLD：直接俯仰目标；DEPTH_CASCADE 中由外环生成
  float target_pitch_rad;
  // 开环推进器百分比：正/负代表两种物理方向，0 表示中立
  float signed_thrust_percent;
} ZigzagControlCommand;

typedef struct
{
  uint8_t valid_for_control;
  /* 仅 DEPTH_CASCADE 使用；调用者根据测量时刻检查超时后给出 valid。 */
  uint8_t depth_sample_valid;
  uint8_t displacement_valid;
  uint32_t depth_update_seq;
  float depth_m;
  float pitch_rad;
  float vertical_velocity_mps;
  float displacement_mm;
} ZigzagControlInput;

typedef struct
{
  float error;
  float p_term;
  float i_term;
  float d_term;
  float unclamped_output;
  float output;
  uint8_t saturated;
} ZigzagPidTelemetry;

typedef struct
{
  ZigzagControlMode mode;
  ZigzagPhase phase;
  ZigzagControlFault fault;
  uint8_t armed;
  uint8_t output_valid;
  uint16_t command_id;
  float target_depth_m;
  float measured_depth_m;
  float target_displacement_mm;
  float measured_displacement_mm;
  float displacement_error_mm;
  float desired_pitch_rad;
  float measured_pitch_rad;
  ZigzagPidTelemetry depth_outer;
  ZigzagPidTelemetry pitch_inner;
  int32_t worm_motor_rpm;
  uint8_t pump_enable;
  uint8_t pump_direction; /* 1=下潜方向，0=上浮方向。 */
  float signed_thrust_percent;
} ZigzagControlOutput;

typedef struct
{
  float integral;
  float filtered_derivative;
  uint8_t first_run;
} ZigzagPidState;

typedef struct
{
  ZigzagControlConfig config;
  ZigzagControlCommand command;
  ZigzagPhase phase;
  ZigzagPidState depth_outer_state;//双环内外环各维护一个
  ZigzagPidState pitch_inner_state;
  ZigzagPidTelemetry depth_outer_telemetry;
  uint32_t last_depth_update_seq;//上一次真实压力序号
  float depth_outer_elapsed_s;//两个压力帧之间累计的外环时间
  float held_desired_pitch_rad;//压力帧之间保持的俯仰目标
  float held_target_displacement_mm;//压力帧之间保持的油囊目标位置
  float previous_pitch_rad;//上一帧俯仰角
  uint8_t have_depth_sample;
  uint8_t have_previous_pitch;
  uint8_t config_valid;
} ZigzagControl;//维护状态

/* 给出安全、保守但尚未实机整定的默认值；PID 模式仍须由外部显式 arm。 */
void ZigzagControl_DefaultConfig(ZigzagControlConfig *out_config);

/* 只校验候选配置，不改变当前控制器状态。 */
uint8_t ZigzagControl_ValidateConfig(
    const ZigzagControlConfig *config);

/* 初始化实例；配置非法时返回 0，并保持控制器不可用。 */
uint8_t ZigzagControl_Init(
    ZigzagControl *control, const ZigzagControlConfig *config);

/* 在控制任务安全周期边界整组应用参数，并复位两环状态。 */
uint8_t ZigzagControl_SetConfig(
    ZigzagControl *control, const ZigzagControlConfig *config);

/* 设置完整运行命令；重复刷新同一命令不会反复清积分。 */
uint8_t ZigzagControl_SetCommand(
    ZigzagControl *control, const ZigzagControlCommand *command);

/* 立即撤销输出并清除两环历史。 */
void ZigzagControl_Disarm(ZigzagControl *control);

/* 纯计算一步；调用周期由外部任务保证。 */
void ZigzagControl_Step(
    ZigzagControl *control,
    const ZigzagControlInput *input,
    float dt_s,
    ZigzagControlOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* ZIGZAG_CONTROL_H */
