/* 深度—俯仰级联控制器纯计算实现，接口与单位见 zigzag_control.h。 */
#include <zigzag_control.h>
#include <pump_driver.h>

#include <math.h>
#include <string.h>

#define ZIGZAG_PI_F 3.14159265358979323846f
#define ZIGZAG_DEFAULT_MAX_PITCH_RAD (ZIGZAG_PI_F / 6.0f)
#define ZIGZAG_COMMAND_EPSILON 1.0e-6f

static float ZigzagControl_Clamp(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

static int32_t ZigzagControl_RoundToI32(float value)
{
  return (value >= 0.0f)
             ? (int32_t)(value + 0.5f)
             : (int32_t)(value - 0.5f);
}

static uint8_t ZigzagControl_IsPidConfigValid(const ZigzagPidConfig *config)
{
  if (config == NULL ||
      !isfinite(config->kp) ||
      !isfinite(config->ki) ||
      !isfinite(config->kd) ||
      !isfinite(config->integral_limit) ||
      !isfinite(config->derivative_tau_s))
  {
    return 0U;
  }

  return (config->kp >= 0.0f &&
          config->ki >= 0.0f &&
          config->kd >= 0.0f &&
          config->integral_limit >= 0.0f &&
          config->derivative_tau_s >= 0.0f)
             ? 1U
             : 0U;
}

uint8_t ZigzagControl_ValidateConfig(
    const ZigzagControlConfig *config)
{
  if (config == NULL ||
      ZigzagControl_IsPidConfigValid(&config->depth_outer) == 0U ||
      ZigzagControl_IsPidConfigValid(&config->pitch_inner) == 0U ||
      !isfinite(config->max_pitch_rad) ||
      !isfinite(config->max_worm_rpm) ||
      !isfinite(config->max_thrust_percent) ||
      !isfinite(config->surface_depth_m) ||
      !isfinite(config->depth_switch_threshold_m) ||
      !isfinite(config->depth_to_pitch_alpha_rad_per_m) ||
      !isfinite(config->displacement_min_mm) ||
      !isfinite(config->displacement_neutral_mm) ||
      !isfinite(config->displacement_max_mm) ||
      !isfinite(config->displacement_deadzone_mm) ||
      !isfinite(config->pitch_actuator_sign))
  {
    return 0U;
  }

  if (config->max_pitch_rad <= 0.0f ||
      config->max_pitch_rad > (ZIGZAG_PI_F / 2.0f) ||
      config->max_worm_rpm <= 0.0f ||
      config->max_worm_rpm > 600.0f ||
      config->max_thrust_percent < 0.0f ||
      config->max_thrust_percent > 100.0f ||
      config->surface_depth_m < -0.2f || //板端二次校验设定水面深度，由于传感器误差暂设负数
      config->depth_switch_threshold_m < 0.0f ||
      config->depth_to_pitch_alpha_rad_per_m < 0.0f ||
      config->depth_to_pitch_alpha_rad_per_m > (ZIGZAG_PI_F / 2.0f) ||
      config->displacement_min_mm < 0.0f ||
      config->displacement_neutral_mm <= config->displacement_min_mm ||
      config->displacement_neutral_mm >= config->displacement_max_mm ||
      config->displacement_deadzone_mm <= 0.0f)
  {
    return 0U;
  }

  return (fabsf(fabsf(config->pitch_actuator_sign) - 1.0f) <=
          ZIGZAG_COMMAND_EPSILON)
             ? 1U
             : 0U;
}

static void ZigzagPid_Reset(ZigzagPidState *state)
{
  if (state == NULL)
  {
    return;
  }

  state->integral = 0.0f;
  state->filtered_derivative = 0.0f;
  state->first_run = 1U;
}

static void ZigzagControl_ResetLoops(ZigzagControl *control)
{
  ZigzagPid_Reset(&control->depth_outer_state);
  ZigzagPid_Reset(&control->pitch_inner_state);
  (void)memset(
      &control->depth_outer_telemetry,
      0,
      sizeof(control->depth_outer_telemetry));
  control->last_depth_update_seq = 0U;
  control->depth_outer_elapsed_s = 0.0f;
  control->held_desired_pitch_rad = 0.0f;
  control->held_target_displacement_mm =
      control->config.displacement_neutral_mm;
  control->previous_pitch_rad = 0.0f;
  control->have_depth_sample = 0U;
  control->have_previous_pitch = 0U;
}

static float ZigzagPid_Update(
    const ZigzagPidConfig *config,
    ZigzagPidState *state,
    float error,
    float raw_error_derivative,
    float dt_s,
    float output_min,
    float output_max,
    ZigzagPidTelemetry *telemetry)
{
  float previous_integral;
  float candidate_integral;
  float alpha;
  float output;
  uint8_t reject_integral = 0U;

  (void)memset(telemetry, 0, sizeof(*telemetry));
  telemetry->error = error;

  previous_integral = state->integral;
  candidate_integral = previous_integral + error * dt_s;
  candidate_integral = ZigzagControl_Clamp(
      candidate_integral,
      -config->integral_limit,
      config->integral_limit);

  if (state->first_run != 0U)
  {
    state->filtered_derivative = 0.0f;
    state->first_run = 0U;
  }
  else if (config->derivative_tau_s > 0.0f)
  {
    alpha = dt_s / (config->derivative_tau_s + dt_s);
    state->filtered_derivative +=
        alpha * (raw_error_derivative - state->filtered_derivative);
  }
  else
  {
    state->filtered_derivative = raw_error_derivative;
  }

  telemetry->p_term = config->kp * error;
  telemetry->i_term = config->ki * candidate_integral;
  telemetry->d_term = config->kd * state->filtered_derivative;
  telemetry->unclamped_output =
      telemetry->p_term + telemetry->i_term + telemetry->d_term;
  output = ZigzagControl_Clamp(
      telemetry->unclamped_output, output_min, output_max);

  /* 条件积分抗饱和：只撤销继续把输出推向饱和方向的本次积分。 */
  if (output >= output_max &&
      telemetry->unclamped_output > output_max &&
      (config->ki * error) > 0.0f)
  {
    reject_integral = 1U;
  }
  else if (output <= output_min &&
           telemetry->unclamped_output < output_min &&
           (config->ki * error) < 0.0f)
  {
    reject_integral = 1U;
  }

  if (reject_integral != 0U)
  {
    candidate_integral = previous_integral;
    telemetry->i_term = config->ki * candidate_integral;
    telemetry->unclamped_output =
        telemetry->p_term + telemetry->i_term + telemetry->d_term;
    output = ZigzagControl_Clamp(
        telemetry->unclamped_output, output_min, output_max);
  }

  state->integral = candidate_integral;
  telemetry->output = output;
  telemetry->saturated =
      (fabsf(output - telemetry->unclamped_output) >
       ZIGZAG_COMMAND_EPSILON)
          ? 1U
          : 0U;
  return output;
}

static uint8_t ZigzagControl_IsCommandValid(
    const ZigzagControl *control,
    const ZigzagControlCommand *command)
{
  if (control == NULL ||
      command == NULL ||
      !isfinite(command->target_depth_m) ||
      !isfinite(command->target_pitch_rad) ||
      !isfinite(command->signed_thrust_percent) ||
      command->mode > ZIGZAG_MODE_DEPTH_CASCADE)
  {
    return 0U;
  }

  if (fabsf(command->signed_thrust_percent) >
      control->config.max_thrust_percent)
  {
    return 0U;
  }

  if (command->mode == ZIGZAG_MODE_PITCH_HOLD &&
      fabsf(command->target_pitch_rad) > control->config.max_pitch_rad)
  {
    return 0U;
  }

  if (command->mode == ZIGZAG_MODE_DEPTH_CASCADE &&
      command->target_depth_m <=
          (control->config.surface_depth_m +
           control->config.depth_switch_threshold_m))
  {
    return 0U;
  }

  return 1U;
}

void ZigzagControl_DefaultConfig(ZigzagControlConfig *out_config)
{
  if (out_config == NULL)
  {
    return;
  }

  (void)memset(out_config, 0, sizeof(*out_config));

  /*
   * 浮力外环初值只用于形成合法默认配置，不视为实机已整定参数。
   * 内环 Kp=57.2958 RPM/rad 等价于旧代码的 1 RPM/deg。
   */
  out_config->depth_outer.kp = 5.0f;
  out_config->depth_outer.ki = 0.0f;
  out_config->depth_outer.kd = 1.0f;
  out_config->depth_outer.integral_limit = 2.0f;
  out_config->depth_outer.derivative_tau_s = 0.20f;

  out_config->pitch_inner.kp = 57.2957795f;
  out_config->pitch_inner.ki = 0.0f;
  out_config->pitch_inner.kd = 0.0f;
  out_config->pitch_inner.integral_limit = 1.0f;
  out_config->pitch_inner.derivative_tau_s = 0.05f;

  out_config->depth_to_pitch_alpha_rad_per_m = 0.03f;
  out_config->displacement_min_mm = IO_PUMP_DISPLACEMENT_MIN_MM;
  out_config->displacement_max_mm = IO_PUMP_DISPLACEMENT_MAX_MM;
  out_config->displacement_neutral_mm =
      0.5f * (out_config->displacement_min_mm +
              out_config->displacement_max_mm);
  out_config->displacement_deadzone_mm =
      IO_PUMP_DISPLACEMENT_DEADZONE_MM;
  out_config->max_pitch_rad = ZIGZAG_DEFAULT_MAX_PITCH_RAD;
  out_config->max_worm_rpm = 50.0f;
  out_config->max_thrust_percent = 30.0f;
  out_config->surface_depth_m = 0.2f;
  out_config->depth_switch_threshold_m = 0.1f;
  out_config->pitch_actuator_sign = 1.0f;
}

uint8_t ZigzagControl_Init(
    ZigzagControl *control, const ZigzagControlConfig *config)
{
  if (control == NULL)
  {
    return 0U;
  }

  (void)memset(control, 0, sizeof(*control));
  control->phase = ZIGZAG_PHASE_DIVE;
  ZigzagControl_ResetLoops(control);

  if (ZigzagControl_ValidateConfig(config) == 0U)
  {
    control->config_valid = 0U;
    return 0U;
  }

  control->config = *config;
  control->config_valid = 1U;
  control->command.mode = ZIGZAG_MODE_OFF;
  return 1U;
}

uint8_t ZigzagControl_SetConfig(
    ZigzagControl *control, const ZigzagControlConfig *config)
{
  if (control == NULL || ZigzagControl_ValidateConfig(config) == 0U)
  {
    if (control != NULL)
    {
      control->config_valid = 0U;
      ZigzagControl_Disarm(control);
    }
    return 0U;
  }

  control->config = *config;
  control->config_valid = 1U;
  control->phase = ZIGZAG_PHASE_DIVE;
  ZigzagControl_ResetLoops(control);

  if (ZigzagControl_IsCommandValid(control, &control->command) == 0U)
  {
    ZigzagControl_Disarm(control);
  }
  return 1U;
}

uint8_t ZigzagControl_SetCommand(
    ZigzagControl *control, const ZigzagControlCommand *command)
{
  uint8_t reset_required;

  if (control == NULL ||
      control->config_valid == 0U ||
      ZigzagControl_IsCommandValid(control, command) == 0U)
  {
    if (control != NULL)
    {
      ZigzagControl_Disarm(control);
    }
    return 0U;
  }

  reset_required =
      (control->command.mode != command->mode ||
       control->command.armed != command->armed ||
       fabsf(control->command.target_depth_m - command->target_depth_m) >
           ZIGZAG_COMMAND_EPSILON ||
       fabsf(control->command.target_pitch_rad - command->target_pitch_rad) >
           ZIGZAG_COMMAND_EPSILON)
          ? 1U
          : 0U;

  control->command = *command;
  control->command.armed = (command->armed != 0U) ? 1U : 0U;

  if (control->command.mode == ZIGZAG_MODE_OFF ||
      control->command.armed == 0U ||
      reset_required != 0U)
  {
    control->phase = ZIGZAG_PHASE_DIVE;
    ZigzagControl_ResetLoops(control);
  }

  return 1U;
}

void ZigzagControl_Disarm(ZigzagControl *control)
{
  if (control == NULL)
  {
    return;
  }

  control->command.armed = 0U;
  control->command.mode = ZIGZAG_MODE_OFF;
  control->command.signed_thrust_percent = 0.0f;
  control->phase = ZIGZAG_PHASE_DIVE;
  ZigzagControl_ResetLoops(control);
}

void ZigzagControl_Step(
    ZigzagControl *control,
    const ZigzagControlInput *input,
    float dt_s,
    ZigzagControlOutput *output)
{
  float depth_pid_output_mm = 0.0f;
  float depth_pid_output_min_mm;
  float depth_pid_output_max_mm;
  float depth_error_m;
  float desired_pitch_rad = 0.0f;
  float displacement_error_mm;
  float pitch_error_derivative = 0.0f;
  float pitch_pid_output;
  uint8_t new_depth_sample;
  uint8_t displacement_is_valid;

  if (output == NULL)
  {
    return;
  }

  (void)memset(output, 0, sizeof(*output));
  output->fault = ZIGZAG_FAULT_BAD_COMMAND;

  if (control == NULL || input == NULL)
  {
    return;
  }

  displacement_is_valid =
      (input->displacement_valid != 0U &&
       isfinite(input->displacement_mm))
          ? 1U
          : 0U;

  output->mode = control->command.mode;
  output->phase = control->phase;
  output->armed = control->command.armed;
  output->command_id = control->command.command_id;
  output->measured_depth_m = input->depth_m;
  output->measured_pitch_rad = input->pitch_rad;
  output->measured_displacement_mm =
      isfinite(input->displacement_mm) ? input->displacement_mm : 0.0f;

  if (control->config_valid == 0U)
  {
    output->fault = ZIGZAG_FAULT_BAD_CONFIG;
    return;
  }

  if (control->command.mode == ZIGZAG_MODE_OFF ||
      control->command.armed == 0U)
  {
    output->fault = ZIGZAG_FAULT_DISARMED;
    return;
  }

  if (input->valid_for_control == 0U || !isfinite(input->pitch_rad))
  {
    ZigzagControl_ResetLoops(control);
    output->fault = ZIGZAG_FAULT_EKF_INVALID;
    return;
  }

  if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.2f)
  {
    ZigzagControl_ResetLoops(control);
    output->fault = ZIGZAG_FAULT_BAD_DT;
    return;
  }

  if (control->command.mode == ZIGZAG_MODE_DEPTH_CASCADE)
  {
    if (input->depth_sample_valid == 0U ||
        input->depth_update_seq == 0U ||
        !isfinite(input->depth_m) ||
        !isfinite(input->vertical_velocity_mps))
    {
      ZigzagControl_ResetLoops(control);
      output->fault = ZIGZAG_FAULT_DEPTH_SAMPLE_INVALID;
      return;
    }

    if (control->phase == ZIGZAG_PHASE_DIVE &&
        input->depth_m >=
            (control->command.target_depth_m -
             control->config.depth_switch_threshold_m))
    {
      control->phase = ZIGZAG_PHASE_CLIMB;
      ZigzagControl_ResetLoops(control);
    }
    else if (control->phase == ZIGZAG_PHASE_CLIMB &&
             input->depth_m <=
                 (control->config.surface_depth_m +
                  control->config.depth_switch_threshold_m))
    {
      control->phase = ZIGZAG_PHASE_DIVE;
      ZigzagControl_ResetLoops(control);
    }

    output->phase = control->phase;
    output->target_depth_m =
        (control->phase == ZIGZAG_PHASE_DIVE)
            ? control->command.target_depth_m
            : control->config.surface_depth_m;

    control->depth_outer_elapsed_s += dt_s;
    new_depth_sample =
        (control->have_depth_sample == 0U ||
         input->depth_update_seq != control->last_depth_update_seq)
            ? 1U
            : 0U;

    if (new_depth_sample != 0U)
    {
      depth_error_m = output->target_depth_m - input->depth_m;

      /*
       * depth 向下为正。浮力 PID 的正输出表示“需要减小浮力”，
       * 所以目标位移为 neutral-u_b。输出上下限由中性位置到两个
       * 物理端点的实际距离计算，不再写死为某个固定 mm 幅度。
       *
       * D 项直接使用误差导数 -vD，避免目标深度换相阶跃造成微分
       * 冲击。外环 dt 是两帧真实压力之间累计的时间
       *
       * 位移反馈无效时冻结浮力 PID 和目标位移，避免执行器不可用
       * 期间继续积分；俯仰目标仍根据本次深度误差正常刷新。
       */
      if (displacement_is_valid != 0U)
      {
        depth_pid_output_min_mm =
            control->config.displacement_neutral_mm -
            control->config.displacement_max_mm;
        depth_pid_output_max_mm =
            control->config.displacement_neutral_mm -
            control->config.displacement_min_mm;
        depth_pid_output_mm = ZigzagPid_Update(
            &control->config.depth_outer,
            &control->depth_outer_state,
            depth_error_m,
            -input->vertical_velocity_mps,
            control->depth_outer_elapsed_s,
            depth_pid_output_min_mm,
            depth_pid_output_max_mm,
            &control->depth_outer_telemetry);
        control->held_target_displacement_mm = ZigzagControl_Clamp(
            control->config.displacement_neutral_mm - depth_pid_output_mm,
            control->config.displacement_min_mm,
            control->config.displacement_max_mm);
      }

      /*
       * 姿态支路不再复用浮力 PID 输出：Alpha 只把深度误差换算成
       * 俯仰目标。误差为正表示目标更深，因此用负号产生低头目标
       */
      control->held_desired_pitch_rad =
          -control->config.depth_to_pitch_alpha_rad_per_m * depth_error_m;
      control->last_depth_update_seq = input->depth_update_seq;
      control->depth_outer_elapsed_s = 0.0f;
      control->have_depth_sample = 1U;
    }

    output->depth_outer = control->depth_outer_telemetry;
    output->target_displacement_mm =
        control->held_target_displacement_mm;
    if (displacement_is_valid != 0U)
    {
      output->displacement_error_mm =
          output->target_displacement_mm -
          output->measured_displacement_mm;
    }
    else
    {
      output->displacement_error_mm = 0.0f;
    }
    desired_pitch_rad = control->held_desired_pitch_rad;

    if (control->phase == ZIGZAG_PHASE_DIVE)
    {
      desired_pitch_rad = ZigzagControl_Clamp(
          desired_pitch_rad, -control->config.max_pitch_rad, 0.0f);
    }
    else
    {
      desired_pitch_rad = ZigzagControl_Clamp(
          desired_pitch_rad, 0.0f, control->config.max_pitch_rad);
    }

    /*
     * 位置闭环使用底层给出的单一死区。目标更小时向最小位移方向
     * 抽油（direction=1），目标更大时充油（direction=0）。
     */
    displacement_error_mm = output->displacement_error_mm;
    if (displacement_is_valid == 0U)
    {
      output->pump_enable = 0U;
      output->pump_direction = 0U;
    }
    else if (displacement_error_mm <=
             -control->config.displacement_deadzone_mm)
    {
      output->pump_enable = 1U;
      output->pump_direction = 1U;
    }
    else if (displacement_error_mm >=
             control->config.displacement_deadzone_mm)
    {
      output->pump_enable = 1U;
      output->pump_direction = 0U;
    }
    else
    {
      output->pump_enable = 0U;
      output->pump_direction = 0U;
    }
  }
  else if (control->command.mode == ZIGZAG_MODE_PITCH_HOLD)
  {
    output->target_depth_m = input->depth_m;
    desired_pitch_rad = ZigzagControl_Clamp(
        control->command.target_pitch_rad,
        -control->config.max_pitch_rad,
        control->config.max_pitch_rad);
    output->target_displacement_mm =
        output->measured_displacement_mm;
    output->displacement_error_mm = 0.0f;
    output->pump_enable = 0U;
    output->pump_direction = 0U;
  }
  else
  {
    output->fault = ZIGZAG_FAULT_BAD_COMMAND;
    return;
  }

  if (control->have_previous_pitch != 0U)
  {
    /* 对测量值微分，避免外环改变 pitch setpoint 时产生 D 项冲击。 */
    pitch_error_derivative =
        -(input->pitch_rad - control->previous_pitch_rad) / dt_s;
  }
  control->previous_pitch_rad = input->pitch_rad;
  control->have_previous_pitch = 1U;

  output->desired_pitch_rad = desired_pitch_rad;
  pitch_pid_output = ZigzagPid_Update(
      &control->config.pitch_inner,
      &control->pitch_inner_state,
      desired_pitch_rad - input->pitch_rad,
      pitch_error_derivative,
      dt_s,
      -control->config.max_worm_rpm,
      control->config.max_worm_rpm,
      &output->pitch_inner);

  pitch_pid_output *= control->config.pitch_actuator_sign;
  output->worm_motor_rpm = ZigzagControl_RoundToI32(
      ZigzagControl_Clamp(
          pitch_pid_output,
          -control->config.max_worm_rpm,
          control->config.max_worm_rpm));
  output->signed_thrust_percent = ZigzagControl_Clamp(
      control->command.signed_thrust_percent,
      -control->config.max_thrust_percent,
      control->config.max_thrust_percent);
  output->output_valid = 1U;
  output->fault =
      (displacement_is_valid != 0U)
          ? ZIGZAG_FAULT_NONE
          : ZIGZAG_FAULT_DISPLACEMENT_INVALID;
}
