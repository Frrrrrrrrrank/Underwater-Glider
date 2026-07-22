/* zigzag_control.c 定义见 zigzag_control.h */
#include "zigzag_control.h"

#include "ekf_bridge.h"
#include "pump_driver.h"
#include "motor_driver.h"
#include "stepper_driver.h"

#include <math.h>

#define ZIGZAG_DEPTH_SURFACE_M       0.2f
#define ZIGZAG_RAD2DEG               57.29577951308232f

#define RC_WORM_MOTOR_NODE_ID        0x01U
#define RC_WORM_MOTOR_SLOT           0U
#define RC_WORM_MOTOR_MAX_RPM_PID    50
#define RC_STEPPER_MAX_RPM_PID       50

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
} ZigzagPid;

static ZigzagPhase s_up_down_flag = ZIGZAG_PHASE_DIVE;

static float s_target_depth_m = 0.0f;
static float s_target_pitch_dive_deg = 0.0f;
static float s_target_pitch_climb_deg = 0.0f;
static float s_target_yaw_deg = 0.0f;

static ZigzagPid s_pitch_pid = {0};
static ZigzagPid s_roll_pid = {0};
static ZigzagPid s_yaw_pid = {0};
static float s_yaw_pid_output = 0.0f;
static ZigzagPid s_depth_pid = {0};

static int32_t ZigzagControl_ClampSigned(int32_t value, int32_t limit)
{
  if (limit < 0)
  {
    limit = -limit;
  }

  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

static float ZigzagPid_Update(ZigzagPid *pid, float setpoint, float measurement, float dt_s)
{
  float error;
  float derivative;
  float output;

  if (dt_s <= 0.0f)
  {
    return 0.0f;
  }

  error = setpoint - measurement;
  pid->integral += error * dt_s;
  derivative = (error - pid->prev_error) / dt_s;
  pid->prev_error = error;

  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

  return output;
}

void ZigzagControl_Init(void)
{
  s_up_down_flag = ZIGZAG_PHASE_DIVE;

  s_target_depth_m = 5.0f;
  s_target_pitch_dive_deg = (3.14f / 6.0f) * ZIGZAG_RAD2DEG;
  s_target_pitch_climb_deg = (3.14f / 6.0f) * ZIGZAG_RAD2DEG;
  s_target_yaw_deg = 0.0f;

  s_pitch_pid.kp = 1.0f;
  s_pitch_pid.ki = 0.0f;
  s_pitch_pid.kd = 0.0f;
  s_pitch_pid.integral = 0.0f;
  s_pitch_pid.prev_error = 0.0f;

  s_roll_pid.kp = 1.0f;
  s_roll_pid.ki = 0.0f;
  s_roll_pid.kd = 0.0f;
  s_roll_pid.integral = 0.0f;
  s_roll_pid.prev_error = 0.0f;

  s_yaw_pid.kp = 1.0f;
  s_yaw_pid.ki = 0.0f;
  s_yaw_pid.kd = 0.0f;
  s_yaw_pid.integral = 0.0f;
  s_yaw_pid.prev_error = 0.0f;
  s_yaw_pid_output = 0.0f;

  s_depth_pid.kp = 1.0f;
  s_depth_pid.ki = 0.0f;
  s_depth_pid.kd = 0.0f;
  s_depth_pid.integral = 0.0f;
  s_depth_pid.prev_error = 0.0f;

  UMotor_SetMotorEnable(RC_WORM_MOTOR_SLOT, 1U);
}

void ZigzagControl_SetTargets(float target_depth_m,
                              float target_pitch_dive_deg,
                              float target_pitch_climb_deg,
                              float target_yaw_deg)
{
  s_target_depth_m = target_depth_m;
  s_target_pitch_dive_deg = target_pitch_dive_deg;
  s_target_pitch_climb_deg = target_pitch_climb_deg;
  s_target_yaw_deg = target_yaw_deg;
}

void ZigzagControl_SetPitchPidGains(float kp, float ki, float kd)
{
  s_pitch_pid.kp = kp;
  s_pitch_pid.ki = ki;
  s_pitch_pid.kd = kd;
}

void ZigzagControl_SetRollPidGains(float kp, float ki, float kd)
{
  s_roll_pid.kp = kp;
  s_roll_pid.ki = ki;
  s_roll_pid.kd = kd;
}

void ZigzagControl_SetYawPidGains(float kp, float ki, float kd)
{
  s_yaw_pid.kp = kp;
  s_yaw_pid.ki = ki;
  s_yaw_pid.kd = kd;
}

float ZigzagControl_GetYawPidOutput(void)
{
  return s_yaw_pid_output;
}

void ZigzagControl_SetDepthPidGains(float kp, float ki, float kd)
{
  s_depth_pid.kp = kp;
  s_depth_pid.ki = ki;
  s_depth_pid.kd = kd;
}

void ZigzagControl_SetPhase(ZigzagPhase phase)
{
  s_up_down_flag = phase;
}

ZigzagPhase ZigzagControl_GetPhase(void)
{
  return s_up_down_flag;
}

void ZigzagControl_Step(float dt_s)
{
  StateEstimate snapshot;
  float pitch_deg;
  float roll_deg;
  float yaw_deg;
  float depth_m;
  float pitch_setpoint_deg;
  float pitch_pid_output;
  float roll_pid_output;
  int32_t worm_cmd_rpm_PID;
  int32_t stepper_cmd_rpm_PID;

  (void)EkfBridge_CopyState(&snapshot);

  pitch_deg = snapshot.out.pitch * ZIGZAG_RAD2DEG;
  roll_deg = snapshot.out.roll * ZIGZAG_RAD2DEG;
  yaw_deg = snapshot.out.yaw * ZIGZAG_RAD2DEG;
  depth_m = snapshot.out.depth_m;

  /* 航向 PID：只计算，输出暂不接执行器 */
  s_yaw_pid_output = ZigzagPid_Update(&s_yaw_pid, s_target_yaw_deg, yaw_deg, dt_s);

  /* roll 期望值恒为 0：与 up_down_flag 无关，始终运行 */
  roll_pid_output = ZigzagPid_Update(&s_roll_pid, 0.0f, roll_deg, dt_s);
  stepper_cmd_rpm_PID = ZigzagControl_ClampSigned((int32_t)roll_pid_output, RC_STEPPER_MAX_RPM_PID);
  Stepper_SetContinuousSignedRpm(stepper_cmd_rpm_PID);

  if (s_up_down_flag == ZIGZAG_PHASE_DIVE)
  {
    if (depth_m >= s_target_depth_m)
    {
      s_up_down_flag = ZIGZAG_PHASE_CLIMB;
    }
  }
  else if (s_up_down_flag == ZIGZAG_PHASE_CLIMB)
  {
    if (depth_m < ZIGZAG_DEPTH_SURFACE_M)
    {
      s_up_down_flag = ZIGZAG_PHASE_DIVE;
    }
  }

  if (s_up_down_flag == ZIGZAG_PHASE_DIVE)
  {
    IoControlPump_SetSpeed(1U);
    IoControlPump_SetDirection(1U);
    pitch_setpoint_deg = s_target_pitch_dive_deg;
  }
  else if (s_up_down_flag == ZIGZAG_PHASE_CLIMB)
  {
    IoControlPump_SetSpeed(1U);
    IoControlPump_SetDirection(0U);
    pitch_setpoint_deg = s_target_pitch_climb_deg;
  }
  else
  {
    /* ZIGZAG_PHASE_MANUAL：泵与俯仰电机交由外部逻辑控制，本模块不下发指令 */
    return;
  }

  pitch_pid_output = ZigzagPid_Update(&s_pitch_pid, pitch_setpoint_deg, pitch_deg, dt_s);
  worm_cmd_rpm_PID = ZigzagControl_ClampSigned((int32_t)pitch_pid_output, RC_WORM_MOTOR_MAX_RPM_PID);

  UMotor_SetTargetSpeedById(RC_WORM_MOTOR_NODE_ID, worm_cmd_rpm_PID);
}
