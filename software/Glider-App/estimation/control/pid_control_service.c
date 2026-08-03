// PID 会话、参数事务与纯计算级联控制器的任务上下文桥接

#include "FreeRTOS.h"
#include "task.h"
#include "pid_control_service.h"

#include <math.h>
#include <string.h>

#define PID_CONTROL_PI_F 3.14159265358979323846f
#define PID_CONTROL_DEG_TO_RAD (PID_CONTROL_PI_F / 180.0f)
#define PID_CONTROL_RAD_TO_DEG (180.0f / PID_CONTROL_PI_F)

typedef struct
{
  PidControlCommand command;
  uint32_t rx_tick_ms;
  uint32_t seq;
} PidControlPendingCommand;

typedef struct
{
  uint8_t table_bytes[PID_CONTROL_PARAM_TABLE_BYTES];
  uint32_t expected_crc;
  uint32_t seq;
} PidControlPendingConfig;

static ZigzagControl s_controller;
static PidControlPendingCommand s_command_pending;
static PidControlPendingConfig s_config_pending;
static PidControlCommand s_active_command;
static uint32_t s_active_command_rx_tick_ms = 0U;
static uint32_t s_command_pending_seq = 0U;
static uint32_t s_command_handled_seq = 0U;
static uint32_t s_config_pending_seq = 0U;
static uint32_t s_config_handled_seq = 0U;
static uint32_t s_active_pid_crc = 0U;
static uint32_t s_status_seq = 0U;
static uint32_t s_config_request_count = 0U;
static uint32_t s_config_apply_count = 0U;
static uint32_t s_config_reject_count = 0U;
static PidControlConfigRejectReason s_config_reject_reason =
    PID_CONTROL_CONFIG_REJECT_NONE;
static uint8_t s_pid_valid = 0U;
static uint8_t s_ready = 0U;

PidControlStatus g_pid_control_status;

static uint32_t PidControlService_ReadLeU32(const uint8_t *bytes)
{
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8U) |
         ((uint32_t)bytes[2] << 16U) |
         ((uint32_t)bytes[3] << 24U);
}

static float PidControlService_ReadLeFloat(const uint8_t *bytes)
{
  const uint32_t bits = PidControlService_ReadLeU32(bytes);
  float value;

  (void)memcpy(&value, &bits, sizeof(value));
  return value;
}

/* CRC-32/ISO-HDLC，与 UGCP 外层和 Python zlib.crc32 的参数表 CRC 一致。 */
static uint32_t PidControlService_Crc32(
    const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;

  for (i = 0U; i < length; i++)
  {
    uint32_t bit;
    crc ^= (uint32_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }

  return crc ^ 0xFFFFFFFFUL;
}

static uint8_t PidControlService_IsCommandBasicValid(
    const PidControlCommand *command)
{
  if (command == NULL ||
      command->mode > ZIGZAG_MODE_DEPTH_CASCADE ||
      command->armed > 1U ||
      !isfinite(command->target_depth_m) ||
      !isfinite(command->target_pitch_deg) ||
      !isfinite(command->signed_thrust_percent))
  {
    return 0U;
  }

  if (command->target_depth_m < 0.0f ||
      command->target_depth_m > 1000.0f ||
      fabsf(command->target_pitch_deg) > 90.0f ||
      fabsf(command->signed_thrust_percent) > 100.0f)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t PidControlService_TakePendingCommand(
    PidControlPendingCommand *pending)
{
  uint8_t got = 0U;

  taskENTER_CRITICAL();
  if (s_command_pending_seq != s_command_handled_seq)
  {
    *pending = s_command_pending;
    s_command_handled_seq = pending->seq;
    got = 1U;
  }
  taskEXIT_CRITICAL();
  return got;
}

static uint8_t PidControlService_TakePendingConfig(
    PidControlPendingConfig *pending)
{
  uint8_t got = 0U;

  taskENTER_CRITICAL();
  if (s_config_pending_seq != s_config_handled_seq)
  {
    *pending = s_config_pending;
    s_config_handled_seq = pending->seq;
    got = 1U;
  }
  taskEXIT_CRITICAL();
  return got;
}
//单位转换函数
static uint8_t PidControlService_DecodeConfig(
    const uint8_t table[PID_CONTROL_PARAM_TABLE_BYTES],
    ZigzagControlConfig *config)
{
  float wire[PID_CONTROL_PARAM_FLOAT_COUNT];
  uint32_t i;

  if (table == NULL || config == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < PID_CONTROL_PARAM_FLOAT_COUNT; i++)
  {
    wire[i] = PidControlService_ReadLeFloat(&table[i * sizeof(float)]);
    if (!isfinite(wire[i]))
    {
      return 0U;
    }
  }

  /*
   * wire 侧外环输出为 deg、内环误差为 deg；核心统一使用 rad。
   * 0..4 depth PID，5..9 pitch PID，10..15 六个公共限制量。
   */
  config->depth_outer.kp = wire[0] * PID_CONTROL_DEG_TO_RAD;
  config->depth_outer.ki = wire[1] * PID_CONTROL_DEG_TO_RAD;
  config->depth_outer.kd = wire[2] * PID_CONTROL_DEG_TO_RAD;
  config->depth_outer.integral_limit = wire[3];
  config->depth_outer.derivative_tau_s = wire[4];

  config->pitch_inner.kp = wire[5] * PID_CONTROL_RAD_TO_DEG;
  config->pitch_inner.ki = wire[6] * PID_CONTROL_RAD_TO_DEG;
  config->pitch_inner.kd = wire[7] * PID_CONTROL_RAD_TO_DEG;
  config->pitch_inner.integral_limit = wire[8] * PID_CONTROL_DEG_TO_RAD;
  config->pitch_inner.derivative_tau_s = wire[9];

  config->max_pitch_rad = wire[10] * PID_CONTROL_DEG_TO_RAD;
  config->max_worm_rpm = wire[11];
  config->max_thrust_percent = wire[12];
  config->surface_depth_m = wire[13];
  config->depth_switch_threshold_m = wire[14];
  config->pitch_actuator_sign = wire[15];

  return ZigzagControl_ValidateConfig(config);
}

static void PidControlService_ProcessPendingConfig(void)
{
  PidControlPendingConfig pending;
  ZigzagControlConfig candidate;
  uint32_t actual_crc;

  if (PidControlService_TakePendingConfig(&pending) == 0U)
  {
    return;
  }

  s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_NONE;

  if (s_active_command.armed != 0U &&
      s_active_command.mode != ZIGZAG_MODE_OFF)
  {
    s_config_reject_count++;
    s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_ARMED;
    return;
  }

  actual_crc = PidControlService_Crc32(
      pending.table_bytes, PID_CONTROL_PARAM_TABLE_BYTES);
  if (actual_crc != pending.expected_crc)
  {
    s_config_reject_count++;
    s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_CRC;
    return;
  }

  if (PidControlService_DecodeConfig(
          pending.table_bytes, &candidate) == 0U)
  {
    s_config_reject_count++;
    s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_VALUE;
    return;
  }

  if (ZigzagControl_SetConfig(&s_controller, &candidate) == 0U)
  {
    s_config_reject_count++;
    s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_VALUE;
    return;
  }

  ZigzagControl_Disarm(&s_controller);
  s_active_pid_crc = actual_crc;
  s_pid_valid = 1U;
  s_config_apply_count++;
}

static void PidControlService_PublishStatus(
    const StateEstimate *state,
    uint32_t now_ms,
    uint32_t command_age_ms,
    PidControlFault fault,
    const ZigzagControlOutput *control_output)
{
  PidControlStatus status;

  (void)memset(&status, 0, sizeof(status));
  status.update_seq = ++s_status_seq;
  status.update_tick_ms = now_ms;
  status.command_age_ms = command_age_ms;
  status.required_pid_crc = s_active_command.required_pid_crc;
  status.active_pid_crc = s_active_pid_crc;
  status.config_request_count = s_config_request_count;
  status.config_apply_count = s_config_apply_count;
  status.config_reject_count = s_config_reject_count;
  status.command_id = s_active_command.command_id;
  status.requested_mode = s_active_command.mode;
  status.arm_requested = s_active_command.armed;
  status.pid_valid = s_pid_valid;
  status.fault = fault;
  status.config_reject_reason = s_config_reject_reason;
  status.requested_depth_m = s_active_command.target_depth_m;
  status.requested_pitch_deg = s_active_command.target_pitch_deg;
  status.requested_thrust_percent =
      s_active_command.signed_thrust_percent;

  if (state != NULL)
  {
    status.depth_measurement_seq = state->depth_measurement_seq;
  }
  if (control_output != NULL)
  {
    status.control_output = *control_output;
    status.effectively_armed = control_output->output_valid;
  }

  taskENTER_CRITICAL();
  g_pid_control_status = status;
  taskEXIT_CRITICAL();
}

void PidControlService_Init(void)
{
  ZigzagControlConfig defaults;

  (void)memset(&s_command_pending, 0, sizeof(s_command_pending));
  (void)memset(&s_config_pending, 0, sizeof(s_config_pending));
  (void)memset(&s_active_command, 0, sizeof(s_active_command));
  (void)memset(&g_pid_control_status, 0, sizeof(g_pid_control_status));
  s_command_pending_seq = 0U;
  s_command_handled_seq = 0U;
  s_config_pending_seq = 0U;
  s_config_handled_seq = 0U;
  s_active_command_rx_tick_ms = 0U;
  s_active_pid_crc = 0U;
  s_status_seq = 0U;
  s_config_request_count = 0U;
  s_config_apply_count = 0U;
  s_config_reject_count = 0U;
  s_config_reject_reason = PID_CONTROL_CONFIG_REJECT_NONE;
  s_pid_valid = 0U;

  ZigzagControl_DefaultConfig(&defaults);
  (void)ZigzagControl_Init(&s_controller, &defaults);
  ZigzagControl_Disarm(&s_controller);
  s_ready = 1U;
}

uint8_t PidControlService_SubmitCommand(
    const PidControlCommand *command, uint32_t rx_tick_ms)
{
  if (s_ready == 0U ||
      PidControlService_IsCommandBasicValid(command) == 0U)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  s_command_pending.command = *command;
  s_command_pending.rx_tick_ms = rx_tick_ms;
  s_command_pending.seq = ++s_command_pending_seq;
  taskEXIT_CRITICAL();
  return 1U;
}

uint8_t PidControlService_SubmitConfigTable(
    const uint8_t table_bytes[PID_CONTROL_PARAM_TABLE_BYTES],
    uint32_t expected_crc)
{
  if (s_ready == 0U || table_bytes == NULL)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  (void)memcpy(
      s_config_pending.table_bytes,
      table_bytes,
      PID_CONTROL_PARAM_TABLE_BYTES);
  s_config_pending.expected_crc = expected_crc;
  s_config_pending.seq = ++s_config_pending_seq;
  s_config_request_count++;
  taskEXIT_CRITICAL();
  return 1U;
}

void PidControlService_Step(
    const StateEstimate *state,
    uint32_t now_ms,
    float dt_s,
    ZigzagControlOutput *output)
{
  PidControlPendingCommand pending_command;
  ZigzagControlCommand core_command;
  ZigzagControlInput input;
  ZigzagControlOutput local_output;
  PidControlFault fault = PID_CONTROL_FAULT_NONE;
  uint32_t command_age_ms = UINT32_MAX;
  uint32_t depth_age_ms = UINT32_MAX;

  (void)memset(&local_output, 0, sizeof(local_output));
  local_output.fault = ZIGZAG_FAULT_DISARMED;

  if (s_ready == 0U)
  {
    fault = PID_CONTROL_FAULT_BAD_COMMAND;
    goto publish;
  }

  if (PidControlService_TakePendingCommand(&pending_command) != 0U)
  {
    s_active_command = pending_command.command;
    s_active_command_rx_tick_ms = pending_command.rx_tick_ms;
  }

  PidControlService_ProcessPendingConfig();

  if (s_active_command_rx_tick_ms != 0U)
  {
    command_age_ms = now_ms - s_active_command_rx_tick_ms;
  }

  if (s_active_command.mode == ZIGZAG_MODE_OFF ||
      s_active_command.armed == 0U)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_DISARMED;
    goto publish;
  }
  if (command_age_ms > PID_CONTROL_COMMAND_TIMEOUT_MS)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_COMMAND_TIMEOUT;
    goto publish;
  }
  if (s_pid_valid == 0U)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_PID_NOT_VALID;
    goto publish;
  }
  if (s_active_command.required_pid_crc != s_active_pid_crc)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_PID_CRC_MISMATCH;
    goto publish;
  }
  if (state == NULL || state->out.valid_for_control == 0U)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_EKF_INVALID;
    goto publish;
  }

  (void)memset(&input, 0, sizeof(input));
  input.valid_for_control = state->out.valid_for_control;
  input.depth_update_seq = state->depth_measurement_seq;
  input.depth_m = state->out.depth_m;
  input.pitch_rad = state->out.pitch;
  input.vertical_velocity_mps = state->out.vD;

  if (state->depth_measurement_seq != 0U)
  {
    depth_age_ms = now_ms - state->depth_measurement_tick_ms;
    input.depth_sample_valid =
        (depth_age_ms <= PID_CONTROL_DEPTH_TIMEOUT_MS) ? 1U : 0U;
  }

  if (s_active_command.mode == ZIGZAG_MODE_DEPTH_CASCADE &&
      input.depth_sample_valid == 0U)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_DEPTH_STALE;
    goto publish;
  }

  (void)memset(&core_command, 0, sizeof(core_command));
  core_command.mode = s_active_command.mode;
  core_command.armed = 1U;
  core_command.command_id = s_active_command.command_id;
  core_command.target_depth_m = s_active_command.target_depth_m;
  core_command.target_pitch_rad =
      s_active_command.target_pitch_deg * PID_CONTROL_DEG_TO_RAD;
  core_command.signed_thrust_percent =
      s_active_command.signed_thrust_percent;

  if (ZigzagControl_SetCommand(&s_controller, &core_command) == 0U)
  {
    ZigzagControl_Disarm(&s_controller);
    fault = PID_CONTROL_FAULT_BAD_COMMAND;
    goto publish;
  }

  ZigzagControl_Step(
      &s_controller, &input, dt_s, &local_output);
  if (local_output.output_valid == 0U)
  {
    fault = PID_CONTROL_FAULT_CORE_REJECTED;
  }
//控制器不健康，停机
publish:
  if (fault != PID_CONTROL_FAULT_NONE)
  {
    local_output.output_valid = 0U;
    local_output.worm_motor_rpm = 0;
    local_output.pump_enable = 0U;
    local_output.signed_thrust_percent = 0.0f;
  }

  PidControlService_PublishStatus(
      state, now_ms, command_age_ms, fault, &local_output);
  if (output != NULL)
  {
    *output = local_output;
  }
}

uint32_t PidControlService_CopyStatus(PidControlStatus *out_status)
{
  uint32_t seq;

  if (out_status == NULL)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  *out_status = g_pid_control_status;
  seq = g_pid_control_status.update_seq;
  taskEXIT_CRITICAL();
  return seq;
}
