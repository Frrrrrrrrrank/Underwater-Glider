/* 位置估算与双轴校准；只产生请求，执行器唯一写点仍为 StartServoTask。 */
#include "worm_axis.h"
#include "motor_driver.h"
#include "stepper_driver.h"
#include "limit_switch.h"
#include "FreeRTOS.h"
#include "task.h"

volatile WormAxisState g_worm_axis = {0};
extern volatile uint8_t g_limit_switch_states[4];

/* 工作副本由 ServoTask 独占，最后一次发布完整快照，读者不会观察到半拍状态。 */
static WormAxisState s_state;
static WormCalibrationOutput s_output;
static uint32_t s_last_valid_ms, s_pitch_phase_ms, s_roll_phase_ms;
static uint32_t s_release_baseline;
static int32_t s_release_pos;
static uint8_t s_have_valid, s_roll_command_sent, s_release_seen;
static volatile uint8_t s_request, s_cancel;

static void Publish(void)
{
  s_output.calib_state = s_state.calib_state;
  taskENTER_CRITICAL();
  g_worm_axis = s_state;
  taskEXIT_CRITICAL();
}

static void StopOutputs(void)
{
  s_output.pitch_rpm = 0;
  s_output.roll_rpm = 0;
  s_output.roll_action = WORM_ROLL_ACTION_STOP;
  s_output.roll_release_arm_mask = 0U;
}

static uint8_t PitchRunning(void)
{
  return (s_state.home_state == WORM_HOME_SEEK ||
          s_state.home_state == WORM_HOME_BACKOFF) ? 1U : 0U;
}

static void FailCalibration(WormCalibrationFail reason)
{
  s_state.calib_state = (reason == WORM_CALIB_FAIL_CANCELLED ||
                        reason == WORM_CALIB_FAIL_PID) ?
                       WORM_CALIB_CANCELLED : WORM_CALIB_FAILED;
  s_state.calib_fail = (uint8_t)reason;
  s_state.calib_hold = 1U;
  s_state.roll.valid = 0U;
  if (PitchRunning() != 0U)
  {
    s_state.home_state = WORM_HOME_FAILED;
    s_state.home_fail = (reason == WORM_CALIB_FAIL_ROLL) ?
                       WORM_HOME_FAIL_OTHER_AXIS : WORM_HOME_FAIL_CANCELLED;
  }
  if (s_state.roll.home_state != WORM_ROLL_HOME_DONE &&
      s_state.roll.home_state != WORM_ROLL_HOME_FAILED)
  {
    s_state.roll.home_state = WORM_ROLL_HOME_FAILED;
    s_state.roll.home_fail = (reason == WORM_CALIB_FAIL_PITCH) ?
                            WORM_ROLL_FAIL_OTHER_AXIS : WORM_ROLL_FAIL_CANCELLED;
  }
  StopOutputs();
}

static void PitchFail(WormHomingFail reason)
{
  s_state.home_state = WORM_HOME_FAILED;
  s_state.home_fail = (uint8_t)reason;
  FailCalibration(WORM_CALIB_FAIL_PITCH);
}

static void RollFail(WormRollHomingFail reason)
{
  s_state.roll.home_state = WORM_ROLL_HOME_FAILED;
  s_state.roll.home_fail = (uint8_t)reason;
  FailCalibration(WORM_CALIB_FAIL_ROLL);
}

static void PitchStep(uint32_t now_ms, uint8_t front_hit, uint8_t rear_hit)
{
  /* 已完成的 pitch 仍需保持反馈有效，不能等待 roll 时丢参考却报告联合成功。 */
  if (s_state.data_fresh == 0U)
  {
    PitchFail(WORM_HOME_FAIL_DATA_STALE);
    return;
  }
  if (front_hit != 0U && rear_hit != 0U)
  {
    PitchFail(WORM_HOME_FAIL_LIMIT_CONFLICT);
    return;
  }
  s_output.pitch_rpm = 0;
  switch ((WormHomingState)s_state.home_state)
  {
    case WORM_HOME_SEEK:
      if ((now_ms - s_pitch_phase_ms) >= WORM_HOME_SEEK_TIMEOUT_MS)
      {
        PitchFail(WORM_HOME_FAIL_SEEK_TIMEOUT);
      }
      else if (front_hit != 0U)
      {
        s_state.home_state = WORM_HOME_BACKOFF;
        s_pitch_phase_ms = now_ms;
        s_output.pitch_rpm = WORM_HOME_BACKOFF_RPM;
      }
      else { s_output.pitch_rpm = -WORM_HOME_SEEK_RPM; }
      break;
    case WORM_HOME_BACKOFF:
      if ((now_ms - s_pitch_phase_ms) >= WORM_HOME_BACKOFF_TIMEOUT_MS)
      {
        PitchFail(WORM_HOME_FAIL_BACKOFF_TIMEOUT);
      }
      else if (front_hit == 0U)
      {
        int64_t rear = (int64_t)s_state.pulse - WORM_STROKE_PULSES;
        if (rear < INT32_MIN)
        {
          PitchFail(WORM_HOME_FAIL_POSITION_RANGE);
          break;
        }
        s_state.pulse_home_front = s_state.pulse;
        s_state.pulse_min_seen = (int32_t)rear;
        s_state.pulse_max_seen = s_state.pulse;
        s_state.reference_kind = WORM_REF_HOMED;
        s_state.home_state = WORM_HOME_DONE;
        s_state.home_done_count++;
      }
      else { s_output.pitch_rpm = WORM_HOME_BACKOFF_RPM; }
      break;
    default:
      break;
  }
}

static void BeginBackoff(uint32_t now_ms, const StepperState *stepper, uint8_t negative)
{
  s_state.roll.home_state = negative ? WORM_ROLL_HOME_BACKOFF_NEG :
                                      WORM_ROLL_HOME_BACKOFF_POS;
  s_roll_phase_ms = now_ms;
  s_release_baseline = stepper->release_seq;
  s_release_seen = 0U;
  s_roll_command_sent = 0U;
  s_output.roll_rpm = negative ? WORM_ROLL_BACKOFF_RPM : -WORM_ROLL_BACKOFF_RPM;
  s_output.roll_release_arm_mask = negative ? ROLL_LIMIT_NEG_ACTIVE :
                                             ROLL_LIMIT_POS_ACTIVE;
  s_output.roll_action = WORM_ROLL_ACTION_START_BACKOFF;
}

static void BackoffStep(uint32_t now_ms, const StepperState *stepper, uint8_t negative)
{
  uint8_t mask = negative ? ROLL_LIMIT_NEG_ACTIVE : ROLL_LIMIT_POS_ACTIVE;
  if ((now_ms - s_roll_phase_ms) >= WORM_ROLL_BACKOFF_TIMEOUT_MS)
  {
    RollFail(WORM_ROLL_FAIL_BACKOFF_TIMEOUT);
    return;
  }
  if (s_roll_command_sent == 0U)
  {
    s_output.roll_action = WORM_ROLL_ACTION_START_BACKOFF;
    return;
  }
  if (s_release_seen == 0U)
  {
    if (stepper->release_seq != s_release_baseline)
    {
      if (stepper->release_mask != mask || stepper->running != 0U ||
          stepper->last_stop_reason != STEPPER_STOP_RELEASE ||
          stepper->current_pos != stepper->release_pos)
      {
        RollFail(WORM_ROLL_FAIL_DRIVER);
        return;
      }
      s_release_seen = 1U;
      s_release_pos = stepper->release_pos;
    }
    else if (stepper->running == 0U)
    {
      RollFail(WORM_ROLL_FAIL_DRIVER);
      return;
    }
    else
    {
      s_output.roll_action = WORM_ROLL_ACTION_KEEP_BACKOFF;
      return;
    }
  }
  /* 释放沿已在 IRQ 停机并捕获；确认稳定期间只停，不重新发送连续速度。 */
  s_output.roll_action = WORM_ROLL_ACTION_STOP;
  s_output.roll_rpm = 0;
  s_output.roll_release_arm_mask = 0U;
  if (stepper->running != 0U || stepper->current_pos != s_release_pos)
  {
    RollFail(WORM_ROLL_FAIL_DRIVER);
    return;
  }
  if ((stepper->blocked_mask & mask) != 0U) { return; }

  if (negative != 0U)
  {
    s_state.roll.pulse_min = s_release_pos;
    s_state.roll.home_state = WORM_ROLL_HOME_SEEK_POS;
    s_roll_phase_ms = now_ms;
    s_output.roll_action = WORM_ROLL_ACTION_CONTINUOUS;
    s_output.roll_rpm = WORM_ROLL_SEEK_RPM;
  }
  else
  {
    int64_t span = (int64_t)s_release_pos - s_state.roll.pulse_min;
    s_state.roll.pulse_max = s_release_pos;
    if (span < WORM_ROLL_MIN_SPAN_PULSES || span > WORM_ROLL_MAX_SPAN_PULSES ||
        span > INT32_MAX)
    {
      RollFail(WORM_ROLL_FAIL_SPAN);
      return;
    }
    s_state.roll.span_pulses = (int32_t)span;
    s_state.roll.valid = 1U;
    s_state.roll.home_state = WORM_ROLL_HOME_CENTER;
    s_output.roll_center_pos = (int32_t)((int64_t)s_state.roll.pulse_min + span / 2);
    s_output.roll_rpm = WORM_ROLL_CENTER_RPM;
    s_output.roll_action = WORM_ROLL_ACTION_START_CENTER;
    s_roll_phase_ms = now_ms;
    s_roll_command_sent = 0U;
  }
}

static void RollStep(uint32_t now_ms, const StepperState *stepper)
{
  if ((stepper->raw_limit_mask & ROLL_LIMIT_ALL_ACTIVE) == ROLL_LIMIT_ALL_ACTIVE)
  {
    RollFail(WORM_ROLL_FAIL_LIMIT_CONFLICT);
    return;
  }
  /* STOP 保留历史停因；新轮第一条命令不能被上一轮错误拦住。
   * 已提交的连续/保持动作发生运行期错误仍必须失败，不能靠重发掩盖。 */
  if ((s_output.roll_action == WORM_ROLL_ACTION_CONTINUOUS ||
       s_output.roll_action == WORM_ROLL_ACTION_KEEP_BACKOFF ||
       s_output.roll_action == WORM_ROLL_ACTION_KEEP_CENTER) &&
      (stepper->last_stop_reason == STEPPER_STOP_RANGE ||
       stepper->last_stop_reason == STEPPER_STOP_START_FAILED))
  {
    RollFail(WORM_ROLL_FAIL_DRIVER);
    return;
  }
  switch ((WormRollHomingState)s_state.roll.home_state)
  {
    case WORM_ROLL_HOME_SEEK_NEG:
    case WORM_ROLL_HOME_SEEK_POS:
    {
      uint8_t negative = (s_state.roll.home_state == WORM_ROLL_HOME_SEEK_NEG);
      uint8_t mask = negative ? ROLL_LIMIT_NEG_ACTIVE : ROLL_LIMIT_POS_ACTIVE;
      if ((now_ms - s_roll_phase_ms) >= WORM_ROLL_SEEK_TIMEOUT_MS)
      {
        RollFail(WORM_ROLL_FAIL_SEEK_TIMEOUT);
      }
      else if ((stepper->raw_limit_mask & mask) != 0U)
      {
        BeginBackoff(now_ms, stepper, negative);
      }
      else
      {
        s_output.roll_action = WORM_ROLL_ACTION_CONTINUOUS;
        s_output.roll_rpm = negative ? -WORM_ROLL_SEEK_RPM : WORM_ROLL_SEEK_RPM;
        s_output.roll_release_arm_mask = 0U;
      }
      break;
    }
    case WORM_ROLL_HOME_BACKOFF_NEG:
      BackoffStep(now_ms, stepper, 1U);
      break;
    case WORM_ROLL_HOME_BACKOFF_POS:
      BackoffStep(now_ms, stepper, 0U);
      break;
    case WORM_ROLL_HOME_CENTER:
      if ((now_ms - s_roll_phase_ms) >= WORM_ROLL_CENTER_TIMEOUT_MS)
      {
        RollFail(WORM_ROLL_FAIL_CENTER_TIMEOUT);
      }
      else if (s_roll_command_sent == 0U)
      {
        s_output.roll_action = WORM_ROLL_ACTION_START_CENTER;
      }
      else if (stepper->target_pos != s_output.roll_center_pos)
      {
        RollFail(WORM_ROLL_FAIL_DRIVER);
      }
      else if (stepper->running != 0U)
      {
        s_output.roll_action = WORM_ROLL_ACTION_KEEP_CENTER;
      }
      else if (stepper->last_stop_reason == STEPPER_STOP_COMPLETED &&
               stepper->current_pos == s_output.roll_center_pos)
      {
        s_state.roll.home_state = WORM_ROLL_HOME_DONE;
        s_output.roll_action = WORM_ROLL_ACTION_STOP;
        s_output.roll_rpm = 0;
      }
      else { RollFail(WORM_ROLL_FAIL_DRIVER); }
      break;
    default:
      s_output.roll_action = WORM_ROLL_ACTION_STOP;
      s_output.roll_rpm = 0;
      break;
  }
}

void WormAxis_RequestCalibration(void)
{
  taskENTER_CRITICAL();
  /* 运行中的重复包不能延长任何阶段的超时；取消邮箱优先，不被 start 覆盖。 */
  if (g_worm_axis.calib_state != WORM_CALIB_RUNNING && s_cancel == 0U)
  {
    s_request = 1U;
  }
  taskEXIT_CRITICAL();
}

void WormAxis_CancelCalibration(void)
{
  taskENTER_CRITICAL();
  s_request = 0U;
  s_cancel = 1U;
  taskEXIT_CRITICAL();
}

void WormAxis_CancelCalibrationForPid(void)
{
  taskENTER_CRITICAL();
  s_request = 0U;
  s_cancel = 0U;
  taskEXIT_CRITICAL();
  if (s_state.calib_state == WORM_CALIB_RUNNING)
  {
    FailCalibration(WORM_CALIB_FAIL_PID);
    Publish();
  }
}

void WormAxis_ReleaseCalibrationHold(void)
{
  if (s_state.calib_state != WORM_CALIB_RUNNING)
  {
    s_state.calib_hold = 0U;
    Publish();
  }
}

uint8_t WormAxis_GetCalibrationOutput(WormCalibrationOutput *out)
{
  if (out != NULL) { *out = s_output; }
  return s_state.calib_hold;
}

void WormAxis_ConfirmRollCommand(uint8_t accepted)
{
  if (s_state.calib_state != WORM_CALIB_RUNNING) { return; }
  if (accepted == 0U) { RollFail(WORM_ROLL_FAIL_DRIVER); }
  else if (s_output.roll_action == WORM_ROLL_ACTION_START_BACKOFF ||
           s_output.roll_action == WORM_ROLL_ACTION_START_CENTER)
  {
    s_roll_command_sent = 1U;
    s_output.roll_action = (s_output.roll_action == WORM_ROLL_ACTION_START_BACKOFF) ?
                          WORM_ROLL_ACTION_KEEP_BACKOFF : WORM_ROLL_ACTION_KEEP_CENTER;
  }
  Publish();
}

void WormAxis_RequestHoming(void) { WormAxis_RequestCalibration(); }
void WormAxis_CancelHoming(void) { WormAxis_CancelCalibration(); }

uint8_t WormAxis_GetHomingWormRpm(int32_t *out_rpm)
{
  uint8_t active = (s_state.calib_state == WORM_CALIB_RUNNING && PitchRunning());
  if (out_rpm != NULL) { *out_rpm = active ? s_output.pitch_rpm : 0; }
  return active;
}

void WormAxis_Step(uint32_t now_ms)
{
  uint32_t valid, silence_ms;
  int32_t pulse;
  uint16_t status_word;
  uint8_t request, cancel, front_hit, rear_hit;
  StepperState stepper = {0};
  uint8_t stepper_valid = Stepper_CopyState(&stepper);
  int64_t span;

  taskENTER_CRITICAL();
  valid = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].valid;
  status_word = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].status_word;
  pulse = g_umotor.nodes[WORM_AXIS_MOTOR_SLOT].debug.pos_signed_32;
  front_hit = (g_limit_switch_states[WORM_FRONT_LIMIT_INDEX] == 0U);
  rear_hit = (g_limit_switch_states[3] == 0U);
  request = s_request;
  cancel = s_cancel;
  s_request = s_cancel = 0U;
  taskEXIT_CRITICAL();

  if (valid != 0U) { s_last_valid_ms = now_ms; s_have_valid = 1U; }
  silence_ms = s_have_valid ? now_ms - s_last_valid_ms : WORM_AXIS_REF_LOST_MS;
  s_state.data_fresh = (silence_ms < WORM_AXIS_STALE_MS);
  if (silence_ms >= WORM_AXIS_REF_LOST_MS)
  {
    if (s_state.reference_kind != WORM_REF_NONE)
    {
      s_state.reference_kind = WORM_REF_NONE;
      s_state.reference_lost_count++;
      if (s_state.home_state == WORM_HOME_DONE) { s_state.home_state = WORM_HOME_IDLE; }
    }
  }
  else if (s_state.data_fresh != 0U && s_state.reference_kind == WORM_REF_NONE)
  {
    s_state.reference_kind = WORM_REF_POWER_ON;
    s_state.pulse_min_seen = s_state.pulse_max_seen = pulse;
  }
  s_state.pulse = pulse;
  s_state.rev = (float)pulse / WORM_PULSES_PER_REV;
  s_state.moving = ((status_word & 0x0002U) != 0U);
  s_state.alarm = ((status_word & 0x0040U) != 0U);
  s_state.roll.pulse = stepper.current_pos;

  if (cancel != 0U)
  {
    FailCalibration(WORM_CALIB_FAIL_CANCELLED);
  }
  else if (request != 0U && s_state.calib_state != WORM_CALIB_RUNNING)
  {
    s_state.calib_state = WORM_CALIB_RUNNING;
    s_state.calib_fail = WORM_CALIB_FAIL_NONE;
    s_state.calib_hold = 1U;
    s_state.home_state = WORM_HOME_SEEK;
    s_state.home_fail = WORM_HOME_FAIL_NONE;
    s_state.roll.valid = 0U;
    s_state.roll.span_pulses = 0;
    s_state.roll.pulse_min = s_state.roll.pulse_max = 0;
    s_state.roll.home_state = WORM_ROLL_HOME_SEEK_NEG;
    s_state.roll.home_fail = WORM_ROLL_FAIL_NONE;
    s_pitch_phase_ms = s_roll_phase_ms = now_ms;
    s_roll_command_sent = s_release_seen = 0U;
    StopOutputs(); /* 启动拍先停，两轴从下一拍共同推进。 */
  }
  else if (s_state.calib_state == WORM_CALIB_RUNNING)
  {
    PitchStep(now_ms, front_hit, rear_hit);
    if (s_state.calib_state == WORM_CALIB_RUNNING)
    {
      if (stepper_valid == 0U) { RollFail(WORM_ROLL_FAIL_DRIVER); }
      else { RollStep(now_ms, &stepper); }
    }
    if (s_state.calib_state == WORM_CALIB_RUNNING &&
        s_state.home_state == WORM_HOME_DONE &&
        s_state.roll.home_state == WORM_ROLL_HOME_DONE)
    {
      s_state.calib_state = WORM_CALIB_DONE;
      StopOutputs();
    }
  }

  if (s_state.data_fresh != 0U && s_state.reference_kind == WORM_REF_POWER_ON)
  {
    if (pulse < s_state.pulse_min_seen) { s_state.pulse_min_seen = pulse; }
    if (pulse > s_state.pulse_max_seen) { s_state.pulse_max_seen = pulse; }
  }
  span = (int64_t)s_state.pulse_max_seen - s_state.pulse_min_seen;
  s_state.span_pulses = (span >= 0 && span <= INT32_MAX) ? (int32_t)span : 0;
  s_state.span_valid = (s_state.reference_kind != WORM_REF_NONE &&
                       span >= WORM_SPAN_MIN_PULSES && span <= INT32_MAX);
  if (s_state.span_valid != 0U)
  {
    float ratio = (float)((int64_t)s_state.pulse_max_seen - pulse) / (float)span;
    s_state.stroke_percent = 100.0f * ratio;
    s_state.mm_from_front = WORM_STROKE_MM * ratio;
  }
  else { s_state.stroke_percent = s_state.mm_from_front = 0.0f; }

  if (stepper_valid == 0U || stepper.last_stop_reason == STEPPER_STOP_RANGE)
  {
    s_state.roll.valid = 0U;
  }
  s_state.roll.stroke_percent = s_state.roll.valid ?
      100.0f * (float)((int64_t)stepper.current_pos - s_state.roll.pulse_min) /
      (float)s_state.roll.span_pulses : 0.0f;
  s_state.update_count++;
  Publish();
}

void WormAxis_CopyState(WormAxisState *out_state)
{
  if (out_state == NULL) { return; }
  taskENTER_CRITICAL();
  *out_state = g_worm_axis;
  taskEXIT_CRITICAL();
}
