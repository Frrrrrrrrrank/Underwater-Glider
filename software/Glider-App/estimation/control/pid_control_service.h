/*
 * pid_control_service.h
 *
 * Ethernet PID 会话与 ZigzagControl 纯算法之间的任务上下文服务。
 * Ethernet task 只提交固定大小邮箱；ServoTask 每 20ms 调 Step，完成参数
 * CRC/解码、命令租约（心跳包）和状态新鲜度检查，再取得纯计算输出。
 */
#ifndef PID_CONTROL_SERVICE_H
#define PID_CONTROL_SERVICE_H

#include "state_estimate.h"
#include "zigzag_control.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_CONTROL_PARAM_TABLE_BYTES 64U
#define PID_CONTROL_PARAM_FLOAT_COUNT 16U
#define PID_CONTROL_COMMAND_TIMEOUT_MS 500U
#define PID_CONTROL_DEPTH_TIMEOUT_MS 600U

typedef struct
{
  ZigzagControlMode mode;
  uint8_t armed;
  uint16_t command_id;
  float target_depth_m;
  /* 通信/上位机边界单位为 deg；服务应用命令时转换为 rad。 */
  float target_pitch_deg;
  /* 正=沿现有 AUTO 的 I 方向，负=沿 K 方向；不进入 PID。 */
  float signed_thrust_percent;
  uint32_t required_pid_crc;
} PidControlCommand;

typedef enum
{
  PID_CONTROL_FAULT_NONE = 0U,
  PID_CONTROL_FAULT_DISARMED = 1U,
  PID_CONTROL_FAULT_COMMAND_TIMEOUT = 2U,
  PID_CONTROL_FAULT_PID_NOT_VALID = 3U,
  PID_CONTROL_FAULT_PID_CRC_MISMATCH = 4U,
  PID_CONTROL_FAULT_EKF_INVALID = 5U,
  PID_CONTROL_FAULT_DEPTH_STALE = 6U,
  PID_CONTROL_FAULT_BAD_COMMAND = 7U,
  PID_CONTROL_FAULT_CORE_REJECTED = 8U
} PidControlFault;

typedef enum
{
  PID_CONTROL_CONFIG_REJECT_NONE = 0U,
  PID_CONTROL_CONFIG_REJECT_ARMED = 1U,
  PID_CONTROL_CONFIG_REJECT_CRC = 2U,
  PID_CONTROL_CONFIG_REJECT_VALUE = 3U
} PidControlConfigRejectReason;

typedef struct
{
  uint32_t update_seq;
  uint32_t update_tick_ms;
  uint32_t command_age_ms;
  uint32_t depth_measurement_seq;
  uint32_t required_pid_crc;
  uint32_t active_pid_crc;
  uint32_t config_request_count;
  uint32_t config_apply_count;
  uint32_t config_reject_count;
  uint16_t command_id;
  ZigzagControlMode requested_mode;
  uint8_t arm_requested;
  uint8_t effectively_armed;
  uint8_t pid_valid;
  PidControlFault fault;
  PidControlConfigRejectReason config_reject_reason;
  float requested_depth_m;
  float requested_pitch_deg;
  float requested_thrust_percent;
  ZigzagControlOutput control_output;
} PidControlStatus;//PID控制的单次状态

/* 在调度器启动前初始化一次；默认配置不标记为已验证参数。 */
void PidControlService_Init(void);

/* Ethernet task 调用：只做基本字段检查、固定大小复制和序号发布。 */
uint8_t PidControlService_SubmitCommand(
    const PidControlCommand *command, uint32_t rx_tick_ms);
uint8_t PidControlService_SubmitConfigTable(
    const uint8_t table_bytes[PID_CONTROL_PARAM_TABLE_BYTES],
    uint32_t expected_crc);

/* ServoTask 每 20ms 调用；所有 CRC、浮点解码和控制计算均在这里发生。 */
void PidControlService_Step(
    const StateEstimate *state,
    uint32_t now_ms,
    float dt_s,
    ZigzagControlOutput *output);

/* Ethernet task/诊断侧整体复制，禁止逐字段散读。 */
uint32_t PidControlService_CopyStatus(PidControlStatus *out_status);

/* Ozone 观察镜像；控制和遥测代码必须走 CopyStatus。 */
extern PidControlStatus g_pid_control_status;

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_SERVICE_H */
