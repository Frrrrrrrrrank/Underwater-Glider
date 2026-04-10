/*
 * motor_driver.h
 *
 * RS485 closed-loop stepper motor driver on USART2.
 *
 * This module provides a task-based, bus-friendly driver skeleton:
 * - Non-blocking task loop with bounded UART timeouts
 * - Multi-device slots on one RS485 bus
 * - Runtime configurable target speed per node
 *
 * NOTE:
 * Register addresses and scaling must match the motor vendor protocol.
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#define UMOTOR_NODE_COUNT              4U
#define MOTOR_BUS_MAX_NODES            UMOTOR_NODE_COUNT
#define MOTOR_RS485_BAUD_RATE          115200U
#define MOTOR_TASK_PERIOD_MS           10U
#define MOTOR_CMD_MIN_INTERVAL_MS      20U
#define MOTOR_POLL_INTERVAL_MS         100U
#define MOTOR_UART_TX_TIMEOUT_MS       20U
#define MOTOR_UART_RX_TIMEOUT_MS       20U

/*
 * Default MODBUS register map (adjust to your motor manual).
 */
#define MOTOR_REG_CMD_SPEED_RPM        0x0033U
#define MOTOR_REG_CMD_START            0x0037U
#define MOTOR_CMD_START_SPEED_MODE     0x0001U
#define MOTOR_REG_STATUS_WORD          0x0004U
#define MOTOR_REG_FEEDBACK_SPEED_RPM   0x0006U

/* Verification read window: 0x0014~0x001D (10 regs, documented config group 1). */
#define MOTOR_VERIFY_READ_START_REG    0x0014U
#define MOTOR_VERIFY_READ_REG_COUNT    10U

/* Fixed status read window: 0x0000~0x000F (version/state/speed/position). */
#define MOTOR_STATUS_READ_START_REG    0x0000U
#define MOTOR_STATUS_READ_REG_COUNT    16U

/* Fixed fault read window: 0x0194~0x019A (fault history + comm fault bits). */
#define MOTOR_FAULT_READ_START_REG     0x0194U
#define MOTOR_FAULT_READ_REG_COUNT     7U

/* Datasheet 4.2.1 status parameter group: 0x0000~0x0013 (20 regs). */
#define MOTOR_STATUS_GROUP_START_REG   0x0000U
#define MOTOR_STATUS_GROUP_REG_COUNT   20U
#define MOTOR_STATUS_GROUP_CHUNK1_CNT  16U
#define MOTOR_STATUS_GROUP_CHUNK2_REG  0x0010U
#define MOTOR_STATUS_GROUP_CHUNK2_CNT  4U
#define UMOTOR_MAX_READ_REG_COUNT       16U
#define UMOTOR_NODE_DEBUG_RX_MAX        40U

typedef struct
{
  uint16_t reg_cmd_speed_rpm;
  uint16_t reg_cmd_start;
  uint16_t cmd_start_speed_mode_value;

  /* Fixed read block 1: usually 0x0014 group */
  uint16_t verify_read_start_reg;
  uint16_t verify_read_reg_count;

  /* Fixed read block 2: usually 0x0000 group */
  uint16_t status_read_start_reg;
  uint16_t status_read_reg_count;

  /* Fixed read block 3: usually 0x0194 group */
  uint16_t fault_read_start_reg;
  uint16_t fault_read_reg_count;
} UMotorNodeRegisterMap;

typedef struct
{
  uint32_t poll_ok_count;
  uint32_t poll_fail_count;
  uint8_t poll_last_stage;
  uint16_t poll_last_req_start_reg;
  uint16_t poll_last_req_reg_count;
  uint16_t poll_last_rsp_reg_count;
  uint16_t poll_last_rsp_len;

  /* 0x0014~0x0017: config group 1 */
  uint16_t reg_0014_node_id_cfg;
  uint16_t reg_0015_baud_cfg;
  uint32_t baud_rate_bps;
  uint16_t reg_0016_uart_fmt_cfg;
  uint16_t reg_0017_save_cmd;

  /* 0x0000~0x000F: status group */
  uint16_t reg_0000_driver_version;
  uint16_t reg_0001_driver_model;
  uint16_t reg_0002_node_id_runtime;
  uint16_t reg_0003_work_mode;
  uint16_t reg_0004_status;
  uint16_t reg_0005_target_speed;
  uint16_t reg_0006_actual_speed;
  uint16_t reg_0007_error_code;
  uint16_t reg_0008_error_subcode;
  uint16_t reg_000B_pos_low;
  uint16_t reg_000C_pos_high;
  int32_t pos_signed_32;

  /* 0x0194~0x019A: fault group */
  uint16_t reg_0194_latest_fault;
  uint16_t reg_0195_latest_fault_sub;
  uint16_t reg_0196_second_fault;
  uint16_t reg_0197_second_fault_sub;
  uint16_t reg_0198_third_fault;
  uint16_t reg_0199_third_fault_sub;
  uint16_t reg_019A_comm_fault_bits;

  uint8_t poll_last_tx_frame[8];
  uint8_t poll_last_rx_frame[UMOTOR_NODE_DEBUG_RX_MAX];
} UMotorNodeDebugInfo;

typedef struct
{
  uint8_t enabled;
  uint8_t node_id;
  uint8_t enable_speed_ctrl;
  uint8_t enable_poll;
  int32_t target_speed_rpm;
  int32_t feedback_speed_rpm;
  uint16_t status_word;
  uint32_t comm_ok;
  uint32_t valid;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t timeout_count;
  uint32_t crc_error_count;
  uint32_t protocol_error_count;
  uint32_t last_exception_func;
  uint32_t last_exception_code;
  UMotorNodeRegisterMap reg_map;
  uint16_t verify_regs[UMOTOR_MAX_READ_REG_COUNT];
  uint16_t status_regs[UMOTOR_MAX_READ_REG_COUNT];
  uint16_t fault_regs[UMOTOR_MAX_READ_REG_COUNT];
  UMotorNodeDebugInfo debug;
} UMotorNodeState;

typedef struct
{
  uint32_t initialized;
  uint32_t loop_count;
  uint32_t active_node_count;
  uint32_t total_tx_count;
  uint32_t total_rx_count;
  uint32_t total_timeout_count;
  uint32_t total_crc_error_count;
  uint32_t total_protocol_error_count;
  uint8_t last_tx_frame[16];
  uint16_t last_tx_len;
  uint8_t last_rx_frame[32];
  uint16_t last_rx_len;
  uint32_t last_error;
  uint32_t last_exception_func;
  uint32_t last_exception_code;
} UMotorBusDiag;

typedef struct
{
  UMotorNodeState nodes[UMOTOR_NODE_COUNT];
  UMotorBusDiag bus_diag;
  uint8_t enable_write_cmd;
} UMotor;

extern volatile UMotor g_umotor;

void UMotor_SetNodeEnabled(uint8_t slot, uint8_t node_id, uint8_t enabled);
void UMotor_SetNodeSpeedControl(uint8_t slot, uint8_t enable_speed_ctrl);
void UMotor_SetNodePollEnabled(uint8_t slot, uint8_t enable_poll);
uint8_t UMotor_SetNodeRegisterMap(uint8_t slot, const UMotorNodeRegisterMap *map);
void UMotor_SetTargetSpeedById(uint8_t node_id, int32_t target_speed_rpm);
void UMotor_StartTask(void *argument);

#endif /* MOTOR_DRIVER_H */
