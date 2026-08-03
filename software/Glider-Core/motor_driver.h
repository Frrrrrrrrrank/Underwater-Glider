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
/* 事务失败后等待半帧到齐再排空的时间：3ms @115200 ≈ 34 字节 > 最长应答 25 字节。 */
#define MOTOR_RX_DRAIN_MS              3U

/*
 * Default MODBUS register map (adjust to your motor manual).
 */
#define MOTOR_REG_CMD_SPEED_RPM        0x0033U
#define MOTOR_REG_CMD_START            0x0037U
#define MOTOR_CMD_START_SPEED_MODE     0x0001U
#define MOTOR_CMD_START_REL_POS        0x0002U
#define MOTOR_CMD_START_ABS_POS        0x0004U
#define MOTOR_REG_ENABLE               0x0039U
#define MOTOR_REG_POS_PULSE_LOW        0x0034U
#define MOTOR_REG_POS_PULSE_HIGH       0x0035U
#define MOTOR_REG_POS_MODE             0x0036U
#define MOTOR_REG_STOP                 0x0038U
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
  /* --- 轮询通讯阶段统计 --- */
  uint32_t poll_ok_count;           /* 轮询成功次数 */
  uint32_t poll_fail_count;         /* 轮询失败或异常次数 */
  uint8_t poll_last_stage;          /* 当前查询停留阶段 1=配置,2=状态,3=警报 */
  uint16_t poll_last_req_start_reg; 
  uint16_t poll_last_req_reg_count;
  uint16_t poll_last_rsp_reg_count;
  uint16_t poll_last_rsp_len;

  /* --- 配置组 (0x0014~0x0017) --- */
  uint16_t reg_0014_node_id_cfg;    /* [参数]固化的节点ID */
  uint16_t reg_0015_baud_cfg;       /* [参数]波特率代码(0~6) */
  uint32_t baud_rate_bps;           /* 解析出的波特率(比如115200) */
  uint16_t reg_0016_uart_fmt_cfg;   /* [参数]串口格式配置 */
  uint16_t reg_0017_save_cmd;       /* [参数]保存至EEPROM命令字 */

  /* --- 状态组 (0x0000~0x000F) --- */
  uint16_t reg_0000_driver_version; /* [状态]固件版本 */
  uint16_t reg_0001_driver_model;   /* [状态]驱动器型号 */
  uint16_t reg_0002_node_id_runtime;/* [状态]实时有效节点ID */
  uint16_t reg_0003_work_mode;      /* [状态]当前工作模式(例如速度/位置/等) */
  uint16_t reg_0004_status;         /* [状态]核心状态字寄存器(运行,错报,启用) */
  uint16_t reg_0005_target_speed;   /* [状态]实际受控目标转速 */
  uint16_t reg_0006_actual_speed;   /* [状态]当前反馈转速 */
  uint16_t reg_0007_error_code;     /* [状态]当前故障大类代码 */
  uint16_t reg_0008_error_subcode;  /* [状态]当前故障子类代码 */
  uint16_t reg_000B_pos_low;        /* [状态]反馈绝对脉冲低16位 */
  uint16_t reg_000C_pos_high;       /* [状态]反馈绝对脉冲高16位 */
  int32_t pos_signed_32;            /* 完整拼接为32位的带符号反馈脉冲 */

  /* --- 历史报错组 (0x0194~0x019A) --- */
  uint16_t reg_0194_latest_fault;      /* 故障发生历史: 最近一次大类 */
  uint16_t reg_0195_latest_fault_sub;  /* 故障发生历史: 最近一次子类 */
  uint16_t reg_0196_second_fault;      
  uint16_t reg_0197_second_fault_sub;  
  uint16_t reg_0198_third_fault;       
  uint16_t reg_0199_third_fault_sub;   
  uint16_t reg_019A_comm_fault_bits;   

  /* --- 最新报文抓取 (用于除错硬件通讯底层) --- */
  uint8_t poll_last_tx_frame[8];
  uint8_t poll_last_rx_frame[UMOTOR_NODE_DEBUG_RX_MAX];
} UMotorNodeDebugInfo;

typedef struct
{
  /* --- 节点使能与配置 --- */
  uint8_t enabled;             /* 节点是否启用 */
  uint8_t node_id;             /* 对应的Modbus从机地址(Node ID) */
  uint8_t enable_speed_ctrl;   /* 是否开启周期性的速度指令下发 */
  uint8_t enable_poll;         /* 是否开启后台寄存器轮询(读取状态/报错等) */
  uint8_t motor_enable;        /* 【控制】手动使能开关：1=使能, 0=释放 */

  /* --- 运行时状态信息 --- */
  uint32_t valid;              /* 通讯且数据有效 (1=正常, 0=无响应或错误) */
  uint32_t comm_ok;            /* 是否建立过通讯 (收到过ACK即为1) */
  uint16_t status_word;        /* 状态字镜像(0x0004寄存器), 可判断报警/到位等状态 */
  int32_t feedback_speed_rpm;  /* 电机实际反馈转速 (RPM) */
  
  /* --- 速度模式参数 --- */
  int32_t target_speed_rpm;    /* 【控制】目标转速设定 (RPM) */
  
  /* --- 位置模式参数 --- */
  uint8_t trigger_pos_cmd;     /* 【触发】下发一次位置模式指令(发完自动清零) */
  uint8_t pos_cmd_is_absolute; /* 【控制】位置模式: 1=绝对位置, 0=相对位置 */
  uint16_t pos_cmd_speed_rpm;  /* 【控制】位置模式运行的最高转速 (RPM) */
  int32_t pos_cmd_pulses;      /* 【控制】位置模式目标脉冲数(结合您的细分看) */

  /* --- 停机控制参数 --- */
  uint8_t trigger_stop_cmd;    /* 【触发】触发一次停机指令(发完自动清零) */
  uint8_t stop_mode;           /* 【控制】停机模式: 0=正常减速停, 1=紧急停 */

  /* --- 诊断与通讯统计 --- */
  uint32_t tx_count;           /* 成功发送帧数 */
  uint32_t rx_count;           /* 成功接收且校验通过帧数 */
  uint32_t timeout_count;      /* 超时无响应次数 */
  uint32_t crc_error_count;    /* CRC校验错误次数 */
  uint32_t protocol_error_count;/* 协议格式错误或异常帧次数 */
  uint32_t last_exception_func;/* 上次Modbus异常返回的功能码 */
  uint32_t last_exception_code;/* 上次Modbus异常返回的错误码 */

  /* --- 底层寄存器收发映射缓存 (Debug专用) --- */
  UMotorNodeRegisterMap reg_map; 
  uint16_t verify_regs[UMOTOR_MAX_READ_REG_COUNT]; 
  uint16_t status_regs[UMOTOR_MAX_READ_REG_COUNT]; 
  uint16_t fault_regs[UMOTOR_MAX_READ_REG_COUNT];  
  UMotorNodeDebugInfo debug;   /* 详细的单寄存器数据解析树 */
} UMotorNodeState;

typedef struct
{
  uint32_t initialized;
  uint32_t loop_count;
  uint32_t active_node_count;
  uint32_t total_tx_count;
  uint32_t total_rx_count;
  uint32_t total_timeout_count;
  uint32_t overrun_count;      /* 其中属于接收溢出/帧错位的次数(last_error=24) */
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
void UMotor_SetMotorEnable(uint8_t index, uint8_t enable);
void UMotor_RunPosition(uint8_t index, int32_t pulses, uint16_t rpm, uint8_t is_absolute);
void UMotor_StopMotor(uint8_t index, uint16_t stop_mode);
void UMotor_StartTask(void *argument);

#endif /* MOTOR_DRIVER_H */
