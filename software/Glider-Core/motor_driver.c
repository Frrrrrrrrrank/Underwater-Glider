/*
 * motor_driver.c
 *
 * RS485 closed-loop stepper motor driver on USART2 bus.
 */

#include "motor_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
/* main.c 里定义为 volatile（Usart3Task 每 10ms 写、电机任务读），此处必须同型。 */
extern volatile uint8_t g_limit_switch_states[4];

#define MOTOR_RS485_UART (&huart2)

volatile UMotor g_umotor = {
  .nodes = {
    {
      .enabled = 1U,
      .node_id = 0x01U,
      .enable_speed_ctrl = 1U,
      .enable_poll = 1U,
      .target_speed_rpm = 0,
      .feedback_speed_rpm = 0,
      .status_word = 0U,
      .comm_ok = 0U,
      .valid = 0U,
      .tx_count = 0U,
      .rx_count = 0U,
      .timeout_count = 0U,
      .crc_error_count = 0U,
      .protocol_error_count = 0U,
      .last_exception_func = 0U,
      .last_exception_code = 0U,
      .reg_map = {
        .reg_cmd_speed_rpm = MOTOR_REG_CMD_SPEED_RPM,
        .reg_cmd_start = MOTOR_REG_CMD_START,
        .cmd_start_speed_mode_value = MOTOR_CMD_START_SPEED_MODE,
        .verify_read_start_reg = MOTOR_VERIFY_READ_START_REG,
        .verify_read_reg_count = MOTOR_VERIFY_READ_REG_COUNT,
        .status_read_start_reg = MOTOR_STATUS_READ_START_REG,
        .status_read_reg_count = MOTOR_STATUS_READ_REG_COUNT,
        .fault_read_start_reg = MOTOR_FAULT_READ_START_REG,
        .fault_read_reg_count = 0U
      },
      .verify_regs = {0},
      .status_regs = {0},
      .fault_regs = {0},
      .debug = {0}
    },
    {
      .enabled = 0U,
      .node_id = 0x02U,
      .enable_speed_ctrl = 1U,
      .enable_poll = 1U,
      .target_speed_rpm = 0,
      .feedback_speed_rpm = 0,
      .status_word = 0U,
      .comm_ok = 0U,
      .valid = 0U,
      .tx_count = 0U,
      .rx_count = 0U,
      .timeout_count = 0U,
      .crc_error_count = 0U,
      .protocol_error_count = 0U,
      .last_exception_func = 0U,
      .last_exception_code = 0U,
      .reg_map = {
        .reg_cmd_speed_rpm = MOTOR_REG_CMD_SPEED_RPM,
        .reg_cmd_start = MOTOR_REG_CMD_START,
        .cmd_start_speed_mode_value = MOTOR_CMD_START_SPEED_MODE,
        .verify_read_start_reg = MOTOR_VERIFY_READ_START_REG,
        .verify_read_reg_count = MOTOR_VERIFY_READ_REG_COUNT,
        .status_read_start_reg = MOTOR_STATUS_READ_START_REG,
        .status_read_reg_count = MOTOR_STATUS_READ_REG_COUNT,
        .fault_read_start_reg = MOTOR_FAULT_READ_START_REG,
        .fault_read_reg_count = 0U
      },
      .verify_regs = {0},
      .status_regs = {0},
      .fault_regs = {0},
      .debug = {0}
    },
    {
      .enabled = 0U,
      .node_id = 0x03U,
      .enable_speed_ctrl = 1U,
      .enable_poll = 1U,
      .target_speed_rpm = 0,
      .feedback_speed_rpm = 0,
      .status_word = 0U,
      .comm_ok = 0U,
      .valid = 0U,
      .tx_count = 0U,
      .rx_count = 0U,
      .timeout_count = 0U,
      .crc_error_count = 0U,
      .protocol_error_count = 0U,
      .last_exception_func = 0U,
      .last_exception_code = 0U,
      .reg_map = {
        .reg_cmd_speed_rpm = MOTOR_REG_CMD_SPEED_RPM,
        .reg_cmd_start = MOTOR_REG_CMD_START,
        .cmd_start_speed_mode_value = MOTOR_CMD_START_SPEED_MODE,
        .verify_read_start_reg = MOTOR_VERIFY_READ_START_REG,
        .verify_read_reg_count = MOTOR_VERIFY_READ_REG_COUNT,
        .status_read_start_reg = MOTOR_STATUS_READ_START_REG,
        .status_read_reg_count = MOTOR_STATUS_READ_REG_COUNT,
        .fault_read_start_reg = MOTOR_FAULT_READ_START_REG,
        .fault_read_reg_count = 0U
      },
      .verify_regs = {0},
      .status_regs = {0},
      .fault_regs = {0},
      .debug = {0}
    },
    {
      .enabled = 0U,
      .node_id = 0x04U,
      .enable_speed_ctrl = 1U,
      .enable_poll = 1U,
      .target_speed_rpm = 0,
      .feedback_speed_rpm = 0,
      .status_word = 0U,
      .comm_ok = 0U,
      .valid = 0U,
      .tx_count = 0U,
      .rx_count = 0U,
      .timeout_count = 0U,
      .crc_error_count = 0U,
      .protocol_error_count = 0U,
      .last_exception_func = 0U,
      .last_exception_code = 0U,
      .reg_map = {
        .reg_cmd_speed_rpm = MOTOR_REG_CMD_SPEED_RPM,
        .reg_cmd_start = MOTOR_REG_CMD_START,
        .cmd_start_speed_mode_value = MOTOR_CMD_START_SPEED_MODE,
        .verify_read_start_reg = MOTOR_VERIFY_READ_START_REG,
        .verify_read_reg_count = MOTOR_VERIFY_READ_REG_COUNT,
        .status_read_start_reg = MOTOR_STATUS_READ_START_REG,
        .status_read_reg_count = MOTOR_STATUS_READ_REG_COUNT,
        .fault_read_start_reg = MOTOR_FAULT_READ_START_REG,
        .fault_read_reg_count = 0U
      },
      .verify_regs = {0},
      .status_regs = {0},
      .fault_regs = {0},
      .debug = {0}
    }
  },
  .bus_diag = {0},
  .enable_write_cmd = 1U
};

#define g_motor_nodes g_umotor.nodes
#define g_motor_bus_diag g_umotor.bus_diag
#define g_motor_enable_write_cmd g_umotor.enable_write_cmd

typedef struct
{
  uint8_t initialized;
  uint8_t rr_index;
  int32_t last_sent_speed_rpm[MOTOR_BUS_MAX_NODES];
  uint32_t last_cmd_tick[MOTOR_BUS_MAX_NODES];
  uint32_t last_poll_tick[MOTOR_BUS_MAX_NODES];
  uint8_t last_motor_enable[MOTOR_BUS_MAX_NODES];
  /*
   * 撞限位后强制释放使能的覆盖位（只有电机任务读写，不是状态机）。
   * 1 = 无论应用层 motor_enable 是什么，对驱动器写 0x0039=0，让保持电流掉下来。
   * 0 = 跟随应用层 motor_enable。
   * 写失败不会卡住任何东西，下一拍自然重试；这是与旧状态机最本质的区别。
   */
  uint8_t limit_hold_release[MOTOR_BUS_MAX_NODES];
} MotorInternal;

static MotorInternal g_motor_internal = {
  0U, 0U, {0}, {0U}, {0U}, {0U}, {0U}
};

static uint16_t Motor_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static uint8_t Motor_BuildWriteSingleRegFrame(uint8_t node_id,
                                               uint16_t reg_addr,
                                               uint16_t reg_value,
                                               uint8_t *frame)
{
  uint16_t crc;

  frame[0] = node_id;
  frame[1] = 0x06U; /* Write single register */
  frame[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
  frame[3] = (uint8_t)(reg_addr & 0xFFU);
  frame[4] = (uint8_t)((reg_value >> 8) & 0xFFU);
  frame[5] = (uint8_t)(reg_value & 0xFFU);

  crc = Motor_Crc16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

static uint16_t Motor_BuildReadStatusFrame(uint8_t node_id,
                                           uint16_t start_reg,
                                           uint16_t reg_count,
                                           uint8_t *frame)
{
  uint16_t crc;

  frame[0] = node_id;
  frame[1] = 0x03U; /* Read holding registers */
  frame[2] = (uint8_t)((start_reg >> 8) & 0xFFU);
  frame[3] = (uint8_t)(start_reg & 0xFFU);
  frame[4] = (uint8_t)((reg_count >> 8) & 0xFFU);
  frame[5] = (uint8_t)(reg_count & 0xFFU);

  crc = Motor_Crc16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)((crc >> 8) & 0xFFU);

  return 8U;
}

/*
 * 发送新请求之前，必须把接收侧彻底清干净。
 *
 * 背景：huart2 关掉了 FIFO（main.c: HAL_UARTEx_DisableFifoMode），RDR 只能存 1 字节。
 * 上一帧只要有一个字节没被及时读走，第二个字节就会置 ORE（溢出），而 HAL 的
 * UART_WaitOnFlagUntilTimeout() 遇到 ORE 只清标志、不读走 RDR 里的残留字节。
 * 结果是：下一次事务的第一个 HAL_UART_Receive 会立刻读到上一帧的残留字节，
 * 第二个字节又立刻撞上 ORE 返回错误 —— 收发从此永久错开一帧且不会自愈，
 * 表现为“每次都超时、但一次事务几乎不耗时”；这就是本次限位卡死的根因
 *
 * 所以这里在每次发送前：先清 ORE/NE/FE/PE，再把 RDR 里的残留字节全部读掉，
 * 最后再清一次标志（读的过程中可能又来新字节）
 */
static void Motor_FlushRx(void)
{
  uint16_t guard = 0U;

  __HAL_UART_CLEAR_FLAG(MOTOR_RS485_UART,
                        UART_CLEAR_OREF | UART_CLEAR_NEF |
                        UART_CLEAR_FEF | UART_CLEAR_PEF);

  while ((__HAL_UART_GET_FLAG(MOTOR_RS485_UART, UART_FLAG_RXNE) != RESET) &&
         (guard < 64U))
  {
    (void)MOTOR_RS485_UART->Instance->RDR;
    guard++;
  }

  __HAL_UART_CLEAR_FLAG(MOTOR_RS485_UART,
                        UART_CLEAR_OREF | UART_CLEAR_NEF |
                        UART_CLEAR_FEF | UART_CLEAR_PEF);

  /* 把 HAL 句柄状态也复位，避免上一次 HAL_ERROR 留下的残留状态*/
  MOTOR_RS485_UART->ErrorCode = HAL_UART_ERROR_NONE;
  MOTOR_RS485_UART->RxState = HAL_UART_STATE_READY;
}

/*
 * 收包失败的统一处理：
 * 1) 区分“溢出/帧错位”(ORE) 与“真的没人应答”(超时)。原来两者都记 last_error=3，
 *    诊断上完全分不开，是上一轮排查方向被带偏的直接原因。
 * 2) 失败时这一帧剩下的字节可能还在路上；等它到齐（3ms @115200 ≈ 34 字节，
 *    比最长的 25 字节应答还长）再统一丢弃，避免污染下一次事务。
 */
static void Motor_RecordRxFailure(void)
{
  g_motor_bus_diag.total_timeout_count++;

  if (MOTOR_RS485_UART->ErrorCode == HAL_UART_ERROR_ORE)
  {
    g_motor_bus_diag.overrun_count++;
    g_motor_bus_diag.last_error = 24U; /* 接收溢出/帧错位 */
  }
  else
  {
    g_motor_bus_diag.last_error = 3U;  /* 超时无应答 */
  }
}

static uint8_t Motor_ReceiveExact(uint8_t *buffer, uint16_t expected_len, uint32_t timeout_ms)
{
  uint16_t index;
  HAL_StatusTypeDef rx_status;

  if (buffer == NULL || expected_len == 0U)
  {
    return 0U;
  }

  for (index = 0U; index < expected_len; index++)
  {
    rx_status = HAL_UART_Receive(MOTOR_RS485_UART, &buffer[index], 1U, timeout_ms);
    if (rx_status != HAL_OK)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t Motor_Transfer(const uint8_t *tx_frame,
                              uint16_t tx_len,
                              uint8_t *rx_frame,
                              uint16_t rx_capacity,
                              uint16_t *rx_got_len)
{
  HAL_StatusTypeDef tx_status;
  uint8_t header[3];
  uint16_t byte_count;
  uint16_t expected_len;
  uint8_t transfer_ok = 0U;

  if (tx_frame == NULL || rx_frame == NULL || rx_got_len == NULL || tx_len == 0U || rx_capacity == 0U)
  {
    g_motor_bus_diag.last_error = 1U;
    return 0U;
  }

  *rx_got_len = 0U;
  g_motor_bus_diag.last_rx_len = 0U;

  /*
   * MODBUS-RTU 帧间静默(t3.5)。从站没有帧结束符，靠"连续 3.5 个字符时间没有新字节"
   * 判定上一帧结束；本文件多处背靠背发帧(轮询链 stage1->stage2、速度块 0x0033->0x0037、
   * 位置块连发 5 笔)，原先帧间间隔为 0，从站可能把两帧粘成一帧去算 CRC ——
   * 实测驱动器报 0x0007=0x05 / 0x0008=0x51(MODBUS CRC 校验错误)。
   * 放在唯一的发送入口统一补静默，比在每个调用点分别加可靠。
   *
   * 注意这句 osDelay 落在"发之前"，此刻总线空闲、没有任何接收在进行，
   * 因此不可能引发接收超时/ORE；那类问题只发生在 Motor_ReceiveExact() 的逐字节循环内。
   * 任务循环末尾用的是相对 osDelay 而非 osDelayUntil，多花的时间只是让周期变长，
   * 不存在错过截止时间的问题。
   */
  osDelay(MOTOR_INTERFRAME_GAP_MS);

  /* 修改点，发送前先把接收侧清干净，否则一次溢出会让收发永久错开一帧。 */
  Motor_FlushRx();

  tx_status = HAL_UART_Transmit(MOTOR_RS485_UART, (uint8_t *)tx_frame, tx_len, MOTOR_UART_TX_TIMEOUT_MS);
  if (tx_status != HAL_OK)
  {
    g_motor_bus_diag.last_error = 2U;
    goto transfer_exit;
  }

  g_motor_bus_diag.total_tx_count++;
  g_motor_bus_diag.last_tx_len = tx_len;
  if (tx_len > sizeof(g_motor_bus_diag.last_tx_frame))
  {
    tx_len = (uint16_t)sizeof(g_motor_bus_diag.last_tx_frame);
  }
  memcpy((void *)g_motor_bus_diag.last_tx_frame, tx_frame, tx_len);

  /*
   * 这里不再 osDelay：HAL_UART_Transmit 返回时 TC 已置位，硬件 RS485 DE
   * （main.c 的 HAL_RS485Ex_Init）此刻已释放总线，可以立刻开始收。
   * 反而是延时期间应答先到、无人读取，会直接造成溢出。
   * 115200 下 8 字节应答只有约 0.7ms，20ms 的接收超时足够宽裕。
   */

  /* Read the fixed header first: addr + func + bytecount/exception code. */
  if (Motor_ReceiveExact(header, 3U, MOTOR_UART_RX_TIMEOUT_MS) == 0U)
  {
    Motor_RecordRxFailure();
    goto transfer_exit;
  }

  rx_frame[0] = header[0];
  rx_frame[1] = header[1];
  rx_frame[2] = header[2];

  if ((header[1] & 0x80U) != 0U)
  {
    expected_len = 5U;
  }
  else if (header[1] == 0x06U)
  {
    expected_len = 8U;
  }
  else
  {
    byte_count = header[2];
    expected_len = (uint16_t)(3U + byte_count + 2U);
  }

  if (expected_len > rx_capacity)
  {
    g_motor_bus_diag.last_error = 20U;
    goto transfer_exit;
  }

  if (expected_len > 3U)
  {
    if (Motor_ReceiveExact(&rx_frame[3], (uint16_t)(expected_len - 3U), MOTOR_UART_RX_TIMEOUT_MS) == 0U)
    {
      Motor_RecordRxFailure();
      goto transfer_exit;
    }
  }

  *rx_got_len = expected_len;
  g_motor_bus_diag.total_rx_count++;
  g_motor_bus_diag.last_rx_len = expected_len;
  if (expected_len > (uint16_t)sizeof(g_motor_bus_diag.last_rx_frame))
  {
    expected_len = (uint16_t)sizeof(g_motor_bus_diag.last_rx_frame);
  }
  memcpy((void *)g_motor_bus_diag.last_rx_frame, rx_frame, expected_len);

  transfer_ok = 1U;

transfer_exit:
  if (transfer_ok == 0U)
  {
    /* 失败时总线上可能还有半帧在路上：等齐后统一丢弃，保证下一次事务从干净状态开始。 */
    osDelay(MOTOR_RX_DRAIN_MS);
    Motor_FlushRx();
  }
  return transfer_ok;
}

static uint8_t Motor_ParseWriteAck(uint8_t node_id, const uint8_t *rx_frame, uint16_t rx_len)
{
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (rx_len < 8U)
  {
    g_motor_bus_diag.last_error = 4U;
    return 0U;
  }

  if (rx_frame[0] != node_id || rx_frame[1] != 0x06U)
  {
    g_motor_bus_diag.total_protocol_error_count++;
    g_motor_bus_diag.last_error = 5U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[7] << 8) | rx_frame[6]);
  calc_crc = Motor_Crc16(rx_frame, 6U);
  if (recv_crc != calc_crc)
  {
    g_motor_bus_diag.total_crc_error_count++;
    g_motor_bus_diag.last_error = 6U;
    return 0U;
  }

  return 1U;
}

static uint8_t Motor_ParseStatusResponse(uint8_t node_id,
                                         const uint8_t *rx_frame,
                                         uint16_t rx_len,
                                         uint16_t expected_reg_count,
                                         uint16_t status_word_index,
                                         uint16_t speed_word_index,
                                         uint16_t *status_word,
                                         int16_t *feedback_speed)
{
  uint16_t byte_count_expected;
  uint16_t status_byte_pos;
  uint16_t speed_byte_pos;
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (expected_reg_count == 0U || expected_reg_count > 16U)
  {
    g_motor_bus_diag.last_error = 13U;
    return 0U;
  }

  byte_count_expected = (uint16_t)(expected_reg_count * 2U);

  if (rx_len < (uint16_t)(5U + byte_count_expected))
  {
    g_motor_bus_diag.last_error = 7U;
    return 0U;
  }

  if (rx_frame[0] != node_id || rx_frame[1] != 0x03U || rx_frame[2] != (uint8_t)byte_count_expected)
  {
    g_motor_bus_diag.total_protocol_error_count++;
    g_motor_bus_diag.last_error = 8U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[(uint16_t)(4U + byte_count_expected)] << 8) |
                        rx_frame[(uint16_t)(3U + byte_count_expected)]);
  calc_crc = Motor_Crc16(rx_frame, (uint16_t)(3U + byte_count_expected));
  if (recv_crc != calc_crc)
  {
    g_motor_bus_diag.total_crc_error_count++;
    g_motor_bus_diag.last_error = 9U;
    return 0U;
  }

  if (status_word_index >= expected_reg_count || speed_word_index >= expected_reg_count)
  {
    g_motor_bus_diag.last_error = 14U;
    return 0U;
  }

  status_byte_pos = (uint16_t)(3U + status_word_index * 2U);
  speed_byte_pos = (uint16_t)(3U + speed_word_index * 2U);

  *status_word = (uint16_t)(((uint16_t)rx_frame[status_byte_pos] << 8) |
                            rx_frame[(uint16_t)(status_byte_pos + 1U)]);
  *feedback_speed = (int16_t)(((uint16_t)rx_frame[speed_byte_pos] << 8) |
                              rx_frame[(uint16_t)(speed_byte_pos + 1U)]);
  return 1U;
}

static uint8_t Motor_ParseReadBlockResponse(uint8_t node_id,
                                            const uint8_t *rx_frame,
                                            uint16_t rx_len,
                                            uint16_t max_out_words,
                                            uint16_t *out_words,
                                            uint16_t *out_reg_count)
{
  uint16_t i;
  uint16_t byte_count;
  uint16_t parsed_reg_count;
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (max_out_words == 0U || max_out_words > 16U || out_words == NULL || out_reg_count == NULL)
  {
    g_motor_bus_diag.last_error = 16U;
    return 0U;
  }

  if (rx_len < 5U)
  {
    g_motor_bus_diag.last_error = 17U;
    return 0U;
  }

  byte_count = rx_frame[2];
  if ((byte_count == 0U) || ((byte_count & 0x01U) != 0U) || (byte_count > 32U))
  {
    g_motor_bus_diag.total_protocol_error_count++;
    g_motor_bus_diag.last_error = 18U;
    return 0U;
  }

  parsed_reg_count = (uint16_t)(byte_count / 2U);
  if (parsed_reg_count > max_out_words)
  {
    g_motor_bus_diag.total_protocol_error_count++;
    g_motor_bus_diag.last_error = 18U;
    return 0U;
  }

  if (rx_len < (uint16_t)(5U + byte_count))
  {
    g_motor_bus_diag.last_error = 17U;
    return 0U;
  }

  if (rx_frame[0] != node_id || rx_frame[1] != 0x03U)
  {
    g_motor_bus_diag.total_protocol_error_count++;
    g_motor_bus_diag.last_error = 18U;
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[(uint16_t)(4U + byte_count)] << 8) |
                        rx_frame[(uint16_t)(3U + byte_count)]);
  calc_crc = Motor_Crc16(rx_frame, (uint16_t)(3U + byte_count));
  if (recv_crc != calc_crc)
  {
    g_motor_bus_diag.total_crc_error_count++;
    g_motor_bus_diag.last_error = 19U;
    return 0U;
  }

  for (i = 0U; i < parsed_reg_count; i++)
  {
    out_words[i] = (uint16_t)(((uint16_t)rx_frame[(uint16_t)(3U + i * 2U)] << 8) |
                              rx_frame[(uint16_t)(4U + i * 2U)]);
  }

  *out_reg_count = parsed_reg_count;

  return 1U;
}

static uint32_t Motor_BaudCodeToBps(uint16_t baud_code)
{
  switch (baud_code)
  {
    case 0U: return 9600U;
    case 1U: return 14400U;
    case 2U: return 19200U;
    case 3U: return 38400U;
    case 4U: return 115200U;
    case 5U: return 128000U;
    case 6U: return 256000U;
    default: return 0U;
  }
}

static uint8_t Motor_ParseExceptionResponse(uint8_t node_id,
                                            const uint8_t *rx_frame,
                                            uint16_t rx_len,
                                            uint8_t *exception_func,
                                            uint8_t *exception_code)
{
  uint16_t recv_crc;
  uint16_t calc_crc;

  if (rx_len < 5U)
  {
    return 0U;
  }

  if (rx_frame[0] != node_id)
  {
    return 0U;
  }

  if ((rx_frame[1] & 0x80U) == 0U)
  {
    return 0U;
  }

  recv_crc = (uint16_t)(((uint16_t)rx_frame[4] << 8) | rx_frame[3]);
  calc_crc = Motor_Crc16(rx_frame, 3U);
  if (recv_crc != calc_crc)
  {
    g_motor_bus_diag.total_crc_error_count++;
    g_motor_bus_diag.last_error = 10U;
    return 0U;
  }

  *exception_func = rx_frame[1] & 0x7FU;
  *exception_code = rx_frame[2];
  return 1U;
}

static void Motor_UpdateActiveNodeCount(void)
{
  uint32_t active = 0U;
  uint8_t i;

  for (i = 0U; i < MOTOR_BUS_MAX_NODES; i++)
  {
    if (g_motor_nodes[i].enabled != 0U)
    {
      active++;
    }
  }

  g_motor_bus_diag.active_node_count = active;
}

void UMotor_SetNodeEnabled(uint8_t slot, uint8_t node_id, uint8_t enabled)
{
  if (slot >= MOTOR_BUS_MAX_NODES)
  {
    return;
  }

  g_motor_nodes[slot].node_id = node_id;
  g_motor_nodes[slot].enabled = (enabled != 0U) ? 1U : 0U;
  if (enabled == 0U)
  {
    g_motor_nodes[slot].valid = 0U;
    g_motor_nodes[slot].comm_ok = 0U;
  }
}

void UMotor_SetNodeSpeedControl(uint8_t slot, uint8_t enable_speed_ctrl)
{
  if (slot >= UMOTOR_NODE_COUNT)
  {
    return;
  }

  g_motor_nodes[slot].enable_speed_ctrl = (enable_speed_ctrl != 0U) ? 1U : 0U;
}

void UMotor_SetNodePollEnabled(uint8_t slot, uint8_t enable_poll)
{
  if (slot >= UMOTOR_NODE_COUNT)
  {
    return;
  }

  g_motor_nodes[slot].enable_poll = (enable_poll != 0U) ? 1U : 0U;
}

uint8_t UMotor_SetNodeRegisterMap(uint8_t slot, const UMotorNodeRegisterMap *map)
{
  if (slot >= UMOTOR_NODE_COUNT || map == NULL)
  {
    return 0U;
  }

  if (map->verify_read_reg_count == 0U || map->verify_read_reg_count > UMOTOR_MAX_READ_REG_COUNT ||
      map->status_read_reg_count > UMOTOR_MAX_READ_REG_COUNT ||
      map->fault_read_reg_count > UMOTOR_MAX_READ_REG_COUNT)
  {
    return 0U;
  }

  g_motor_nodes[slot].reg_map = *map;
  return 1U;
}

/*
 * 限位方向判定（唯一实现，setter 与电机任务共用）。
 *
 * 硬件事实
 *   states[1] = PB4，被压时为 0 —— 挡 +RPM（上位机 S 方向）
 *   states[3] = PD7，被压时为 0 —— 挡 -RPM（上位机 W 方向，即“前限位”）
 * 只判当前电平、不做粘性锁存：滑块压住时一直挡，退出来就立刻放行，
 * 这样“撞哪边挡哪边、反方向永远能走”，不存在需要解锁的状态。
 */
static uint8_t Motor_IsLimitBlocked(int32_t rpm)
{
  if ((rpm > 0) && (g_limit_switch_states[1] == 0U))
  {
    return 1U;
  }
  if ((rpm < 0) && (g_limit_switch_states[3] == 0U))
  {
    return 1U;
  }
  return 0U;
}

void UMotor_SetTargetSpeedById(uint8_t node_id, int32_t target_speed_rpm)
{
  uint8_t i;
  if (target_speed_rpm > 32767)
  {
    target_speed_rpm = 32767;
  }
  if (target_speed_rpm < -32768)
  {
    target_speed_rpm = -32768;
  }

  /*
   * 这里不做限位钳位：target_speed_rpm 保存的是“应用层原始意图”。
   * 限位判定全部集中在电机任务 Motor_ProcessOneNode()（唯一发帧的地方），
   * 好处有二：
   *  1) 单一判定点，不会出现“setter 已经钳成 0、任务侧再也看不出撞了限位”
   *     从而无法触发释放使能的情况；
   *  2) Ozone 里 target_speed_rpm 能如实反映上位机要什么，
   *     和 g_motor_internal.last_sent_speed_rpm（实际发出去的）对照即可定位问题。
   */
  for (i = 0U; i < UMOTOR_NODE_COUNT; i++)
  {
    if ((g_motor_nodes[i].enabled != 0U) && (g_motor_nodes[i].node_id == node_id))
    {
      g_motor_nodes[i].target_speed_rpm = target_speed_rpm;
      break;
    }
  }
}

static void Motor_ProcessOneNode(uint8_t index, uint32_t now_tick)
{
  uint8_t tx_frame[8];
  uint8_t rx_frame[64];
  uint16_t tx_len;
  uint16_t rx_len;
  int16_t speed_cmd;
  uint16_t status_word;
  int16_t feedback_speed;
  uint16_t block_words[16];
  uint16_t got_regs;
  uint16_t req_start_reg;
  uint16_t req_count;
  uint16_t copy_count;
  uint8_t poll_ok;
  uint8_t exception_func;
  uint8_t exception_code;
  uint8_t node_id;
  uint8_t effective_enable;
  int32_t requested_rpm;
  volatile UMotorNodeState *node;
  volatile UMotorNodeDebugInfo *dbg;

  if (index >= MOTOR_BUS_MAX_NODES || g_motor_nodes[index].enabled == 0U)
  {
    return;
  }

  node = &g_motor_nodes[index];
  dbg = &node->debug;
  node_id = node->node_id;

  /*
   * 限位释放判定，每拍都重算，不锁存：
   *  - 当前目标方向撞限位 => 置 1，下面的使能块会写 0x0039=0，保持电流掉下来；
   *  - 目标非 0 且方向没被挡（例如撞前限位后上位机按 S 给 +200）=> 清 0，
   *    使能块自动写 0x0039=1，随后速度块发 0x0033+0x0037，电机反向退出限位。
   * 目标为 0 时保持当前释放状态不变，避免“松手就自动上电”。
   */
  /* 先取一次快照：ServoTask 随时可能改共享的 target_speed_rpm，两次读会读到不同值。 */
  requested_rpm = node->target_speed_rpm;

  if (Motor_IsLimitBlocked(requested_rpm) != 0U)
  {
    g_motor_internal.limit_hold_release[index] = 1U;
  }
  else if (requested_rpm != 0)
  {
    g_motor_internal.limit_hold_release[index] = 0U;
  }
  else
  {
    /* 目标 0：维持现状 */
  }
  //应用层意图与非限位释放
  effective_enable =
      ((node->motor_enable != 0U) &&
       (g_motor_internal.limit_hold_release[index] == 0U)) ? 1U : 0U;

  if (g_motor_enable_write_cmd != 0U)
  {
    /* Handle Manual Enable/Disable（叠加限位强制释放） */
    if (effective_enable != g_motor_internal.last_motor_enable[index])
    {
      if ((now_tick - g_motor_internal.last_cmd_tick[index]) >= MOTOR_CMD_MIN_INTERVAL_MS)
      {
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, MOTOR_REG_ENABLE, (effective_enable != 0U) ? 0x0001U : 0x0000U, tx_frame);
        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
            Motor_ParseWriteAck(node_id, rx_frame, rx_len) != 0U)
        {
          g_motor_internal.last_motor_enable[index] = effective_enable;
          node->comm_ok = 1U;
        }
        else
        {
          node->timeout_count++;
          node->comm_ok = 0U;
          /*
           * 丢应答时的方向性处理（这是整个修复的安全核心）：
           *  - 写“释放(0)”丢应答 => 不记账，下一拍继续重发。重复释放永远安全，
           *    绝不会出现“以为已经释放、实际还带电”的旧状态机式死锁。
           *  - 写“使能(1)”丢应答 => 记为已发，让流程继续走到速度+START。
           *    万一驱动器其实没收到，后续速度/START 也会失败，电机保持不动，安全。
           */
          if (effective_enable != 0U)
          {
            g_motor_internal.last_motor_enable[index] = 1U;
          }
        }

        if (effective_enable == 0U)
        {
          /* 已经（或正在）释放：本地速度记账清零，重新使能后必然重发速度+START。 */
          g_motor_internal.last_sent_speed_rpm[index] = 0;
        }

        /* 无论成败都推进节流时刻：失败下一拍重试，绝不在这里挂住整个节点。 */
        g_motor_internal.last_cmd_tick[index] = now_tick;

        if (effective_enable == g_motor_internal.last_motor_enable[index])
        {
          /* 本拍已完成一次使能写：一拍一个事务，剩下的下一拍再说。 */
          return;
        }
        /*
         * 写失败(释放方向)时故意不 return：让后面的后台轮询照常跑，
         * 否则总线一坏就连诊断都读不到，正是上一版死锁最难查的地方。
         * 中间的停机/位置/速度块都被刚刷新的 last_cmd_tick 节流挡住，不会误发。
         */
      }
    }
  }

  if (g_motor_enable_write_cmd != 0U)
  {
    /* Handle Stop Command */
    if (node->trigger_stop_cmd != 0U)
    {
      if ((now_tick - g_motor_internal.last_cmd_tick[index]) >= MOTOR_CMD_MIN_INTERVAL_MS)
      {
        node->trigger_stop_cmd = 0U; /* Clear trigger */
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, MOTOR_REG_STOP, node->stop_mode, tx_frame);
        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U)
        {
          /* Stopped. Set target speed to 0 to prevent automatic speed resume */
          node->target_speed_rpm = 0;
          g_motor_internal.last_sent_speed_rpm[index] = 0;
          g_motor_internal.last_cmd_tick[index] = now_tick;
        }
      }
    }
  }

  if (g_motor_enable_write_cmd != 0U)
  {
    /* Handle specific Position Run Trigger */
    if (node->trigger_pos_cmd != 0U)
    {
      if ((now_tick - g_motor_internal.last_cmd_tick[index]) >= MOTOR_CMD_MIN_INTERVAL_MS)
      {
        node->trigger_pos_cmd = 0U; /* Clear trigger */
        
        /* 1. Send total pulses low 16-bit */
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, MOTOR_REG_POS_PULSE_LOW, (uint16_t)((uint32_t)node->pos_cmd_pulses & 0xFFFFU), tx_frame);
        Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len);

        /* 2. Send total pulses high 16-bit */
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, MOTOR_REG_POS_PULSE_HIGH, (uint16_t)(((uint32_t)node->pos_cmd_pulses >> 16) & 0xFFFFU), tx_frame);
        Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len);

        /* 3. Send speed */
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, node->reg_map.reg_cmd_speed_rpm, node->pos_cmd_speed_rpm, tx_frame);
        Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len);

        /* 4. Send position mode (0=Relative, 1=Absolute) */
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, MOTOR_REG_POS_MODE, node->pos_cmd_is_absolute, tx_frame);
        Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len);

        /* 5. Send start position command */
        uint16_t pos_start_cmd = node->pos_cmd_is_absolute ? MOTOR_CMD_START_ABS_POS : MOTOR_CMD_START_REL_POS;
        tx_len = Motor_BuildWriteSingleRegFrame(node_id, node->reg_map.reg_cmd_start, pos_start_cmd, tx_frame);
        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U)
        {
          g_motor_internal.last_cmd_tick[index] = now_tick;
        }
      }
    }
  }

  if ((g_motor_enable_write_cmd != 0U) &&
      (node->enable_speed_ctrl != 0U) &&
      /* 驱动器处于释放状态（含限位强制释放）时不下发速度/START。 */
      (g_motor_internal.last_motor_enable[index] != 0U) &&
      ((now_tick - g_motor_internal.last_cmd_tick[index]) >= MOTOR_CMD_MIN_INTERVAL_MS))
  {
    if (g_motor_internal.last_sent_speed_rpm[index] != node->target_speed_rpm)
    {
      /*
       * 取一次快照后，本次事务全程只用 speed_cmd，绝不再重读共享的 target_speed_rpm。
       * 基线的 bug 就在这里：下面记账时重读了一次共享变量，若 ServoTask 在两句之间
       * 把目标从 -200 改成 0，就变成“实际发出去 -200、记账记成 0”，此后
       * last_sent == target 恒成立，再也不会补发停止。这是限位失效的真正老 bug。
       */
      speed_cmd = (int16_t)node->target_speed_rpm;

      /* 第二道钳位：快照之后、真正发帧之前，用当前限位电平再判一次。 */
      if (Motor_IsLimitBlocked((int32_t)speed_cmd) != 0U)
      {
        speed_cmd = 0;
      }

      tx_len = Motor_BuildWriteSingleRegFrame(node_id,
                                              node->reg_map.reg_cmd_speed_rpm,
                                              (uint16_t)speed_cmd,
                                              tx_frame);

      if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
          Motor_ParseWriteAck(node_id, rx_frame, rx_len) != 0U)
      {
        /*
         * 第三道钳位：0x0033 已写入、START 尚未发出。这一小段窗口正是
         * “限位刚好在此刻触发”的危险区，此时必须放弃 START。
         * 不记账 last_sent_speed_rpm，下一拍会按已被钳为 0 的目标重来。
         */
        if (Motor_IsLimitBlocked((int32_t)speed_cmd) != 0U)
        {
          node->comm_ok = 1U;
          g_motor_internal.last_cmd_tick[index] = now_tick;
          return;
        }

        tx_len = Motor_BuildWriteSingleRegFrame(node_id,
                                                node->reg_map.reg_cmd_start,
                                                node->reg_map.cmd_start_speed_mode_value,
                                                tx_frame);

        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
            Motor_ParseWriteAck(node_id, rx_frame, rx_len) != 0U)
        {
          /* 记账必须用真正发出去的 speed_cmd，不能重读共享变量。 */
          g_motor_internal.last_sent_speed_rpm[index] = (int32_t)speed_cmd;
          node->tx_count++;
          node->comm_ok = 1U;
          node->last_exception_func = 0U;
          node->last_exception_code = 0U;
        }
        else if (Motor_ParseExceptionResponse(node_id, rx_frame, rx_len, &exception_func, &exception_code) != 0U)
        {
          node->protocol_error_count++;
          node->comm_ok = 1U;
          node->valid = 0U;
          node->last_exception_func = (uint32_t)exception_func;
          node->last_exception_code = (uint32_t)exception_code;
          g_motor_bus_diag.last_exception_func = (uint32_t)exception_func;
          g_motor_bus_diag.last_exception_code = (uint32_t)exception_code;
          g_motor_bus_diag.last_error = 15U;
        }
        else
        {
          node->timeout_count++;
          node->protocol_error_count++;
          node->comm_ok = 0U;
        }
      }
      else if (Motor_ParseExceptionResponse(node_id, rx_frame, rx_len, &exception_func, &exception_code) != 0U)
      {
        node->protocol_error_count++;
        node->comm_ok = 1U;
        node->valid = 0U;
        node->last_exception_func = (uint32_t)exception_func;
        node->last_exception_code = (uint32_t)exception_code;
        g_motor_bus_diag.last_exception_func = (uint32_t)exception_func;
        g_motor_bus_diag.last_exception_code = (uint32_t)exception_code;
        g_motor_bus_diag.last_error = 11U;
      }
      else
      {
        node->timeout_count++;
        node->protocol_error_count++;
        node->comm_ok = 0U;
      }

      g_motor_internal.last_cmd_tick[index] = now_tick;
    }
  }

  if ((node->enable_poll != 0U) &&
      ((now_tick - g_motor_internal.last_poll_tick[index]) >= MOTOR_POLL_INTERVAL_MS))
  {
    poll_ok = 0U;
    dbg->poll_last_stage = 1U;

    req_start_reg = node->reg_map.verify_read_start_reg;
    req_count = node->reg_map.verify_read_reg_count;
    tx_len = Motor_BuildReadStatusFrame(node_id, req_start_reg, req_count, tx_frame);
    memcpy((void *)dbg->poll_last_tx_frame, (const void *)tx_frame, 8U);
    dbg->poll_last_req_start_reg = req_start_reg;
    dbg->poll_last_req_reg_count = req_count;

    if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
        Motor_ParseReadBlockResponse(node_id,
                                     rx_frame,
                                     rx_len,
                                     req_count,
                                     block_words,
                                     &got_regs) != 0U)
    {
      dbg->poll_last_rsp_len = rx_len;
      dbg->poll_last_rsp_reg_count = got_regs;
      memcpy((void *)dbg->poll_last_rx_frame, (const void *)rx_frame,
             (rx_len > UMOTOR_NODE_DEBUG_RX_MAX) ? UMOTOR_NODE_DEBUG_RX_MAX : rx_len);

      copy_count = (got_regs > UMOTOR_MAX_READ_REG_COUNT) ? UMOTOR_MAX_READ_REG_COUNT : got_regs;
      memcpy((void *)node->verify_regs, (const void *)block_words, copy_count * sizeof(uint16_t));

      dbg->reg_0014_node_id_cfg = (got_regs >= 1U) ? block_words[0U] : 0U;
      dbg->reg_0015_baud_cfg = (got_regs >= 2U) ? block_words[1U] : 0U;
      dbg->baud_rate_bps = Motor_BaudCodeToBps(dbg->reg_0015_baud_cfg);
      dbg->reg_0016_uart_fmt_cfg = (got_regs >= 3U) ? block_words[2U] : 0U;
      dbg->reg_0017_save_cmd = (got_regs >= 4U) ? block_words[3U] : 0U;

      poll_ok = 1U;
    }
    else if (Motor_ParseExceptionResponse(node_id, rx_frame, rx_len, &exception_func, &exception_code) != 0U)
    {
      node->protocol_error_count++;
      node->valid = 0U;
      node->comm_ok = 1U;
      node->last_exception_func = (uint32_t)exception_func;
      node->last_exception_code = (uint32_t)exception_code;
      g_motor_bus_diag.last_exception_func = (uint32_t)exception_func;
      g_motor_bus_diag.last_exception_code = (uint32_t)exception_code;
      g_motor_bus_diag.last_error = 12U;
      poll_ok = 0U;
    }

    if (poll_ok != 0U)
    {
      dbg->poll_last_stage = 2U;
      if (node->reg_map.status_read_reg_count != 0U)
      {
        req_start_reg = node->reg_map.status_read_start_reg;
        req_count = node->reg_map.status_read_reg_count;
        tx_len = Motor_BuildReadStatusFrame(node_id, req_start_reg, req_count, tx_frame);
        memcpy((void *)dbg->poll_last_tx_frame, (const void *)tx_frame, 8U);
        dbg->poll_last_req_start_reg = req_start_reg;
        dbg->poll_last_req_reg_count = req_count;

        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
            Motor_ParseReadBlockResponse(node_id,
                                         rx_frame,
                                         rx_len,
                                         req_count,
                                         block_words,
                                         &got_regs) != 0U)
        {
          dbg->poll_last_rsp_len = rx_len;
          dbg->poll_last_rsp_reg_count = got_regs;
          memcpy((void *)dbg->poll_last_rx_frame, (const void *)rx_frame,
                 (rx_len > UMOTOR_NODE_DEBUG_RX_MAX) ? UMOTOR_NODE_DEBUG_RX_MAX : rx_len);

          copy_count = (got_regs > UMOTOR_MAX_READ_REG_COUNT) ? UMOTOR_MAX_READ_REG_COUNT : got_regs;
          memcpy((void *)node->status_regs, (const void *)block_words, copy_count * sizeof(uint16_t));

          dbg->reg_0000_driver_version = (got_regs >= 1U) ? block_words[0U] : 0U;
          dbg->reg_0001_driver_model = (got_regs >= 2U) ? block_words[1U] : 0U;
          dbg->reg_0002_node_id_runtime = (got_regs >= 3U) ? block_words[2U] : 0U;
          dbg->reg_0003_work_mode = (got_regs >= 4U) ? block_words[3U] : 0U;
          dbg->reg_0004_status = (got_regs >= 5U) ? block_words[4U] : 0U;
          dbg->reg_0005_target_speed = (got_regs >= 6U) ? block_words[5U] : 0U;
          dbg->reg_0006_actual_speed = (got_regs >= 7U) ? block_words[6U] : 0U;
          dbg->reg_0007_error_code = (got_regs >= 8U) ? block_words[7U] : 0U;
          dbg->reg_0008_error_subcode = (got_regs >= 9U) ? block_words[8U] : 0U;
          /* 0x0009 输入端口状态: 判断限位是否也接进了驱动器 X 端子, 见 .h 中该字段说明。 */
          dbg->reg_0009_input_ports = (got_regs >= 10U) ? block_words[9U] : 0U;
          dbg->reg_000B_pos_low = (got_regs >= 12U) ? block_words[11U] : 0U;
          dbg->reg_000C_pos_high = (got_regs >= 13U) ? block_words[12U] : 0U;
          dbg->pos_signed_32 = (int32_t)(((uint32_t)dbg->reg_000C_pos_high << 16) | dbg->reg_000B_pos_low);

          node->status_word = dbg->reg_0004_status;
          node->feedback_speed_rpm = (int32_t)(int16_t)dbg->reg_0006_actual_speed;
        }
        else
        {
          poll_ok = 0U;
        }
      }
      else
      {
        /* Status read disabled by config. */
      }
    }

    if (poll_ok != 0U)
    {
      dbg->poll_last_stage = 3U;
      if (node->reg_map.fault_read_reg_count != 0U)
      {
        req_start_reg = node->reg_map.fault_read_start_reg;
        req_count = node->reg_map.fault_read_reg_count;
        tx_len = Motor_BuildReadStatusFrame(node_id, req_start_reg, req_count, tx_frame);
        memcpy((void *)dbg->poll_last_tx_frame, (const void *)tx_frame, 8U);
        dbg->poll_last_req_start_reg = req_start_reg;
        dbg->poll_last_req_reg_count = req_count;

        if (Motor_Transfer(tx_frame, tx_len, rx_frame, (uint16_t)sizeof(rx_frame), &rx_len) != 0U &&
            Motor_ParseReadBlockResponse(node_id,
                                         rx_frame,
                                         rx_len,
                                         req_count,
                                         block_words,
                                         &got_regs) != 0U)
        {
          dbg->poll_last_rsp_len = rx_len;
          dbg->poll_last_rsp_reg_count = got_regs;
          memcpy((void *)dbg->poll_last_rx_frame, (const void *)rx_frame,
                 (rx_len > UMOTOR_NODE_DEBUG_RX_MAX) ? UMOTOR_NODE_DEBUG_RX_MAX : rx_len);

          copy_count = (got_regs > UMOTOR_MAX_READ_REG_COUNT) ? UMOTOR_MAX_READ_REG_COUNT : got_regs;
          memcpy((void *)node->fault_regs, (const void *)block_words, copy_count * sizeof(uint16_t));

          dbg->reg_0194_latest_fault = (got_regs >= 1U) ? block_words[0U] : 0U;
          dbg->reg_0195_latest_fault_sub = (got_regs >= 2U) ? block_words[1U] : 0U;
          dbg->reg_0196_second_fault = (got_regs >= 3U) ? block_words[2U] : 0U;
          dbg->reg_0197_second_fault_sub = (got_regs >= 4U) ? block_words[3U] : 0U;
          dbg->reg_0198_third_fault = (got_regs >= 5U) ? block_words[4U] : 0U;
          dbg->reg_0199_third_fault_sub = (got_regs >= 6U) ? block_words[5U] : 0U;
          dbg->reg_019A_comm_fault_bits = (got_regs >= 7U) ? block_words[6U] : 0U;
        }
        else
        {
          poll_ok = 0U;
        }
      }
      else
      {
        /* Fault read disabled by config. */
      }
    }

    if (poll_ok != 0U)
    {
      status_word = dbg->reg_0004_status;
      feedback_speed = (int16_t)dbg->reg_0006_actual_speed;

      node->status_word = status_word;
      node->feedback_speed_rpm = (int32_t)feedback_speed;
      /*
       * 使能记账以驱动器上报的真实状态(0x0004 Bit0)为准，不再盲信 last_motor_enable。
       * 需要这么做的两个实测原因：
       *  1) 驱动器报警(Bit6，如 0x07 超差)时会【自行释放电机】，本地却仍以为使能着；
       *  2) 写 0x0039 的应答被 ORE 吞掉时，本地按"失败方向朝断电"记成了已发。
       * 两种情况下都不再有使能跳变，于是 0x0039 永远不重写、报警永远不清，
       * 表现为"发任何指令都不动，必须先按 X 再发"。
       * 同步之后，下一拍 effective_enable 与之不符就会自动补写 0x0039：
       * 补写 1 顺带产生使能上升沿，正是手册表 6.1 对 0x07 给出的清报警手段；
       * 补写 0 则是把没写进去的释放重发一次，方向仍然朝断电，安全。
       *
       * 刻意【不】在这里清 last_sent_speed_rpm：这样报警清除后电机不会拿旧目标
       * 自动重新冲出去，必须等上位机给一条新指令(target 变化)才会重发速度+START。
       * 代价是"重复按同一个方向键不会自动重试"，这是有意为之的安全取舍。
       */
      if ((uint8_t)(status_word & 0x0001U) !=
          g_motor_internal.last_motor_enable[index])
      {
        g_motor_internal.last_motor_enable[index] =
            (uint8_t)(status_word & 0x0001U);
        g_motor_bus_diag.enable_resync_count++;
      }
      node->valid = 1U;
      node->comm_ok = 1U;
      node->rx_count++;
      dbg->poll_ok_count++;
      node->last_exception_func = 0U;
      node->last_exception_code = 0U;
    }
    else
    {
      node->valid = 0U;
      node->comm_ok = 0U;
      node->timeout_count++;
      dbg->poll_fail_count++;
    }

    g_motor_internal.last_poll_tick[index] = now_tick;
  }
}

void UMotor_StartTask(void *argument)
{
  uint32_t now_tick;

  uint8_t init_index;

  (void)argument;
  g_motor_internal.initialized = 1U;
  g_motor_bus_diag.initialized = 1U;

  /*
   * 只复位 MCU 而驱动器不断电时，驱动器可能还停在“已使能 + 上次速度”状态。
   * 这里把本地记账置成一个不可能的值(0xFF)，强制第一拍必然写一次 0x0039，
   * 把驱动器同步到已知的释放状态，避免上电瞬间按旧速度冲出去。
   */
  for (init_index = 0U; init_index < MOTOR_BUS_MAX_NODES; init_index++)
  {
    g_motor_internal.last_motor_enable[init_index] = 0xFFU;
    g_motor_internal.last_sent_speed_rpm[init_index] = 0;
    g_motor_internal.limit_hold_release[init_index] = 0U;
  }

  /* ====== 测试代码：启动自动使能 ====== */
  /* 取消注释以下代码可开机自动转动测试 */
  /*
  g_umotor.enable_write_cmd = 1U; 
  UMotor_SetMotorEnable(0, 1U);   
  
  uint32_t last_test_tick = osKernelGetTickCount();
  uint8_t test_direction = 0U;
  */
  /* ===================================================== */

  for (;;)
  {
    now_tick = osKernelGetTickCount();
    g_motor_bus_diag.loop_count++;

    /* 测试：每8秒来回以100RPM转10圈 */
    /*
    if (now_tick - last_test_tick > 8000U)
    {
      last_test_tick = now_tick;
      if (test_direction == 0U)
      {
        UMotor_RunPosition(0, 100000, 100U, 1U); 
        test_direction = 1U;
      }
      else
      {
        UMotor_RunPosition(0, 0, 100U, 1U);     
        test_direction = 0U;
      }
    }
    */

    Motor_UpdateActiveNodeCount();
    Motor_ProcessOneNode(g_motor_internal.rr_index, now_tick);

    g_motor_internal.rr_index++;
    if (g_motor_internal.rr_index >= MOTOR_BUS_MAX_NODES)
    {
      g_motor_internal.rr_index = 0U;
    }

    osDelay(MOTOR_TASK_PERIOD_MS);
  }
}

/* Set enable state manually */
void UMotor_SetMotorEnable(uint8_t index, uint8_t enable)
{
  if (index < MOTOR_BUS_MAX_NODES)
  {
    g_umotor.nodes[index].motor_enable = (enable != 0U) ? 1U : 0U;
  }
}

/* Stop motor manually */
void UMotor_StopMotor(uint8_t index, uint16_t stop_mode)
{
  if (index < MOTOR_BUS_MAX_NODES)
  {
    g_umotor.nodes[index].stop_mode = stop_mode;
    g_umotor.nodes[index].trigger_stop_cmd = 1U;
  }
}

/* Trigger position run */
void UMotor_RunPosition(uint8_t index, int32_t pulses, uint16_t rpm, uint8_t is_absolute)
{
  if (index < MOTOR_BUS_MAX_NODES)
  {
    g_umotor.nodes[index].pos_cmd_pulses = pulses;
    g_umotor.nodes[index].pos_cmd_speed_rpm = rpm;
    g_umotor.nodes[index].pos_cmd_is_absolute = (is_absolute != 0U) ? 1U : 0U;
    g_umotor.nodes[index].trigger_pos_cmd = 1U;
  }
}
