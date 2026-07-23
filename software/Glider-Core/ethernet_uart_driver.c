#include "ethernet_uart_driver.h"
#include "cmsis_os.h"
#include "displacement_sensor.h"
#include "motor_driver.h"
#include "pressure_sensor_driver.h"
#include "pump_driver.h"
#include "servo_driver.h"
#include "limit_switch.h"
#include "ekf_bridge.h"   /* EkfBridge_CopyState / StateEstimate / g_ekf_depth_meas */
#include "ekf_task.h"     /* g_ekf_imu_hz */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>         /* isfinite：定点格式化前剔除 NaN/Inf */

typedef struct
{
  uint32_t frame_count;
  uint32_t crc_error_count;
  uint32_t rx_overflow_count;
  uint32_t rx_irq_count;
  uint32_t rx_byte_count;
  uint32_t rx_poll_byte_count;
  uint32_t rx_rearm_ok_count;
  uint32_t rx_rearm_fail_count;
  uint32_t rx_init_status;
  uint32_t uart4_irq_count;
  uint32_t uart4_error_count;
  uint32_t uart4_last_error;
  uint32_t uart_rxne_flag;
  uint32_t uart_error_flags;
  uint32_t recent_rx_len;
  uint8_t recent_rx_bytes[16];
  uint32_t task_loop_count;
  uint32_t last_function_code;
  uint32_t last_rx_function;
  int32_t last_rx_state;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  uint32_t raw_updated;
  float roll, pitch, yaw;
  uint32_t euler_updated;
  float q0, q1, q2, q3;
  uint32_t quat_updated;
  float height, temperature;
  float pressure, pressure_contrast;
  uint32_t baro_updated;
  uint8_t version_high;
  uint8_t version_mid;
  uint8_t version_low;
} IMUDebugView;

typedef struct
{
  uint32_t update_count;
  uint32_t frame_type;
  uint32_t checksum_valid;
  uint32_t is_bdpwi;
  uint32_t is_bdepi;
  uint32_t is_gnrmc;
  char sentence[24];
  char utc[16];
  uint32_t mode_code;
  uint32_t status_code;
  float longitude_deg;
  float latitude_deg;
  uint32_t position_valid;
  uint32_t signal_good;
  uint32_t tx_mode;
  uint32_t card_id_known;
  uint32_t card_mismatch;
  uint32_t has_cached_fix;
  uint32_t shortmsg_pending;
  uint32_t ready_to_send;
  char local_card_id[24];
  char target_card_id[24];
  char last_tx_text[32];
  char last_text[96];
} GliderDebugQuickView;

typedef struct
{
  uint32_t boot_stage;
  uint32_t alive_counter;
  uint32_t rtos_tick;
  uint32_t rx_line_count;
  uint32_t rx_overflow_count;
  uint32_t tx_attempt_count;
  uint32_t tx_ok_count;
  uint32_t tx_fail_count;
  uint32_t tx_last_status;
  uint32_t tx_last_uart_error;
  char last_tx_text[32];
  char card_id[24];
  uint32_t card_id_known;
  uint32_t shortmsg_sent_count;
  char last_shortmsg_payload[96];
  uint32_t last_rx_update_count;
  uint32_t last_rx_text_len;
  char last_rx_text[512];
  uint32_t last_rx_bin_len;
  uint8_t last_rx_bin[64];
  uint32_t is_rd_mode;
  uint32_t is_bdpwi;
  char bdpwi_utc[16];
  uint32_t bdpwi_field1;
  uint32_t bdpwi_field2;
  uint32_t bdpwi_field1_valid;
  uint32_t bdpwi_field2_valid;
  char bdpwi_latitude_raw[16];
  char bdpwi_longitude_raw[16];
  char bdpwi_heading_raw[16];
  int32_t bdpwi_latitude_x100;
  int32_t bdpwi_longitude_x100;
  int32_t bdpwi_heading_x100;
  uint32_t bdpwi_latitude_valid;
  uint32_t bdpwi_longitude_valid;
  uint32_t bdpwi_heading_valid;
  char bdpwi_latitude_dm[20];
  char bdpwi_latitude_ns[4];
  char bdpwi_longitude_dm[20];
  char bdpwi_longitude_ew[4];
  int32_t bdpwi_latitude_deg_e7;
  int32_t bdpwi_longitude_deg_e7;
  float bdpwi_latitude_deg;
  float bdpwi_longitude_deg;
  uint32_t bdpwi_latitude_deg_valid;
  uint32_t bdpwi_longitude_deg_valid;
  uint32_t bdpwi_old_beam_count;
  uint32_t bdpwi_new_beam_count;
  uint32_t bdpwi_best_d_cnr;
  uint32_t bdpwi_best_p_cnr;
  uint32_t bdpwi_good_beam_count;
  uint32_t rd_signal_good;
  uint32_t tx_mode;
  uint32_t is_gnrmc;
  char gnrmc_utc[16];
  char gnrmc_status[4];
  char gnrmc_date[16];
  char gnrmc_latitude_dm[20];
  char gnrmc_latitude_ns[4];
  char gnrmc_longitude_dm[20];
  char gnrmc_longitude_ew[4];
  int32_t gnrmc_latitude_deg_e7;
  int32_t gnrmc_longitude_deg_e7;
  float gnrmc_latitude_deg;
  float gnrmc_longitude_deg;
  uint32_t gnrmc_status_valid;
  uint32_t gnrmc_latitude_valid;
  uint32_t gnrmc_longitude_valid;
  uint32_t is_bdepi;
  char bdepi_sender_card[24];
  char bdepi_receiver_card[24];
  char bdepi_utc[16];
  uint32_t bdepi_freq_code;
  uint32_t bdepi_emergency_code;
  uint32_t bdepi_report_mode;
  uint32_t bdepi_freq_valid;
  uint32_t bdepi_emergency_valid;
  uint32_t bdepi_report_mode_valid;
  char bdepi_longitude_dm[20];
  char bdepi_longitude_ew[4];
  char bdepi_latitude_dm[20];
  char bdepi_latitude_ns[4];
  int32_t bdepi_longitude_deg_e7;
  int32_t bdepi_latitude_deg_e7;
  float bdepi_longitude_deg;
  float bdepi_latitude_deg;
  uint32_t bdepi_longitude_valid;
  uint32_t bdepi_latitude_valid;
  char bdepi_altitude_raw[16];
  int32_t bdepi_altitude_x100;
  uint32_t bdepi_altitude_valid;
  char bdepi_altitude_unit[4];
  char bdepi_altitude_abnormal[8];
  char bdepi_altitude_abn_unit[4];
  char bdepi_precision_indicator[8];
  char bdepi_multi_solution[8];
  char bdepi_rescue_subtype[8];
  char bdepi_rescue_center_type[8];
  uint32_t bdepi_data_len;
  uint32_t bdepi_data_len_valid;
  char bdepi_data_payload[48];
} GliderDebugView;

/* EthernetUartSystemSnapshot 定义已上移至 ethernet_uart_driver.h，供以太网上报
 * 与 SD 卡日志任务共用。 */

extern volatile GliderDebugView g_dbg_view;
extern volatile GliderDebugQuickView g_dbg_quick;
extern volatile IMUDebugView g_imu_debug;
extern volatile uint8_t g_limit_switch_states[LIMIT_SWITCH_COUNT];

volatile EthernetUartSystemSnapshot g_ethernet_system_snapshot = {0};

volatile EthernetUartState g_ethernet_uart_state = {0};
volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack = {0};
volatile uint8_t g_ethernet_rx_message_flag = 0U;
volatile uint32_t g_ethernet_control_last_rx_tick = 0U;
volatile uint8_t g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_NONE;

/* EKF 遥测帧累计成功发送计数（Ozone 观察用） */
volatile uint32_t g_ekf_telem_tx_count = 0U;

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_byte = 0U;
static volatile uint8_t s_eth_rx_ring[ETHERNET_UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint32_t s_tx_busy = 0U;
static uint32_t s_last_command_tick = 0U;
/* 两路遥测独立控制；默认只发状态 EKF，磁标定原始流必须由采集脚本显式开启。 */
static volatile uint8_t s_ekf_telem_enabled = 1U;
static volatile uint8_t s_mag_telem_enabled = 0U;
/* HAL_BUSY 时不篡改 HAL 内部状态，由 Ethernet task 在 READY 后重试挂接。 */
static volatile uint8_t s_rx_rearm_pending = 0U;

#define UGCP_MAGIC_SIZE             ETHERNET_UART_FRAME_MAGIC_SIZE
#define UGCP_FIXED_HEADER_SIZE      ETHERNET_UART_FRAME_HEADER_SIZE
#define UGCP_CRC_SIZE               ETHERNET_UART_FRAME_CRC_SIZE
#define UGCP_MAX_FRAME_SIZE         ETHERNET_UART_FRAME_MAX_SIZE
#define UGCP_CONTROL_PAYLOAD_SIZE  19U
#define UGCP_PARAM_GROUP_MAG_CAL    1U
#define UGCP_PARAM_BLOCK_COUNT      4U
#define UGCP_PARAM_BLOCK_DATA_SIZE 16U
#define UGCP_PARAM_BLOCK_PAYLOAD_SIZE 19U
#define UGCP_PARAM_COMMIT_PAYLOAD_SIZE 7U

static const uint8_t s_ugcp_magic[UGCP_MAGIC_SIZE] = {'U', 'G', 'C', 'P'};
static uint8_t s_ugcp_parse_buffer[UGCP_MAX_FRAME_SIZE];
static uint16_t s_ugcp_parse_length = 0U;
static uint32_t s_ugcp_last_byte_tick = 0U;
static uint8_t s_ugcp_have_sequence = 0U;
static uint8_t s_snapshot_pending = 0U;

typedef struct
{
  uint8_t table[EKF_MAG_CAL_TABLE_BYTES];
  uint8_t group;
  uint8_t transaction;
  uint8_t received_mask;
} EthernetUartParamStage;

static EthernetUartParamStage s_param_stage;

static void EthernetUartDriver_RearmReceive(void);
static void EthernetUartDriver_RingPush(uint8_t byte);
static bool EthernetUartDriver_RingPop(uint8_t *byte);
static void EthernetUartDriver_ProcessReceivedData(void);
static void EthernetUartDriver_ProcessLine(const char *line);
static void EthernetUartDriver_FeedFrameParser(uint8_t byte);
static void EthernetUartDriver_ProcessFrameParser(void);
static void EthernetUartDriver_DispatchFrame(
    uint8_t message_type, const uint8_t *payload, uint16_t payload_length);
static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length);
static uint32_t EthernetUartDriver_ReadLeU32(const uint8_t *bytes);
static int32_t EthernetUartDriver_ReadLeI32(const uint8_t *bytes);
static uint32_t EthernetUartDriver_Crc32(const uint8_t *data, uint16_t length);
static void EthernetUartDriver_UpdateSystemSnapshot(void);

static int16_t EthernetUartDriver_ClampI16(int32_t value)
{
  if (value > 32767)
  {
    return 32767;
  }
  if (value < -32768)
  {
    return -32768;
  }
  return (int16_t)value;
}

static int16_t EthernetUartDriver_ScaleFloatToI16(float value, float scale)
{
  float scaled = value * scale;
  if (scaled >= 0.0f)
  {
    scaled += 0.5f;
  }
  else
  {
    scaled -= 0.5f;
  }
  return EthernetUartDriver_ClampI16((int32_t)scaled);
}

static int32_t EthernetUartDriver_ScaleFloatToI32(float value, float scale)
{
  float scaled = value * scale;
  if (scaled >= 0.0f)
  {
    scaled += 0.5f;
  }
  else
  {
    scaled -= 0.5f;
  }
  return (int32_t)scaled;
}

void EthernetUartDriver_UpdateSystemSnapshot(void)
{
  uint32_t alarm = 0U;
  uint8_t limit_bits = 0U;
  uint32_t i;
  StateEstimate ekf_state;

  for (i = 0U; i < 4U; i++)
  {
    if (g_limit_switch_states[i] != 0U)
    {
      limit_bits |= (uint8_t)(1U << i);
    }
  }

  if (g_displacement_diag.last_error != 0U)
  {
    alarm |= (1UL << 0);
  }
  if (g_displacement_diag.timeout_count != 0U)
  {
    alarm |= (1UL << 1);
  }
  if (g_displacement_diag.crc_error_count != 0U)
  {
    alarm |= (1UL << 2);
  }
  if (g_imu_debug.crc_error_count != 0U)
  {
    alarm |= (1UL << 3);
  }
  if (g_imu_debug.rx_overflow_count != 0U)
  {
    alarm |= (1UL << 4);
  }
  if (g_imu_debug.uart4_error_count != 0U)
  {
    alarm |= (1UL << 5);
  }
  if (g_pressure_sensor_state.diag.last_error != 0U)
  {
    alarm |= (1UL << 6);
  }
  if (g_pressure_sensor_state.diag.timeout_count != 0U)
  {
    alarm |= (1UL << 7);
  }
  if (g_pressure_sensor_state.diag.crc_error_count != 0U)
  {
    alarm |= (1UL << 8);
  }
  if (g_pressure_sensor_state.diag.exception_count != 0U)
  {
    alarm |= (1UL << 9);
  }
  if ((g_pump_motor_uart3_diag.last_error != 0U) || (g_pump_motor_uart3_diag.alarm_code != 0U))
  {
    alarm |= (1UL << 10);
  }
  if ((g_pump_driver_status.motor.alarm_code != 0U) || (g_pump_driver_status.motor.last_error != 0U))
  {
    alarm |= (1UL << 11);
  }
  if (g_umotor.bus_diag.last_error != 0U)
  {
    alarm |= (1UL << 12);
  }
  if ((g_umotor.nodes[0].debug.reg_0007_error_code != 0U) ||
      (g_umotor.nodes[0].debug.reg_0194_latest_fault != 0U) ||
      (g_umotor.nodes[1].debug.reg_0007_error_code != 0U) ||
      (g_umotor.nodes[1].debug.reg_0194_latest_fault != 0U))
  {
    alarm |= (1UL << 13);
  }
  if ((g_dbg_view.gnrmc_status_valid == 0U) ||
      (g_dbg_view.gnrmc_latitude_valid == 0U) ||
      (g_dbg_view.gnrmc_longitude_valid == 0U))
  {
    alarm |= (1UL << 14);
  }
  if (limit_bits != 0U)
  {
    alarm |= (1UL << 15);
  }

  g_ethernet_system_snapshot.alive_counter = g_dbg_view.alive_counter;
  g_ethernet_system_snapshot.gnrmc_latitude_e6 =
      EthernetUartDriver_ScaleFloatToI32(g_dbg_view.gnrmc_latitude_deg, 1000000.0f);
  g_ethernet_system_snapshot.gnrmc_longitude_e6 =
      EthernetUartDriver_ScaleFloatToI32(g_dbg_view.gnrmc_longitude_deg, 1000000.0f);
  g_ethernet_system_snapshot.alarm_bits = alarm;

  g_ethernet_system_snapshot.displacement_mm_x10 =
      EthernetUartDriver_ScaleFloatToI16(g_displacement_sensor.displacement_mm, 10.0f);
  g_ethernet_system_snapshot.imu_roll_x10 = EthernetUartDriver_ScaleFloatToI16(g_imu_debug.roll, 10.0f);
  g_ethernet_system_snapshot.imu_pitch_x10 = EthernetUartDriver_ScaleFloatToI16(g_imu_debug.pitch, 10.0f);
  g_ethernet_system_snapshot.imu_yaw_x10 = EthernetUartDriver_ScaleFloatToI16(g_imu_debug.yaw, 10.0f);
  g_ethernet_system_snapshot.imu_height_dm = EthernetUartDriver_ScaleFloatToI16(g_imu_debug.height, 10.0f);
  g_ethernet_system_snapshot.imu_temp_c_x10 =
      EthernetUartDriver_ScaleFloatToI16(g_imu_debug.temperature, 10.0f);
  g_ethernet_system_snapshot.pressure_bar_x100 =
      (uint16_t)EthernetUartDriver_ClampI16((int32_t)(g_pressure_sensor_state.readout.pressure_bar * 100.0f));
  g_ethernet_system_snapshot.pressure_temp_c_x10 =
      EthernetUartDriver_ScaleFloatToI16(g_pressure_sensor_state.readout.temperature_c, 10.0f);
  g_ethernet_system_snapshot.pump_feedback_rpm =
      EthernetUartDriver_ClampI16(g_pump_driver_status.motor.feedback_speed_rpm);
  g_ethernet_system_snapshot.umotor0_feedback_rpm =
      EthernetUartDriver_ClampI16(g_umotor.nodes[0].feedback_speed_rpm);
  g_ethernet_system_snapshot.umotor1_feedback_rpm =
      EthernetUartDriver_ClampI16(g_umotor.nodes[1].feedback_speed_rpm);
  g_ethernet_system_snapshot.servo_pulse_us = g_servo_debug.current_pulse_us;

  g_ethernet_system_snapshot.limit_switch_bits = limit_bits;
  g_ethernet_system_snapshot.displacement_valid = (uint8_t)(g_displacement_sensor.valid != 0U);
  g_ethernet_system_snapshot.gnrmc_valid =
      (uint8_t)((g_dbg_view.gnrmc_status_valid != 0U) &&
                (g_dbg_view.gnrmc_latitude_valid != 0U) &&
                (g_dbg_view.gnrmc_longitude_valid != 0U));
  g_ethernet_system_snapshot.pressure_valid = (uint8_t)(g_pressure_sensor_state.readout.pressure_valid != 0U);
  g_ethernet_system_snapshot.servo_mode = (uint8_t)(g_servo_debug.mode & 0xFFU);
  g_ethernet_system_snapshot.pump_speed_triggered = g_io_pump_debug.speed_triggered;
  g_ethernet_system_snapshot.pump_direction_reversed = g_io_pump_debug.direction_reversed;

  /* 本函数只在 Ethernet task 运行，因此可以整体复制 EKF 快照，避免跨帧撕裂。
     角度 rad→deg×10；位置 m→cm；深度 m→mm。 */
  {
    const UgEkfOutput *eo;
    (void)EkfBridge_CopyState(&ekf_state);
    eo = &ekf_state.out;
    g_ethernet_system_snapshot.ekf_pN_cm =
        EthernetUartDriver_ScaleFloatToI32(eo->pN, 100.0f);
    g_ethernet_system_snapshot.ekf_pE_cm =
        EthernetUartDriver_ScaleFloatToI32(eo->pE, 100.0f);
    g_ethernet_system_snapshot.ekf_depth_mm =
        EthernetUartDriver_ScaleFloatToI32(eo->depth_m, 1000.0f);
    g_ethernet_system_snapshot.ekf_roll_x10 =
        EthernetUartDriver_ScaleFloatToI16(eo->roll * 57.2957795f, 10.0f);
    g_ethernet_system_snapshot.ekf_pitch_x10 =
        EthernetUartDriver_ScaleFloatToI16(eo->pitch * 57.2957795f, 10.0f);
    g_ethernet_system_snapshot.ekf_yaw_x10 =
        EthernetUartDriver_ScaleFloatToI16(eo->yaw * 57.2957795f, 10.0f);
    g_ethernet_system_snapshot.ekf_status = (uint8_t)eo->status;
    g_ethernet_system_snapshot.ekf_valid = (uint8_t)eo->valid_for_control;
  }
}

static void EthernetUartDriver_RearmReceive(void)
{
  HAL_StatusTypeDef status;

  if (s_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  g_ethernet_uart_diag.last_hal_status = (uint32_t)status;
  if (status == HAL_OK)
  {
    s_rx_rearm_pending = 0U;
    g_ethernet_uart_diag.rx_armed = 1U;
    g_ethernet_uart_diag.rx_rearm_ok_count++;
    return;
  }

  if (status == HAL_BUSY)
  {
    g_ethernet_uart_diag.rx_rearm_busy_count++;
    s_rx_rearm_pending = 1U;
    return;
  }

  s_rx_rearm_pending = 1U;
  g_ethernet_uart_diag.rx_armed = 0U;
  g_ethernet_uart_diag.rx_rearm_fail_count++;
  g_ethernet_uart_diag.last_error = 1U;
}

//这里记录数据
static void EthernetUartDriver_RingPush(uint8_t byte)
{
  uint16_t next = (uint16_t)(s_rx_head + 1U);
  if (next >= ETHERNET_UART_RX_RING_SIZE)
  {
    next = 0U;
  }

  if (next == s_rx_tail)
  {
    g_ethernet_uart_diag.rx_ring_overflow_count++;
    return;
  }

  s_eth_rx_ring[s_rx_head] = byte;
  s_rx_head = next;
  g_ethernet_uart_diag.rx_byte_count++;
  g_ethernet_uart_diag.ring_available_bytes =
      (s_rx_head >= s_rx_tail)
          ? (uint32_t)(s_rx_head - s_rx_tail)
          : (uint32_t)(ETHERNET_UART_RX_RING_SIZE - s_rx_tail + s_rx_head);
}

//这里读出数据
static bool EthernetUartDriver_RingPop(uint8_t *byte)
{
  if (s_rx_head == s_rx_tail)
  {
    return false;
  }

  *byte = s_eth_rx_ring[s_rx_tail];
  s_rx_tail++;
  if (s_rx_tail >= ETHERNET_UART_RX_RING_SIZE)
  {
    s_rx_tail = 0U;
  }

  g_ethernet_uart_diag.ring_available_bytes =
      (s_rx_head >= s_rx_tail)
          ? (uint32_t)(s_rx_head - s_rx_tail)
          : (uint32_t)(ETHERNET_UART_RX_RING_SIZE - s_rx_tail + s_rx_head);
  return true;
}

static bool EthernetUartDriver_ParseControlPayload(
    const uint8_t *raw, uint16_t length, EthernetUartReadoutPack *parsed)
{
  if (raw == NULL || parsed == NULL || length != UGCP_CONTROL_PAYLOAD_SIZE)
  {
    return false;
  }

  parsed->pitchmotor_enable = raw[0];
  parsed->pitchmotor_speed_rpm = EthernetUartDriver_ReadLeI32(&raw[1]);
  parsed->rollmotor_enable = raw[5];
  parsed->rollmotor_speed_rpm = EthernetUartDriver_ReadLeI32(&raw[6]);
  parsed->thrustmotor_direction = raw[10];
  parsed->thrustmotor_direction2 = raw[11];
  parsed->thrustmotor1se_speed_rpm = raw[12];
  parsed->thrustmotor2se_speed_rpm = raw[13];
  parsed->backup_for_thrust = raw[14];
  parsed->pump_direction = raw[15];
  parsed->pump_on_off = raw[16];
  parsed->ask_for_status = raw[17];
  parsed->eth_control_flag = raw[18];

  /* CRC 能发现传输错误；这里再做语义约束，未知枚举一律 fail-closed。 */
  if (parsed->pitchmotor_enable > 1U ||
      parsed->rollmotor_enable > 1U ||
      parsed->thrustmotor_direction > 1U ||
      parsed->thrustmotor_direction2 > 1U ||
      parsed->thrustmotor1se_speed_rpm > 100U ||
      parsed->thrustmotor2se_speed_rpm > 100U ||
      parsed->backup_for_thrust > 1U ||
      parsed->pump_direction > 1U ||
      parsed->pump_on_off > 1U ||
      parsed->ask_for_status > 1U ||
      parsed->eth_control_flag > 1U)
  {
    return false;
  }

  return true;
}
//CRC校验通过后解析控制包
static void EthernetUartDriver_HandleControl(
    const uint8_t *payload, uint16_t payload_length, uint8_t policy)
{
  EthernetUartReadoutPack parsed;
  uint32_t now;

  if (!EthernetUartDriver_ParseControlPayload(payload, payload_length, &parsed))
  {
    g_ethernet_uart_diag.control_reject_count++;
    g_ethernet_uart_diag.control_payload_error_count++;
    if (payload_length != UGCP_CONTROL_PAYLOAD_SIZE)
    {
      g_ethernet_uart_diag.frame_length_error_count++;
    }
    return;
  }

  now = HAL_GetTick();
  taskENTER_CRITICAL();
  g_ethernet_uart_readout_pack = parsed;
  g_ethernet_control_last_rx_tick = now;
  g_ethernet_rx_message_flag = parsed.eth_control_flag;
  g_ethernet_control_policy =
      (parsed.eth_control_flag != 0U) ? policy : ETHERNET_UART_CONTROL_POLICY_NONE;
  taskEXIT_CRITICAL();
  g_ethernet_uart_diag.control_packet_count++;

  if (parsed.ask_for_status == 1U)
  {
    /* 只置请求，实际构造/异步发送由 Ethernet task 的发送调度完成。 */
    s_snapshot_pending = 1U;
  }
}

static void EthernetUartDriver_HandleParamBlock(
    const uint8_t *payload, uint16_t payload_length)
{
  uint8_t group;
  uint8_t transaction;
  uint8_t block;

  if (payload == NULL || payload_length != UGCP_PARAM_BLOCK_PAYLOAD_SIZE)
  {
    g_ethernet_uart_diag.frame_length_error_count++;
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  group = payload[0];
  transaction = payload[1];
  block = payload[2];
  if (group != UGCP_PARAM_GROUP_MAG_CAL || block >= UGCP_PARAM_BLOCK_COUNT)
  {
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  /* block0 明确开启/重启一次事务；TCP 保序，后续块必须带同一 group/transaction。 */
  if (block == 0U)
  {
    (void)memset(&s_param_stage, 0, sizeof(s_param_stage));
    s_param_stage.group = group;
    s_param_stage.transaction = transaction;
  }
  else if (s_param_stage.group != group ||
           s_param_stage.transaction != transaction)
  {
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  (void)memcpy(&s_param_stage.table[(uint32_t)block * UGCP_PARAM_BLOCK_DATA_SIZE],
               &payload[3],
               UGCP_PARAM_BLOCK_DATA_SIZE);
  s_param_stage.received_mask |= (uint8_t)(1U << block);
  g_ethernet_uart_diag.parameter_block_count++;
  g_ethernet_uart_diag.parameter_pending_mask = s_param_stage.received_mask;
  g_ethernet_uart_diag.parameter_last_group = group;
  g_ethernet_uart_diag.parameter_last_transaction = transaction;
}

static void EthernetUartDriver_HandleParamCommit(
    const uint8_t *payload, uint16_t payload_length)
{
  uint32_t expected_crc;
  const uint8_t all_blocks_mask =
      (uint8_t)((1U << UGCP_PARAM_BLOCK_COUNT) - 1U);

  if (payload == NULL || payload_length != UGCP_PARAM_COMMIT_PAYLOAD_SIZE)
  {
    g_ethernet_uart_diag.frame_length_error_count++;
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  expected_crc = EthernetUartDriver_ReadLeU32(&payload[3]);
  if (payload[0] != UGCP_PARAM_GROUP_MAG_CAL ||
      payload[2] != UGCP_PARAM_BLOCK_COUNT ||
      s_param_stage.group != payload[0] ||
      s_param_stage.transaction != payload[1] ||
      s_param_stage.received_mask != all_blocks_mask)
  {
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  if (EkfBridge_SubmitMagCalibration(s_param_stage.table, expected_crc) == 0U)
  {
    g_ethernet_uart_diag.parameter_reject_count++;
    return;
  }

  g_ethernet_uart_diag.parameter_commit_count++;
  g_ethernet_uart_diag.parameter_last_crc32 = expected_crc;
  s_param_stage.received_mask = 0U;
  g_ethernet_uart_diag.parameter_pending_mask = 0U;
}

static void EthernetUartDriver_DispatchFrame(
    uint8_t message_type, const uint8_t *payload, uint16_t payload_length)
{
  switch (message_type)
  {
    case ETHERNET_UART_MSG_CONTROL19_STREAM:
      EthernetUartDriver_HandleControl(
          payload, payload_length, ETHERNET_UART_CONTROL_POLICY_STREAM);
      break;

    case ETHERNET_UART_MSG_CONTROL19_LATCHED:
      EthernetUartDriver_HandleControl(
          payload, payload_length, ETHERNET_UART_CONTROL_POLICY_LATCHED);
      break;

    case ETHERNET_UART_MSG_TELEM_CONTROL:
      if (payload_length != 1U ||
          (payload[0] & (uint8_t)(~ETHERNET_UART_STREAM_ALL)) != 0U)
      {
        g_ethernet_uart_diag.frame_length_error_count++;
        g_ethernet_uart_diag.frame_type_error_count++;
      }
      else
      {
        s_ekf_telem_enabled =
            ((payload[0] & ETHERNET_UART_STREAM_EKF) != 0U) ? 1U : 0U;
        s_mag_telem_enabled =
            ((payload[0] & ETHERNET_UART_STREAM_MAG) != 0U) ? 1U : 0U;
        g_ethernet_uart_diag.telemetry_control_count++;
      }
      break;

    case ETHERNET_UART_MSG_SNAPSHOT_REQ:
      if (payload_length != 0U)
      {
        g_ethernet_uart_diag.frame_length_error_count++;
      }
      else
      {
        s_snapshot_pending = 1U;
        g_ethernet_uart_diag.snapshot_request_count++;
      }
      break;

    case ETHERNET_UART_MSG_PARAM_BLOCK:
      EthernetUartDriver_HandleParamBlock(payload, payload_length);
      break;

    case ETHERNET_UART_MSG_PARAM_COMMIT:
      EthernetUartDriver_HandleParamCommit(payload, payload_length);
      break;

    default:
      g_ethernet_uart_diag.frame_type_error_count++;
      break;
  }
}

static void EthernetUartDriver_ProcessLine(const char *line)
{
  size_t i;

  if (line == NULL || line[0] == '\0')
  {
    return;
  }

  size_t len = strlen(line);
  if (len >= ETHERNET_UART_RX_LINE_MAX)
  {
    g_ethernet_uart_diag.rx_line_overflow_count++;
    return;
  }

  for (i = 0U; i <= len; i++)
  {
    g_ethernet_uart_readout.last_rx_line[i] = line[i];
  }
  g_ethernet_uart_readout.last_rx_byte = (uint32_t)line[len - 1U];
  g_ethernet_uart_diag.rx_line_count++;
  g_ethernet_uart_diag.last_rx_tick = HAL_GetTick();

  if (strstr(line, "CONNECT") != NULL || strstr(line, "OK") != NULL)
  {
    g_ethernet_uart_diag.last_error = 0U;
  }
  else if (strstr(line, "ERROR") != NULL || strstr(line, "FAIL") != NULL)
  {
    g_ethernet_uart_diag.last_error = 2U;
  }
}

static void EthernetUartDriver_DiscardParserFront(uint16_t count)
{
  if (count >= s_ugcp_parse_length)
  {
    s_ugcp_parse_length = 0U;
    return;
  }

  (void)memmove(s_ugcp_parse_buffer,
                &s_ugcp_parse_buffer[count],
                (size_t)(s_ugcp_parse_length - count));
  s_ugcp_parse_length = (uint16_t)(s_ugcp_parse_length - count);
}

static int32_t EthernetUartDriver_FindMagic(uint16_t start)
{
  uint16_t i;
  if (s_ugcp_parse_length < UGCP_MAGIC_SIZE || start >= s_ugcp_parse_length)
  {
    return -1;
  }

  for (i = start; i + UGCP_MAGIC_SIZE <= s_ugcp_parse_length; i++)
  {
    if (memcmp(&s_ugcp_parse_buffer[i],
               s_ugcp_magic,
               UGCP_MAGIC_SIZE) == 0)
    {
      return (int32_t)i;
    }
  }
  return -1;
}

/* 没找到完整 magic 时，保留末尾可能构成下一帧开头的 U / UG / UGC。 */
static uint16_t EthernetUartDriver_MagicSuffixLength(void)
{
  uint16_t keep =
      (s_ugcp_parse_length < (UGCP_MAGIC_SIZE - 1U))
          ? s_ugcp_parse_length
          : (UGCP_MAGIC_SIZE - 1U);

  while (keep > 0U)
  {
    if (memcmp(&s_ugcp_parse_buffer[s_ugcp_parse_length - keep],
               s_ugcp_magic,
               keep) == 0)
    {
      break;
    }
    keep--;
  }
  return keep;
}

// 逐字节喂入、滑窗找帧。CRC/长度失败只丢一个候选起始字节再搜索，
//因而插字节、丢字节或前置 AT 文本后都能在下一帧自动恢复同步
static void EthernetUartDriver_FeedFrameParser(uint8_t byte)
{
  if (s_ugcp_parse_length >= UGCP_MAX_FRAME_SIZE)
  {
    EthernetUartDriver_DiscardParserFront(1U);
    g_ethernet_uart_diag.frame_resync_discard_count++;
  }
  s_ugcp_parse_buffer[s_ugcp_parse_length++] = byte;
  s_ugcp_last_byte_tick = HAL_GetTick();

  EthernetUartDriver_ProcessFrameParser();
}

//只处理当前解析窗口，不接触 UART/ring；任务上下文可在收字节后或超时重同步后调用
static void EthernetUartDriver_ProcessFrameParser(void)
{
  for (;;)
  {
    int32_t magic_pos;
    uint16_t payload_length;
    uint16_t total_length;
    uint16_t sequence;
    uint32_t expected_crc;
    uint32_t actual_crc;

    if (s_ugcp_parse_length < UGCP_MAGIC_SIZE)
    {
      return;
    }

    magic_pos = EthernetUartDriver_FindMagic(0U);
    if (magic_pos < 0)
    {
      const uint16_t keep = EthernetUartDriver_MagicSuffixLength();
      const uint16_t discard = (uint16_t)(s_ugcp_parse_length - keep);
      if (discard != 0U)
      {
        EthernetUartDriver_DiscardParserFront(discard);
        g_ethernet_uart_diag.frame_resync_discard_count += discard;
      }
      return;
    }
    if (magic_pos > 0)
    {
      EthernetUartDriver_DiscardParserFront((uint16_t)magic_pos);
      g_ethernet_uart_diag.frame_resync_discard_count += (uint32_t)magic_pos;
    }

    if (s_ugcp_parse_length < UGCP_FIXED_HEADER_SIZE)
    {
      return;
    }

    if (s_ugcp_parse_buffer[4] != ETHERNET_UART_FRAME_VERSION)
    {
      g_ethernet_uart_diag.frame_version_error_count++;
      g_ethernet_uart_diag.frame_resync_discard_count++;
      EthernetUartDriver_DiscardParserFront(1U);
      continue;
    }

    payload_length =
        (uint16_t)((uint16_t)s_ugcp_parse_buffer[8] |
                   ((uint16_t)s_ugcp_parse_buffer[9] << 8U));
    if (payload_length > ETHERNET_UART_FRAME_MAX_PAYLOAD)
    {
      g_ethernet_uart_diag.frame_length_error_count++;
      g_ethernet_uart_diag.frame_resync_discard_count++;
      EthernetUartDriver_DiscardParserFront(1U);
      continue;
    }

    total_length =
        (uint16_t)(UGCP_FIXED_HEADER_SIZE + payload_length + UGCP_CRC_SIZE);
    if (s_ugcp_parse_length < total_length)
    {
      return;
    }

    expected_crc =
        EthernetUartDriver_ReadLeU32(
            &s_ugcp_parse_buffer[UGCP_FIXED_HEADER_SIZE + payload_length]);
    actual_crc =
        EthernetUartDriver_Crc32(
            &s_ugcp_parse_buffer[UGCP_MAGIC_SIZE],
            (uint16_t)((UGCP_FIXED_HEADER_SIZE - UGCP_MAGIC_SIZE) +
                       payload_length));
    if (actual_crc != expected_crc)
    {
      g_ethernet_uart_diag.frame_crc_error_count++;
      g_ethernet_uart_diag.frame_resync_discard_count++;
      EthernetUartDriver_DiscardParserFront(1U);
      continue;
    }

    sequence =
        (uint16_t)((uint16_t)s_ugcp_parse_buffer[6] |
                   ((uint16_t)s_ugcp_parse_buffer[7] << 8U));
    if (s_ugcp_have_sequence != 0U &&
        sequence != (uint16_t)(g_ethernet_uart_diag.frame_last_seq + 1U))
    {
      g_ethernet_uart_diag.frame_seq_gap_count++;
    }
    s_ugcp_have_sequence = 1U;
    g_ethernet_uart_diag.frame_last_seq = sequence;
    g_ethernet_uart_diag.frame_ok_count++;

    EthernetUartDriver_DispatchFrame(
        s_ugcp_parse_buffer[5],
        &s_ugcp_parse_buffer[UGCP_FIXED_HEADER_SIZE],
        payload_length);
    EthernetUartDriver_DiscardParserFront(total_length);
  }
}

static void EthernetUartDriver_ProcessReceivedData(void)
{
  uint8_t byte;
  static char line[ETHERNET_UART_RX_LINE_MAX];
  static size_t line_len = 0U;

  while (EthernetUartDriver_RingPop(&byte))
  {
    EthernetUartDriver_FeedFrameParser(byte);

    /* AT 配置回显仍按 ASCII 行保留给诊断；二进制/控制字节会直接打断文本行，
     * 不会再与 UGCP 争抢 ring。 */
    if (byte == '\r' || byte == '\n')
    {
      if (line_len != 0U)
      {
        line[line_len] = '\0';
        EthernetUartDriver_ProcessLine(line);
        line_len = 0U;
      }
      continue;
    }

    if (byte >= 0x20U && byte <= 0x7EU &&
        line_len + 1U < sizeof(line))
    {
      line[line_len++] = (char)byte;
    }
    else if (byte < 0x20U || byte > 0x7EU)
    {
      line_len = 0U;
    }
    else
    {
      g_ethernet_uart_diag.rx_line_overflow_count++;
      line_len = 0U;
    }
  }

  /* 若伪帧头声明了一个尚未攒够的长度，它可能遮住后面已经完整到达的帧。
   * 超时后优先跳到下一个完整 magic 并立即再解析；没有下一个 magic 才清空。 */
  if (s_ugcp_parse_length != 0U &&
      (HAL_GetTick() - s_ugcp_last_byte_tick) >=
          ETHERNET_UART_FRAME_PARTIAL_TIMEOUT_MS)
  {
    const int32_t next_magic = EthernetUartDriver_FindMagic(1U);
    const uint16_t discard =
        (next_magic > 0) ? (uint16_t)next_magic : s_ugcp_parse_length;

    EthernetUartDriver_DiscardParserFront(discard);
    g_ethernet_uart_diag.frame_resync_discard_count += discard;
    g_ethernet_uart_diag.frame_partial_timeout_count++;
    EthernetUartDriver_ProcessFrameParser();
  }
}

static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length)
{
  /* 上限放宽到 EKF 遥测行长度；请求应答路径仍按 ≤160 分块，互不影响 */
  if (s_uart == NULL || data == NULL || length == 0U || length > ETHERNET_UART_EKF_LINE_MAX)
  {
    return HAL_ERROR;
  }

  if (s_tx_busy != 0U)
  {
    g_ethernet_uart_diag.tx_busy_count++;
    return HAL_BUSY;
  }

  HAL_StatusTypeDef status = HAL_UART_Transmit_IT(s_uart, data, length);
  if (status == HAL_OK)
  {
    s_tx_busy = 1U;
    g_ethernet_uart_diag.tx_count++;
    g_ethernet_uart_diag.tx_in_flight = 1U;
  }
  else
  {
    g_ethernet_uart_diag.tx_fail_count++;
    g_ethernet_uart_diag.last_hal_status = (uint32_t)status;
  }

  return status;
}

static uint32_t EthernetUartDriver_ReadLeU32(const uint8_t *bytes)
{
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8U) |
         ((uint32_t)bytes[2] << 16U) |
         ((uint32_t)bytes[3] << 24U);
}

static int32_t EthernetUartDriver_ReadLeI32(const uint8_t *bytes)
{
  const uint32_t raw = EthernetUartDriver_ReadLeU32(bytes);
  int32_t value;

  /* 原样保留二进制补码位型，避免依赖超出 INT32_MAX 时的整数强转实现。 */
  (void)memcpy(&value, &raw, sizeof(value));
  return value;
}

/* CRC-32/ISO-HDLC：poly=0xEDB88320, init/xorout=0xFFFFFFFF，
 * 与 PC 端 zlib.crc32 完全一致。 */
static uint32_t EthernetUartDriver_Crc32(const uint8_t *data, uint16_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint16_t i;

  for (i = 0U; i < length; i++)
  {
    uint8_t bit;
    crc ^= (uint32_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      const uint32_t lsb_mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & lsb_mask);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

void EthernetUartDriver_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    g_ethernet_uart_diag.last_error = 1U;
    return;
  }

  s_uart = huart;

  (void)memset((void *)&g_ethernet_uart_state, 0, sizeof(g_ethernet_uart_state));
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_byte = 0U;
  s_tx_busy = 0U;
  s_rx_rearm_pending = 0U;
  s_last_command_tick = 0U;
  s_ugcp_parse_length = 0U;
  s_ugcp_last_byte_tick = 0U;
  s_ugcp_have_sequence = 0U;
  s_snapshot_pending = 0U;
  s_ekf_telem_enabled = 1U;
  s_mag_telem_enabled = 0U;
  (void)memset(s_ugcp_parse_buffer, 0, sizeof(s_ugcp_parse_buffer));
  (void)memset(&s_param_stage, 0, sizeof(s_param_stage));
  (void)memset((void *)&g_ethernet_uart_readout_pack,
               0,
               sizeof(g_ethernet_uart_readout_pack));
  g_ethernet_rx_message_flag = 0U;
  g_ethernet_control_last_rx_tick = 0U;
  g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_NONE;

  EthernetUartDriver_RearmReceive();
  if (g_ethernet_uart_diag.last_error == 0U)
  {
    g_ethernet_uart_diag.initialized = 1U;
  }
}

void EthernetUartDriver_RunTask(void *argument)
{
  (void)argument;
  const uint32_t setup_period = ETHERNET_UART_HEARTBEAT_PERIOD_MS;
  const char *setup_commands[] = {
      "AT\r\n",
      "AT+IPR=115200\r\n",
      "AT+NETP=1,0,12345,0\r\n",
      "AT+ENTM\r\n"
  };
  const size_t setup_count = sizeof(setup_commands) / sizeof(setup_commands[0]);
  static size_t setup_index = 0U;
  static uint32_t s_last_ekf_tx_tick = 0U; /* EKF 遥测上次推送 tick */
  static uint32_t s_last_mag_tx_tick = 0U; /* MAG 遥测有自己的周期，不依赖 EKF */

  for (;;)
  {
    uint32_t now;
    g_ethernet_uart_diag.task_loop_count++;

    EthernetUartDriver_ProcessReceivedData();

    /* ISR 中重挂若短暂遇到 HAL_BUSY，等 HAL 状态回到 READY 后由任务重试，
     * 避免一次竞态让字节接收永久停止。 */
    if (s_rx_rearm_pending != 0U &&
        s_uart != NULL &&
        s_uart->RxState == HAL_UART_STATE_READY)
    {
      EthernetUartDriver_RearmReceive();
    }

    /* 必须在处理 RX 之后取时刻：HandleControl() 也在处理过程中取 tick。
     * 若反过来，跨 1ms 时 now-last_rx 会发生 uint32 下溢并误触发超时。 */
    now = HAL_GetTick();
    g_ethernet_uart_diag.control_age_ms =
        (g_ethernet_control_last_rx_tick != 0U)
            ? (now - g_ethernet_control_last_rx_tick)
            : 0U;

    /* 仅 STREAM（auto/Xbox）是租约控制；键盘 LATCHED 命令保持到下一帧。
     * STREAM 超时后退回 flag=0 的现有 failsafe/SBUS 分支。 */
    if (g_ethernet_control_policy == ETHERNET_UART_CONTROL_POLICY_STREAM &&
        g_ethernet_rx_message_flag != 0U &&
        (now - g_ethernet_control_last_rx_tick) >
            ETHERNET_UART_CONTROL_TIMEOUT_MS)
    {
      taskENTER_CRITICAL();
      g_ethernet_rx_message_flag = 0U;
      g_ethernet_uart_readout_pack.eth_control_flag = 0U;
      g_ethernet_control_policy = ETHERNET_UART_CONTROL_POLICY_NONE;
      taskEXIT_CRITICAL();
      g_ethernet_uart_diag.control_timeout_count++;
    }

    if (setup_index < setup_count)
    {
      if ((now - s_last_command_tick) >= setup_period)
      {
        EthernetUartDriver_SendText(setup_commands[setup_index]);
        s_last_command_tick = now;
        setup_index++;
      }
    }
    else
    {
      /* AT 配置完成进入透传。TELEM_CONTROL 的 bit0/bit1 分别独立控制
       * $UGEKF/$UGMAG；默认只发 EKF，磁标定脚本才请求 MAG。 */
      if (s_snapshot_pending != 0U && s_tx_busy == 0U)
      {
        EthernetUartDriver_SendSystemSnapshot();
        if (s_tx_busy != 0U)
        {
          s_snapshot_pending = 0U;
        }
      }
      else if (s_tx_busy == 0U &&
          (s_ekf_telem_enabled != 0U) &&
          ((now - s_last_ekf_tx_tick) >= ETHERNET_UART_EKF_PERIOD_MS))
      {
        EthernetUartDriver_SendEkfTelemetry();
        s_last_ekf_tx_tick = now;
      }
      else if (s_tx_busy == 0U &&
          (s_mag_telem_enabled != 0U) &&
          ((now - s_last_mag_tx_tick) >= ETHERNET_UART_MAG_PERIOD_MS))
      {
        /* 只有成功占用 TX 才推进周期；HAL_BUSY/失败时下一拍重试。 */
        if (EthernetUartDriver_SendMagTelemetry() == HAL_OK)
        {
          s_last_mag_tx_tick = now;
        }
      }
      /* 保留原心跳计数（AT 心跳本就注释掉，仅留 liveness 计数） */
      if ((now - g_ethernet_uart_diag.last_tx_tick) >= ETHERNET_UART_HEARTBEAT_PERIOD_MS)
      {
        g_ethernet_uart_diag.heartbeat_count++;
        g_ethernet_uart_diag.last_tx_tick = now;
        //EthernetUartDriver_SendText("AT\r\n");
      }
    }

    g_ethernet_uart_readout.ring_available_bytes = g_ethernet_uart_diag.ring_available_bytes;
    
    osDelay(ETHERNET_UART_TASK_PERIOD_MS);
  }
}

void EthernetUartDriver_SendText(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  uint16_t length = (uint16_t)strlen(text);
  if (length == 0U || length > ETHERNET_UART_TX_LINE_MAX)
  {
    return;
  }

  (void)EthernetUartDriver_SendRaw((const uint8_t *)text, length);
}

void EthernetUartDriver_SendSystemSnapshot(void)
{
  static const char hex_chars[] = "0123456789ABCDEF";
  uint8_t snapshot_bytes[sizeof(EthernetUartSystemSnapshot)];
  const uint16_t bytes_per_chunk = (uint16_t)((ETHERNET_UART_TX_LINE_MAX - 2U) / 2U);
  uint16_t out_offset = 0U;
  uint16_t i;
  /* HAL_UART_Transmit_IT 返回后仍在读缓冲，必须保持到 TX complete。 */
  static char line[ETHERNET_UART_TX_LINE_MAX + 1U];

  EthernetUartDriver_UpdateSystemSnapshot();

  for (i = 0U; i < (uint16_t)sizeof(EthernetUartSystemSnapshot); i++)
  {
    snapshot_bytes[i] = ((const volatile uint8_t *)&g_ethernet_system_snapshot)[i];
  }

  while (out_offset < (uint16_t)sizeof(EthernetUartSystemSnapshot))
  {
    uint16_t chunk_len = (uint16_t)(sizeof(EthernetUartSystemSnapshot) - out_offset);
    uint16_t chunk_i;
    uint16_t line_i = 0U;

    if (chunk_len > bytes_per_chunk)
    {
      chunk_len = bytes_per_chunk;
    }

    for (chunk_i = 0U; chunk_i < chunk_len; chunk_i++)
    {
      uint8_t b = snapshot_bytes[out_offset + chunk_i];
      line[line_i++] = hex_chars[(b >> 4U) & 0x0FU];
      line[line_i++] = hex_chars[b & 0x0FU];
    }

    line[line_i++] = '\r';
    line[line_i++] = '\n';
    line[line_i] = '\0';
    EthernetUartDriver_SendText(line);

    out_offset = (uint16_t)(out_offset + chunk_len);
  }
}

/*  ===== EKF 状态估计遥测：定点格式化工具 + 单行构帧（$UGEKF,...*HH） =====  */

/* 把无符号整数追加到 p，至少 min_width 位（不足左补 0）。返回写入字符数。
 * 调用方保证缓冲足够（行缓冲已按最坏情况留余量）。 */
static int EthernetUartDriver_AppendUint(char *p, unsigned long val, int min_width)
{
  char tmp[12];
  int n = 0;
  int w = 0;
  int pad;

  do
  {
    tmp[n++] = (char)('0' + (val % 10UL));
    val /= 10UL;
  } while (val != 0UL);

  for (pad = min_width - n; pad > 0; pad--)
  {
    p[w++] = '0';
  }
  while (n > 0)
  {
    p[w++] = tmp[--n];
  }
  return w;
}

/* 把浮点 v 以定点（decimals 位小数）追加到 p。返回写入字符数。
 * 绕开 nano.specs 不支持 %f：手动整数化；非有限值输出 "nan"；
 * 幅值 clamp 到 ±1e6 保证行长有界、不溢出行缓冲。 */
static int EthernetUartDriver_AppendFixed(char *p, float v, int decimals)
{
  int w = 0;
  int neg;
  int k;
  unsigned long scale = 1UL;
  unsigned long scaled;

  if (!isfinite(v))
  {
    p[0] = 'n';
    p[1] = 'a';
    p[2] = 'n';
    return 3;
  }
  if (v > 1.0e6f)
  {
    v = 1.0e6f;
  }
  if (v < -1.0e6f)
  {
    v = -1.0e6f;
  }

  neg = (v < 0.0f);
  if (neg)
  {
    v = -v;
  }
  for (k = 0; k < decimals; k++)
  {
    scale *= 10UL;
  }
  scaled = (unsigned long)((double)v * (double)scale + 0.5);
  if (scaled == 0UL)
  {
    neg = 0; /* 避免 -0.000 */
  }
  if (neg)
  {
    p[w++] = '-';
  }
  w += EthernetUartDriver_AppendUint(p + w, scaled / scale, 1);
  if (decimals > 0)
  {
    p[w++] = '.';
    w += EthernetUartDriver_AppendUint(p + w, scaled % scale, decimals);
  }
  return w;
}

void EthernetUartDriver_SendEkfTelemetry(void)
{
  static const char hex_chars[] = "0123456789ABCDEF";
  static char line[ETHERNET_UART_EKF_LINE_MAX]; /* static：IT 发送期间须保持有效 */
  StateEstimate se;
  const UgEkfOutput *o;
  uint32_t fl;
  uint8_t cks = 0U;
  int i = 0;
  int k;

  if (s_uart == NULL)
  {
    return;
  }
  /* 上一帧还在发：跳过本拍，避免改写正在 IT 发送中的静态缓冲 */
  if (s_tx_busy != 0U)
  {
    g_ethernet_uart_diag.tx_busy_count++;
    return;
  }

  (void)EkfBridge_CopyState(&se); /* 防撕裂整体快照 */
  o = &se.out;

  /* 健康 / 状态位打包：bit0 healthy,1 valid_for_control,2 still,3 finite,4 aligned */
  fl = (o->healthy ? 1UL : 0UL);
  fl |= (o->valid_for_control ? 1UL : 0UL) << 1;
  fl |= (o->still ? 1UL : 0UL) << 2;
  fl |= (o->finite ? 1UL : 0UL) << 3;
  fl |= (o->aligned ? 1UL : 0UL) << 4;

  line[i++] = '$';
  line[i++] = 'U';
  line[i++] = 'G';
  line[i++] = 'E';
  line[i++] = 'K';
  line[i++] = 'F';

  /* 时序 / 状态 */
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)se.update_seq, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)HAL_GetTick(), 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)o->status, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)fl, 1);
  /* 姿态 deg */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->roll * 57.2957795f, 2);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->pitch * 57.2957795f, 2);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->yaw * 57.2957795f, 2);
  /* 位置 m */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->pN, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->pE, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->depth_m, 3);
  /* 速度 m/s */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->vN, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->vE, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->vD, 3);
  /* 零偏（陀螺 / 加计） */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_g[0], 5);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_g[1], 5);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_g[2], 5);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_a[0], 5);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_a[1], 5);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->b_a[2], 5);
  /* 协方差迹（发散指示） */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->P_trace_pos, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->P_trace_vel, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->P_trace_att, 3);
  /* 深度量测一致性 */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->nis_depth, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)o->accept_depth, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)o->reject_depth, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)o->depth_reject_streak, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)o->depth_reset_count, 1);
  /* 交叉验证 / 时序诊断 */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, g_ekf_depth_meas, 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->dt_last, 4);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, g_ekf_imu_hz, 2);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, o->t_last, 3);
  /* 磁标定参数事务 ACK：PC 以完整表 CRC 对账，valid=1 才代表已在 EKF task 生效。 */
  line[i++] = ','; i += EthernetUartDriver_AppendUint(
      line + i, (unsigned long)g_ekf_mag_cal_crc, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(
      line + i, (unsigned long)g_ekf_mag_cal_valid, 1);

  /* NMEA 异或校验：$ 与 * 之间所有字符 */
  for (k = 1; k < i; k++)
  {
    cks ^= (uint8_t)line[k];
  }
  line[i++] = '*';
  line[i++] = hex_chars[(cks >> 4) & 0x0FU];
  line[i++] = hex_chars[cks & 0x0FU];
  line[i++] = '\r';
  line[i++] = '\n';

  if (EthernetUartDriver_SendRaw((const uint8_t *)line, (uint16_t)i) == HAL_OK)
  {
    g_ekf_telem_tx_count++;
  }
}

/* 磁力计原始值遥测（磁标定采集用，$UGMAG,seq,t_ms,mx,my,mz*HH）
 * 目的：硬铁/软铁标定需要在 PC 端记录"模块系原始 mag"(g_ekf_mag_raw, FLU, uT)。
 * 设计：
 *  - 独立语句而非扩展 $UGEKF：contcp_t.py / ekf_logger.py 均按前缀过滤+字段数校验，
 *    新前缀会被它们直接忽略，不破坏队友协议。
 *  - g_ekf_mag_raw 由 IMU 任务 EkfTask_PublishImuRaw() 单写(100Hz)，此处任务上下文散读；
 *    单个 float 读取在 M7 上原子，三分量间最多差一帧(10ms)，对标定(姿态缓变)无影响。
 *  - seq 独立自增，供 PC 端判丢帧。 */
HAL_StatusTypeDef EthernetUartDriver_SendMagTelemetry(void)
{
  static const char hex_chars[] = "0123456789ABCDEF";
  static char line[96];        /* static：IT 发送期间缓冲须保持有效；本行最长 ~70 字节 */
  static uint32_t s_mag_seq = 0U;
  uint8_t cks = 0U;
  int i = 0;
  int k;

  if (s_uart == NULL)
  {
    return HAL_ERROR;
  }
  if (s_tx_busy != 0U)
  {
    g_ethernet_uart_diag.tx_busy_count++;
    return HAL_BUSY;
  }

  line[i++] = '$';
  line[i++] = 'U';
  line[i++] = 'G';
  line[i++] = 'M';
  line[i++] = 'A';
  line[i++] = 'G';

  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)s_mag_seq, 1);
  line[i++] = ','; i += EthernetUartDriver_AppendUint(line + i, (unsigned long)HAL_GetTick(), 1);
  /* 模块系(FLU)原始磁场，uT；3 位小数足够(量化步距 800/32767≈0.024uT) */
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, g_ekf_mag_raw[0], 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, g_ekf_mag_raw[1], 3);
  line[i++] = ','; i += EthernetUartDriver_AppendFixed(line + i, g_ekf_mag_raw[2], 3);

  /* NMEA 异或校验：$ 与 * 之间所有字符 */
  for (k = 1; k < i; k++)
  {
    cks ^= (uint8_t)line[k];
  }
  line[i++] = '*';
  line[i++] = hex_chars[(cks >> 4) & 0x0FU];
  line[i++] = hex_chars[cks & 0x0FU];
  line[i++] = '\r';
  line[i++] = '\n';

  {
    HAL_StatusTypeDef st = EthernetUartDriver_SendRaw((const uint8_t *)line, (uint16_t)i);
    if (st == HAL_OK)
    {
      s_mag_seq++;
    }
    return st;
  }
}

void EthernetUartDriver_ClearRxRing(void)
{
  uint32_t i;

  __disable_irq();
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_byte = 0U;
  for (i = 0U; i < ETHERNET_UART_RX_RING_SIZE; i++)
  {
    s_eth_rx_ring[i] = 0U;
  }
  __enable_irq();

  g_ethernet_uart_diag.ring_available_bytes = 0U;
  g_ethernet_uart_readout.ring_available_bytes = 0U;
}

bool EthernetUartDriver_CopyControlPack(
    EthernetUartReadoutPack *out_pack,
    uint8_t *out_eth_control_flag)
{
  if (out_pack == NULL || out_eth_control_flag == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_pack = g_ethernet_uart_readout_pack;
  *out_eth_control_flag = g_ethernet_rx_message_flag;
  taskEXIT_CRITICAL();
  return true;
}

void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }
  g_ethernet_uart_diag.rx_irq_count++;
  g_ethernet_uart_diag.rx_armed = 0U;
  EthernetUartDriver_RingPush(s_rx_byte);
//ISR 到此为止：不找帧、不算 CRC、不清 ring、不发 UART
  EthernetUartDriver_RearmReceive();
}

void EthernetUartDriver_OnTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }

  s_tx_busy = 0U;
  g_ethernet_uart_diag.tx_complete_count++;
  g_ethernet_uart_diag.tx_in_flight = 0U;
  g_ethernet_uart_diag.last_tx_tick = HAL_GetTick();
}

void EthernetUartDriver_OnError(UART_HandleTypeDef *huart)
{
  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }

  g_ethernet_uart_diag.uart_error_code = huart->ErrorCode;
  g_ethernet_uart_diag.last_error = 5U;

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  /* Drain receiver data register to clear overrun condition */
  volatile uint32_t dummy = huart->Instance->RDR;
  (void)dummy;

  huart->RxState = HAL_UART_STATE_READY;
  EthernetUartDriver_RearmReceive();
}
