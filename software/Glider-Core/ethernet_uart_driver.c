#include "ethernet_uart_driver.h"
#include "cmsis_os.h"
#include "displacement_sensor.h"
#include "motor_driver.h"
#include "pressure_sensor_driver.h"
#include "pump_driver.h"
#include "servo_driver.h"
#include "limit_switch.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

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

typedef struct
{
  uint32_t alive_counter;
  int32_t gnrmc_latitude_e6;
  int32_t gnrmc_longitude_e6;
  uint32_t alarm_bits;
  int16_t displacement_mm_x10;
  int16_t imu_roll_x10;
  int16_t imu_pitch_x10;
  int16_t imu_yaw_x10;
  int16_t imu_height_dm;
  int16_t imu_temp_c_x10;
  uint16_t pressure_bar_x100;
  int16_t pressure_temp_c_x10;
  int16_t pump_feedback_rpm;
  int16_t umotor0_feedback_rpm;
  int16_t umotor1_feedback_rpm;
  uint16_t servo_pulse_us;
  uint8_t limit_switch_bits;
  uint8_t displacement_valid;
  uint8_t gnrmc_valid;
  uint8_t pressure_valid;
  uint8_t servo_mode;
  uint8_t pump_speed_triggered;
  uint8_t pump_direction_reversed;
  uint8_t reserved[2];
} EthernetUartSystemSnapshot;

extern volatile GliderDebugView g_dbg_view;
extern volatile GliderDebugQuickView g_dbg_quick;
extern volatile IMUDebugView g_imu_debug;
extern volatile uint8_t g_limit_switch_states[LIMIT_SWITCH_COUNT];

volatile EthernetUartSystemSnapshot g_ethernet_system_snapshot = {0};

volatile EthernetUartState g_ethernet_uart_state = {0};
volatile EthernetUartReadoutPack g_ethernet_uart_readout_pack = {0};
volatile uint8_t g_ethernet_rx_message_flag = 0U;

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_byte = 0U;
static volatile uint8_t s_eth_rx_ring[ETHERNET_UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint32_t s_tx_busy = 0U;
static uint32_t s_last_command_tick = 0U;

static void EthernetUartDriver_RearmReceive(void);
static void EthernetUartDriver_RingPush(uint8_t byte);
static bool EthernetUartDriver_RingPop(uint8_t *byte);
static void EthernetUartDriver_ProcessReceivedData(void);
static void EthernetUartDriver_ProcessLine(const char *line);
static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length);
static uint32_t EthernetUartDriver_ReadLeU32(const uint8_t *bytes);
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

static void EthernetUartDriver_UpdateSystemSnapshot(void)
{
  uint32_t alarm = 0U;
  uint8_t limit_bits = 0U;
  uint32_t i;

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
  g_ethernet_system_snapshot.reserved[0] = 0U;
  g_ethernet_system_snapshot.reserved[1] = 0U;
}

static void EthernetUartDriver_RearmReceive(void)
{
  HAL_StatusTypeDef status;

  if (s_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  if (status == HAL_OK)
  {
    g_ethernet_uart_diag.rx_rearm_ok_count++;
    return;
  }

  if (status == HAL_BUSY)
  {
    g_ethernet_uart_diag.rx_rearm_busy_count++;
    s_uart->RxState = HAL_UART_STATE_READY;
    s_uart->ErrorCode = HAL_UART_ERROR_NONE;
    status = HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
    if (status == HAL_OK)
    {
      g_ethernet_uart_diag.rx_rearm_ok_count++;
      return;
    }
  }

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

static void EthernetUartDriver_ProcessLine(const char *line)
{
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

  memcpy(g_ethernet_uart_readout.last_rx_line, line, len + 1U);
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

static void EthernetUartDriver_ProcessReceivedData(void)
{
  uint8_t byte;
  static char line[ETHERNET_UART_RX_LINE_MAX];
  static size_t line_len = 0U;

  while (EthernetUartDriver_RingPop(&byte))
  {
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

    if (line_len + 1U < sizeof(line))
    {
      line[line_len++] = (char)byte;
    }
    else
    {
      g_ethernet_uart_diag.rx_line_overflow_count++;
      line_len = 0U;
    }
  }
}

static HAL_StatusTypeDef EthernetUartDriver_SendRaw(const uint8_t *data, uint16_t length)
{
  if (s_uart == NULL || data == NULL || length == 0U || length > ETHERNET_UART_TX_LINE_MAX)
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
  s_last_command_tick = 0U;

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

  for (;;)
  {
    const uint32_t now = HAL_GetTick();
    g_ethernet_uart_diag.task_loop_count++;

    
    EthernetUartDriver_ProcessReceivedData();

    if (setup_index < setup_count)
    {
      if ((now - s_last_command_tick) >= setup_period)
      {
        EthernetUartDriver_SendText(setup_commands[setup_index]);
        s_last_command_tick = now;
        setup_index++;
      }
    }
    else if ((now - g_ethernet_uart_diag.last_tx_tick) >= ETHERNET_UART_HEARTBEAT_PERIOD_MS)
    {
      g_ethernet_uart_diag.heartbeat_count++;
      g_ethernet_uart_diag.last_tx_tick = now;
      //EthernetUartDriver_SendText("AT\r\n");
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
  char line[ETHERNET_UART_TX_LINE_MAX + 1U];

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

bool EthernetUartDriver_ConvertFront19BytesToPack(EthernetUartReadoutPack *out_pack)
{
  uint8_t raw[19];
  uint16_t index;
  uint16_t i;
  uint32_t available;
  EthernetUartReadoutPack parsed;

  __disable_irq();
  available =
      (s_rx_head >= s_rx_tail)
          ? (uint32_t)(s_rx_head - s_rx_tail)
          : (uint32_t)(ETHERNET_UART_RX_RING_SIZE - s_rx_tail + s_rx_head);

  if (available < 19U)
  {
    __enable_irq();
    return false;
  }

  index = s_rx_tail;
  for (i = 0U; i < 19U; i++)
  {
    raw[i] = s_eth_rx_ring[index];
    index++;
    if (index >= ETHERNET_UART_RX_RING_SIZE)
    {
      index = 0U;
    }
  }
  __enable_irq();

  parsed.pitchmotor_enable = raw[0];
  parsed.pitchmotor_speed_rpm = EthernetUartDriver_ReadLeU32(&raw[1]);
  parsed.rollmotor_enable = raw[5];
  parsed.rollmotor_speed_rpm = EthernetUartDriver_ReadLeU32(&raw[6]);
  parsed.thrustmotor_direction = raw[10];
  parsed.thrustmotor_speed_rpm = raw[11];
  parsed.thrustmotor1se_speed_rpm = raw[12];
  parsed.thrustmotor2se_speed_rpm = raw[13];
  parsed.backup_for_thrust = raw[14];
  parsed.pump_direction = raw[15];
  parsed.pump_on_off = raw[16];
  parsed.ask_for_status = raw[17];
  parsed.eth_control_flag = raw[18];

  g_ethernet_uart_readout_pack = parsed;
  if (out_pack != NULL)
  {
    *out_pack = parsed;
  }

  EthernetUartDriver_ClearRxRing();
  return true;
}

void EthernetUartDriver_OnRxComplete(UART_HandleTypeDef *huart)
{
  bool packet_updated;

  if (huart == NULL || s_uart == NULL || huart->Instance != s_uart->Instance)
  {
    return;
  }
  g_ethernet_uart_diag.rx_irq_count++;
  EthernetUartDriver_RingPush(s_rx_byte);
  packet_updated = EthernetUartDriver_ConvertFront19BytesToPack(&g_ethernet_uart_readout_pack);

  if ((packet_updated != false) && (g_ethernet_uart_readout_pack.ask_for_status == 1U))
  {
    EthernetUartDriver_SendSystemSnapshot();
  }
  else if ((packet_updated != false) && (g_ethernet_uart_readout_pack.ask_for_status == 0U))
  {
    EthernetUartDriver_SendText("AT\r\n");
  }

  if (g_ethernet_uart_readout_pack.eth_control_flag == 1U)
  {
    g_ethernet_rx_message_flag = 1U;
  }
  else
  {
    g_ethernet_rx_message_flag = 0U;
  }
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
