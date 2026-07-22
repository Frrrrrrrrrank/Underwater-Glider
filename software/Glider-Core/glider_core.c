#include "glider_core.h"

#include "cmsis_os.h"
#include "ethernet_uart_driver.h"
#include "glider_rdss_codec.h"
#include "imu_core.h"
#include "pressure_sensor_driver.h"
#include "sbus_driver.h"

#include <stdio.h>
#include <string.h>

#define GLIDER_RX_RING_SIZE 512U
#define GLIDER_RX_LINE_SIZE 512U
#define GLIDER_RD_MODE_CMD "AT+TESTRD\r\n"
#define GLIDER_POSITION_TEST_CMD "AT+TESTRN\r\n"
#define GLIDER_TRANSPARENT_MODE_CMD "AT+TESTTC\r\n"
#define GLIDER_QUERY_LOCAL_CARD_CMD "$CCICA,0,00*7B\r\n"
#define GLIDER_QUERY_SIGNAL_CMD "$CCRMO,BSI,2,0*26\r\n"
#define GLIDER_QUERY_TARGET_CARD_CMD "AT+SENDTO?\r\n"
#define GLIDER_SET_CARD_CMD_FMT "AT+SENDTO=%s=\r\n"
#define GLIDER_TX_MODE_SIGNAL_POLL 0U
#define GLIDER_TX_MODE_POSITION_TEST 1U
#define GLIDER_TX_MODE_RECEIVE_ONLY 2U
/* Only edit this macro to change the target platform card ID. */
#define GLIDER_TARGET_CARD_ID "381127"
#define GLIDER_PLATFORM_CARD_ID GLIDER_TARGET_CARD_ID
#define GLIDER_SIGNAL_STABLE_COUNT 2U
#define GLIDER_MODE_SWITCH_COOLDOWN_LINES 3U
#define GLIDER_CARD_QUERY_COOLDOWN_LOOPS 6U
#define GLIDER_RN_REFRESH_INTERVAL_MS 30000U
#define GLIDER_RELOCATE_INTERVAL_MS 180000U
#define GLIDER_SHORTMSG_SEND_INTERVAL_MS 2000U
#define GLIDER_TC_REQUEST_RETRY_MS 3000U
#define GLIDER_TC_ENTER_GRACE_MS 1200U
#define GLIDER_RD_MODE_RETRY_MS 10000U
#define GLIDER_SHORTMSG_ACK_TIMEOUT_MS 8000U

static UART_HandleTypeDef *s_uart = NULL;
static volatile uint8_t s_rx_ring[GLIDER_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint8_t s_rx_byte = 0;

static volatile uint32_t s_rx_overflow_count = 0;
static volatile uint32_t s_rx_line_count = 0;
static volatile uint32_t s_tx_attempt_count = 0;
static volatile uint32_t s_tx_ok_count = 0;
static volatile uint32_t s_tx_fail_count = 0;
static volatile uint32_t s_tx_last_hal_status = 0;
static volatile uint32_t s_tx_last_uart_error = 0;
static volatile uint32_t s_shortmsg_sent_count = 0;
static volatile uint32_t s_bdpwi_signal_available = 0;
static char s_last_tx_text[32] = {0};
static char s_last_shortmsg_payload[96] = {0};

typedef struct
{
  volatile uint32_t signal_good;
  volatile uint32_t tx_mode;
  char local_card_id[24];
  volatile uint32_t local_card_id_known;
  volatile uint32_t local_card_mismatch;
  volatile uint32_t transparent_mode_ready;
  volatile uint32_t tc_mode_requested;
  volatile uint32_t tc_mode_active;
  volatile uint32_t rn_mode_active;
  volatile uint32_t rd_mode_active;
  volatile uint32_t shortmsg_pending;
  volatile uint32_t shortmsg_waiting_ack;
  char current_target_card[24];
  volatile uint32_t current_target_card_known;
  volatile uint32_t current_target_card_match;
  volatile uint32_t target_card_query_attempted;
  char pending_shortmsg_payload[96];
  volatile uint32_t has_cached_fix;
  char cached_shortmsg_payload[96];
  volatile uint32_t signal_good_streak;
  volatile uint32_t signal_bad_streak;
  volatile uint32_t mode_switch_cooldown;
  volatile uint32_t card_query_cooldown;
} GliderLastState;

static GliderLastState s_last = {
    0U,
    GLIDER_TX_MODE_SIGNAL_POLL,
    {0},
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    {0},
    0U,
    0U,
    0U,
    {0},
    0U,
    {0},
    0U,
    0U,
    0U,
    0U};

#define s_signal_good s_last.signal_good
#define s_tx_mode s_last.tx_mode
#define s_local_card_id s_last.local_card_id
#define s_local_card_id_known s_last.local_card_id_known
#define s_local_card_mismatch s_last.local_card_mismatch
#define s_transparent_mode_ready s_last.transparent_mode_ready
#define s_tc_mode_requested s_last.tc_mode_requested
#define s_tc_mode_active s_last.tc_mode_active
#define s_rn_mode_active s_last.rn_mode_active
#define s_rd_mode_active s_last.rd_mode_active
#define s_shortmsg_pending s_last.shortmsg_pending
#define s_shortmsg_waiting_ack s_last.shortmsg_waiting_ack
#define s_current_target_card s_last.current_target_card
#define s_current_target_card_known s_last.current_target_card_known
#define s_current_target_card_match s_last.current_target_card_match
#define s_target_card_query_attempted s_last.target_card_query_attempted
#define s_pending_shortmsg_payload s_last.pending_shortmsg_payload
#define s_has_cached_fix s_last.has_cached_fix
#define s_cached_shortmsg_payload s_last.cached_shortmsg_payload
#define s_signal_good_streak s_last.signal_good_streak
#define s_signal_bad_streak s_last.signal_bad_streak
#define s_mode_switch_cooldown s_last.mode_switch_cooldown
#define s_card_query_cooldown s_last.card_query_cooldown

static volatile uint32_t s_last_valid_fix_tick = 0U;
static volatile uint32_t s_last_rn_request_tick = 0U;
static volatile uint32_t s_last_shortmsg_try_tick = 0U;
static volatile uint32_t s_last_tc_request_tick = 0U;
static volatile uint32_t s_last_rd_request_tick = 0U;
static char s_line_buffer[GLIDER_RX_LINE_SIZE];
static size_t s_line_len = 0;
static bool s_line_overflowed = false;
static GliderRdssDecodedFrame s_decoded_frame;

static const char *s_boot_test_cmds[] = {
  "AT\r\n",
  "ATI\r\n",
  "AT+CSQ\r\n"
};

static void update_local_card_id(const char *parsed);

static void request_rd_mode_soft(uint32_t now)
{
  if (s_rd_mode_active != 0U)
  {
    return;
  }

  if (s_last_rd_request_tick != 0U && (now - s_last_rd_request_tick) < GLIDER_RD_MODE_RETRY_MS)
  {
    return;
  }

  if (GliderCore_SendText(GLIDER_RD_MODE_CMD) == HAL_OK)
  {
    s_last_rd_request_tick = now;
  }
}

static char to_upper_ascii(char ch)
{
  if (ch >= 'a' && ch <= 'z')
  {
    return (char)(ch - ('a' - 'A'));
  }
  return ch;
}

static bool text_contains_token(const char *text, const char *token)
{
  size_t i = 0U;
  size_t j = 0U;

  if (text == NULL || token == NULL || token[0] == '\0')
  {
    return false;
  }

  for (i = 0U; text[i] != '\0'; i++)
  {
    for (j = 0U; token[j] != '\0'; j++)
    {
      if (text[i + j] == '\0')
      {
        return false;
      }
      if (to_upper_ascii(text[i + j]) != to_upper_ascii(token[j]))
      {
        break;
      }
    }
    if (token[j] == '\0')
    {
      return true;
    }
  }

  return false;
}

static bool raw_extract_csv_field(const char *text, uint32_t index, const char **field_start, size_t *field_len)
{
  uint32_t current_index = 0U;
  const char *cursor = NULL;
  const char *field = NULL;

  if (text == NULL || field_start == NULL || field_len == NULL)
  {
    return false;
  }

  cursor = text;
  if (*cursor == '$')
  {
    cursor++;
  }

  while (*cursor != '\0' && *cursor != ',')
  {
    cursor++;
  }

  if (*cursor == ',')
  {
    cursor++;
  }

  field = cursor;
  while (*cursor != '\0' && *cursor != '*')
  {
    if (*cursor == ',')
    {
      if (current_index == index)
      {
        *field_start = field;
        *field_len = (size_t)(cursor - field);
        return true;
      }
      current_index++;
      cursor++;
      field = cursor;
      continue;
    }
    cursor++;
  }

  if (current_index == index)
  {
    *field_start = field;
    *field_len = (size_t)(cursor - field);
    return true;
  }

  return false;
}

static bool try_parse_local_card_from_bdici(const char *line)
{
  const char *field = NULL;
  size_t field_len = 0U;
  char parsed[24] = {0};
  size_t i = 0U;

  if (line == NULL || !text_contains_token(line, "BDICI"))
  {
    return false;
  }

  if (!raw_extract_csv_field(line, 0U, &field, &field_len) || field_len < 6U || field_len > 8U || field_len >= sizeof(parsed))
  {
    return false;
  }

  for (i = 0U; i < field_len; i++)
  {
    if (field[i] < '0' || field[i] > '9')
    {
      return false;
    }
    parsed[i] = field[i];
  }
  parsed[field_len] = '\0';

  if (strcmp(parsed, "2097151") == 0)
  {
    return false;
  }

  update_local_card_id(parsed);
  return true;
}

static bool try_parse_signal_from_bdbsi(const char *line, uint32_t *has_good_signal)
{
  const char *field = NULL;
  size_t field_len = 0U;
  uint32_t index = 0U;
  uint32_t beam_value = 0U;
  uint32_t strong_beam_count = 0U;

  if (has_good_signal == NULL)
  {
    return false;
  }

  *has_good_signal = 0U;
  if (line == NULL || !text_contains_token(line, "BDBSI"))
  {
    return false;
  }

  for (index = 2U; index < 12U; index++)
  {
    if (!raw_extract_csv_field(line, index, &field, &field_len) || field_len != 1U)
    {
      continue;
    }

    if (field[0] < '0' || field[0] > '4')
    {
      continue;
    }

    beam_value = (uint32_t)(field[0] - '0');
    if (beam_value >= 4U)
    {
      strong_beam_count++;
    }
  }

  *has_good_signal = (strong_beam_count > 0U) ? 1U : 0U;
  return true;
}

static void update_signal_state(uint32_t new_signal_good)
{
  if (new_signal_good != 0U)
  {
    s_signal_good_streak++;
    s_signal_bad_streak = 0U;
    if (s_signal_good_streak > GLIDER_SIGNAL_STABLE_COUNT)
    {
      s_signal_good_streak = GLIDER_SIGNAL_STABLE_COUNT;
    }
  }
  else
  {
    s_signal_bad_streak++;
    s_signal_good_streak = 0U;
    if (s_signal_bad_streak > GLIDER_SIGNAL_STABLE_COUNT)
    {
      s_signal_bad_streak = GLIDER_SIGNAL_STABLE_COUNT;
    }
  }

  if (s_signal_good_streak >= GLIDER_SIGNAL_STABLE_COUNT)
  {
    s_signal_good = 1U;
  }
  else if (s_signal_bad_streak >= GLIDER_SIGNAL_STABLE_COUNT)
  {
    s_signal_good = 0U;
  }

  if (s_mode_switch_cooldown == 0U && s_shortmsg_pending == 0U && s_tc_mode_active == 0U && s_tc_mode_requested == 0U)
  {
    if (s_signal_good_streak >= GLIDER_SIGNAL_STABLE_COUNT && s_tx_mode != GLIDER_TX_MODE_POSITION_TEST)
    {
      s_tx_mode = GLIDER_TX_MODE_POSITION_TEST;
      s_mode_switch_cooldown = GLIDER_MODE_SWITCH_COOLDOWN_LINES;
    }
    else if (s_signal_bad_streak >= GLIDER_SIGNAL_STABLE_COUNT && s_tx_mode != GLIDER_TX_MODE_SIGNAL_POLL)
    {
      s_tx_mode = GLIDER_TX_MODE_SIGNAL_POLL;
      s_mode_switch_cooldown = GLIDER_MODE_SWITCH_COOLDOWN_LINES;
    }
  }
}

static bool try_parse_target_card_from_sendto(const char *line)
{
  size_t i = 0U;
  size_t start = 0U;
  size_t len = 0U;
  size_t best_start = 0U;
  size_t best_len = 0U;
  char parsed[24] = {0};

  if (line == NULL || !text_contains_token(line, "SENDTO"))
  {
    return false;
  }

  while (line[i] != '\0')
  {
    if (line[i] >= '0' && line[i] <= '9')
    {
      start = i;
      while (line[i] >= '0' && line[i] <= '9')
      {
        i++;
      }
      len = i - start;
      if (len >= 6U && len <= 8U)
      {
        best_start = start;
        best_len = len;
      }
    }
    else
    {
      i++;
    }
  }

  if (best_len == 0U || best_len >= sizeof(parsed))
  {
    return false;
  }

  for (i = 0U; i < best_len; i++)
  {
    parsed[i] = line[best_start + i];
  }
  parsed[best_len] = '\0';

  strncpy(s_current_target_card, parsed, sizeof(s_current_target_card) - 1U);
  s_current_target_card[sizeof(s_current_target_card) - 1U] = '\0';
  s_current_target_card_known = 1U;
  s_current_target_card_match = (strcmp(parsed, GLIDER_PLATFORM_CARD_ID) == 0) ? 1U : 0U;
  return true;
}

static void update_local_card_id(const char *parsed)
{
  if (parsed == NULL || parsed[0] == '\0')
  {
    return;
  }

  if (s_local_card_id_known == 0U)
  {
    strncpy(s_local_card_id, parsed, sizeof(s_local_card_id) - 1U);
    s_local_card_id[sizeof(s_local_card_id) - 1U] = '\0';
    s_local_card_id_known = 1U;
    s_local_card_mismatch = 0U;
    return;
  }

  if (strncmp(parsed, s_local_card_id, sizeof(s_local_card_id)) == 0)
  {
    s_local_card_mismatch = 0U;
  }
  else
  {
    s_local_card_mismatch = 1U;
  }
}

static bool cache_coordinate_shortmsg(const GliderRdssDecodedFrame *frame)
{
  char payload[96] = {0};
  int written = 0;
  int sign_lat = 0;
  int sign_lon = 0;
  uint32_t abs_lat = 0U;
  uint32_t abs_lon = 0U;

  if (frame == NULL)
  {
    return false;
  }

  sign_lat = (frame->gnrmc.latitude_deg_e7 >= 0) ? 1 : -1;
  sign_lon = (frame->gnrmc.longitude_deg_e7 >= 0) ? 1 : -1;
  abs_lat = (uint32_t)((sign_lat > 0) ? frame->gnrmc.latitude_deg_e7 : -frame->gnrmc.latitude_deg_e7);
  abs_lon = (uint32_t)((sign_lon > 0) ? frame->gnrmc.longitude_deg_e7 : -frame->gnrmc.longitude_deg_e7);

  written = snprintf(
      payload,
      sizeof(payload),
      "LAT=%c%lu.%07lu,LON=%c%lu.%07lu,UTC=%s,DATE=%s\r\n",
      (sign_lat > 0) ? '+' : '-',
      (unsigned long)(abs_lat / 10000000U),
      (unsigned long)(abs_lat % 10000000U),
      (sign_lon > 0) ? '+' : '-',
      (unsigned long)(abs_lon / 10000000U),
      (unsigned long)(abs_lon % 10000000U),
      frame->gnrmc.utc,
      frame->gnrmc.date);

  if (written <= 0 || (size_t)written >= sizeof(payload))
  {
    return false;
  }

  strncpy(s_cached_shortmsg_payload, payload, sizeof(s_cached_shortmsg_payload) - 1U);
  s_cached_shortmsg_payload[sizeof(s_cached_shortmsg_payload) - 1U] = '\0';
  s_has_cached_fix = 1U;
  s_last_valid_fix_tick = HAL_GetTick();
  return true;
}

static bool cached_fix_expired(uint32_t now)
{
  if (s_has_cached_fix == 0U || s_last_valid_fix_tick == 0U)
  {
    return false;
  }

  return ((now - s_last_valid_fix_tick) >= GLIDER_RELOCATE_INTERVAL_MS);
}

static void invalidate_cached_fix_and_request_relocate(void)
{
  s_has_cached_fix = 0U;
  s_cached_shortmsg_payload[0] = '\0';
  s_pending_shortmsg_payload[0] = '\0';
  s_shortmsg_pending = 0U;
  s_transparent_mode_ready = 0U;
  s_tc_mode_active = 0U;
  s_tc_mode_requested = 0U;
  s_last_rn_request_tick = 0U;
}

static void try_queue_cached_shortmsg(void)
{
  uint32_t now = HAL_GetTick();

  if (cached_fix_expired(now))
  {
    if (s_shortmsg_waiting_ack == 0U)
    {
      invalidate_cached_fix_and_request_relocate();
    }
    return;
  }

  if (s_shortmsg_pending != 0U || s_has_cached_fix == 0U)
  {
    return;
  }

  if (s_signal_good == 0U)
  {
    return;
  }

  strncpy(s_pending_shortmsg_payload, s_cached_shortmsg_payload, sizeof(s_pending_shortmsg_payload) - 1U);
  s_pending_shortmsg_payload[sizeof(s_pending_shortmsg_payload) - 1U] = '\0';
  s_shortmsg_pending = 1U;
}

static bool try_send_pending_shortmsg(void)
{
  char cmd[48] = {0};
  int written = 0;

  if (s_shortmsg_pending == 0U)
  {
    return false;
  }

  if (s_signal_good == 0U || s_local_card_id_known == 0U || s_local_card_mismatch != 0U)
  {
    if (s_local_card_id_known == 0U)
    {
      if (s_card_query_cooldown == 0U)
      {
        uint32_t now = HAL_GetTick();
        request_rd_mode_soft(now);
        (void)GliderCore_SendText(GLIDER_QUERY_LOCAL_CARD_CMD);
        s_card_query_cooldown = GLIDER_CARD_QUERY_COOLDOWN_LOOPS;
      }
    }
    return false;
  }

  if (s_tc_mode_active == 0U)
  {
    if (s_tc_mode_requested == 0U)
    {
      if (GliderCore_SendText(GLIDER_TRANSPARENT_MODE_CMD) == HAL_OK)
      {
        s_tc_mode_requested = 1U;
        s_last_tc_request_tick = HAL_GetTick();
      }
    }
    else
    {
      uint32_t now = HAL_GetTick();

      /* Fallback: some modules do not print a stable TC prompt immediately.
       * Keep timed promotion to avoid being stuck forever in requested state. */
      if ((now - s_last_tc_request_tick) >= GLIDER_TC_ENTER_GRACE_MS)
      {
        s_tc_mode_active = 1U;
        s_rn_mode_active = 0U;
        s_rd_mode_active = 0U;
        s_transparent_mode_ready = 0U;
        s_tc_mode_requested = 0U;
        return false;
      }

      if ((now - s_last_tc_request_tick) >= GLIDER_TC_REQUEST_RETRY_MS)
      {
        if (GliderCore_SendText(GLIDER_TRANSPARENT_MODE_CMD) == HAL_OK)
        {
          s_last_tc_request_tick = now;
        }
      }
    }
    return false;
  }

  if (s_transparent_mode_ready == 0U)
  {
    if (s_current_target_card_known == 0U && s_target_card_query_attempted == 0U)
    {
      if (GliderCore_SendText(GLIDER_QUERY_TARGET_CARD_CMD) != HAL_OK)
      {
        return false;
      }
      s_target_card_query_attempted = 1U;
      return false;
    }

    if (s_current_target_card_known == 0U || s_current_target_card_match == 0U)
    {
      written = snprintf(cmd, sizeof(cmd), GLIDER_SET_CARD_CMD_FMT, GLIDER_PLATFORM_CARD_ID);
      if (written <= 0 || (size_t)written >= sizeof(cmd))
      {
        return false;
      }
      if (GliderCore_SendText(cmd) != HAL_OK)
      {
        return false;
      }
      strncpy(s_current_target_card, GLIDER_PLATFORM_CARD_ID, sizeof(s_current_target_card) - 1U);
      s_current_target_card[sizeof(s_current_target_card) - 1U] = '\0';
      s_current_target_card_known = 1U;
      s_current_target_card_match = 1U;
      s_target_card_query_attempted = 0U;
      s_transparent_mode_ready = 1U;
      return false;
    }

    s_target_card_query_attempted = 0U;
    s_transparent_mode_ready = 1U;
    return false;
  }

  if (s_shortmsg_waiting_ack != 0U)
  {
    uint32_t now = HAL_GetTick();
    if ((now - s_last_shortmsg_try_tick) >= GLIDER_SHORTMSG_ACK_TIMEOUT_MS)
    {
      s_shortmsg_waiting_ack = 0U;
      s_transparent_mode_ready = 0U;
      s_tc_mode_active = 0U;
      s_tc_mode_requested = 0U;
      s_target_card_query_attempted = 0U;
    }
    return false;
  }

  if (GliderCore_SendText(s_pending_shortmsg_payload) != HAL_OK)
  {
    return false;
  }

  s_last_shortmsg_try_tick = HAL_GetTick();
  s_shortmsg_waiting_ack = 1U;
  return true;
}

static bool is_valid_gnrmc_fix(const GliderRdssDecodedFrame *frame)
{
  int32_t lat = 0;
  int32_t lon = 0;

  if (frame == NULL)
  {
    return false;
  }

  if (!frame->is_gnrmc || !frame->gnrmc.status_valid || !frame->gnrmc.latitude_valid || !frame->gnrmc.longitude_valid)
  {
    return false;
  }

  lat = frame->gnrmc.latitude_deg_e7;
  lon = frame->gnrmc.longitude_deg_e7;
  if (lat < -900000000 || lat > 900000000 || lon < -1800000000 || lon > 1800000000)
  {
    return false;
  }

  return !(lat == 0 && lon == 0);
}

static bool ring_pop(uint8_t *out)
{
  uint16_t tail = 0;

  if (out == NULL)
  {
    return false;
  }

  if (s_rx_tail == s_rx_head)
  {
    return false;
  }

  tail = s_rx_tail;
  *out = s_rx_ring[tail];
  s_rx_tail = (uint16_t)((tail + 1U) % GLIDER_RX_RING_SIZE);
  return true;
}

static void ring_push(const uint8_t data)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % GLIDER_RX_RING_SIZE);

  if (next == s_rx_tail)
  {
    s_rx_overflow_count++;
    return;
  }

  s_rx_ring[s_rx_head] = data;
  s_rx_head = next;
}

static void process_line(const char *line)
{
  uint32_t new_signal_good = 0U;

  if (line == NULL)
  {
    return;
  }

  if (!GliderRdssCodec_TryDecodeLine(line, &s_decoded_frame))
  {
    return;
  }

  s_rx_line_count++;
  if (s_mode_switch_cooldown > 0U)
  {
    s_mode_switch_cooldown--;
  }

  if (s_decoded_frame.is_rd_mode)
  {
    s_rd_mode_active = 1U;
    s_rn_mode_active = 0U;
    s_tc_mode_active = 0U;
    s_tc_mode_requested = 0U;
    s_transparent_mode_ready = 0U;
  }

  if (text_contains_token(s_decoded_frame.text, "TC MODE") || text_contains_token(s_decoded_frame.text, "TCMODE"))
  {
    s_tc_mode_active = 1U;
    s_tc_mode_requested = 0U;
    s_rn_mode_active = 0U;
    s_rd_mode_active = 0U;
    s_transparent_mode_ready = 0U;
    s_target_card_query_attempted = 0U;
  }

  if (s_tc_mode_requested != 0U && s_tc_mode_active == 0U && text_contains_token(s_decoded_frame.text, "OK"))
  {
    s_tc_mode_active = 1U;
    s_tc_mode_requested = 0U;
    s_rn_mode_active = 0U;
    s_rd_mode_active = 0U;
    s_transparent_mode_ready = 0U;
    s_target_card_query_attempted = 0U;
  }

  if (text_contains_token(s_decoded_frame.text, "ERROR"))
  {
    s_tc_mode_active = 0U;
    s_tc_mode_requested = 0U;
    s_transparent_mode_ready = 0U;
    s_shortmsg_waiting_ack = 0U;
  }

  if (try_parse_target_card_from_sendto(s_decoded_frame.text))
  {
    s_target_card_query_attempted = 0U;
  }

  if (s_shortmsg_waiting_ack != 0U)
  {
    if (text_contains_token(s_decoded_frame.text, "SEND OK"))
    {
      strncpy(s_last_shortmsg_payload, s_pending_shortmsg_payload, sizeof(s_last_shortmsg_payload) - 1U);
      s_last_shortmsg_payload[sizeof(s_last_shortmsg_payload) - 1U] = '\0';
      s_shortmsg_pending = 0U;
      s_shortmsg_waiting_ack = 0U;
      s_shortmsg_sent_count++;
      if (cached_fix_expired(HAL_GetTick()))
      {
        invalidate_cached_fix_and_request_relocate();
      }
    }
    else if (text_contains_token(s_decoded_frame.text, "BDFKI") && text_contains_token(s_decoded_frame.text, "TXA"))
    {
      s_shortmsg_waiting_ack = 0U;
      if (text_contains_token(s_decoded_frame.text, ",Y,Y,") || text_contains_token(s_decoded_frame.text, ",TXA,Y,"))
      {
        strncpy(s_last_shortmsg_payload, s_pending_shortmsg_payload, sizeof(s_last_shortmsg_payload) - 1U);
        s_last_shortmsg_payload[sizeof(s_last_shortmsg_payload) - 1U] = '\0';
        s_shortmsg_pending = 0U;
        s_shortmsg_sent_count++;
        if (cached_fix_expired(HAL_GetTick()))
        {
          invalidate_cached_fix_and_request_relocate();
        }
      }
      else
      {
        s_transparent_mode_ready = 0U;
      }
    }
  }

  if (try_parse_local_card_from_bdici(s_decoded_frame.text))
  {
    s_rd_mode_active = 1U;
    s_rn_mode_active = 0U;
  }

  if (try_parse_signal_from_bdbsi(s_decoded_frame.text, &new_signal_good))
  {
    s_rd_mode_active = 1U;
    s_rn_mode_active = 0U;
    if (s_bdpwi_signal_available == 0U)
    {
      update_signal_state(new_signal_good);
    }
  }

  if (s_decoded_frame.is_bdpwi)
  {
    s_rd_mode_active = 1U;
    s_rn_mode_active = 0U;
    s_bdpwi_signal_available = 1U;
    update_signal_state(s_decoded_frame.bdpwi.signal_good ? 1U : 0U);
  }

  if (s_decoded_frame.is_rn_mode || s_decoded_frame.is_gnrmc)
  {
    /* Treat RN text OR continuous GNRMC stream as RN evidence.
     * This prevents stale TC flags from blocking re-entry/retry. */
    s_rn_mode_active = 1U;
    s_rd_mode_active = 0U;
    if (s_tc_mode_active != 0U || s_tc_mode_requested != 0U)
    {
      s_tc_mode_active = 0U;
      s_tc_mode_requested = 0U;
      s_transparent_mode_ready = 0U;
      s_shortmsg_waiting_ack = 0U;
      s_target_card_query_attempted = 0U;
    }
    if (s_decoded_frame.is_rn_mode && s_tx_mode == GLIDER_TX_MODE_SIGNAL_POLL)
    {
      s_tx_mode = GLIDER_TX_MODE_POSITION_TEST;
    }

    if (s_decoded_frame.is_gnrmc)
    {
      if (is_valid_gnrmc_fix(&s_decoded_frame))
      {
        (void)cache_coordinate_shortmsg(&s_decoded_frame);
      }
    }
  }

  try_queue_cached_shortmsg();

  (void)try_send_pending_shortmsg();

  GliderCore_OnDecodedFrame(&s_decoded_frame);

  if (s_decoded_frame.type == GLIDER_RDSS_FRAME_HEX_BINARY)
  {
    GliderCore_OnBinaryFrame(s_decoded_frame.payload, s_decoded_frame.payload_len);
    return;
  }

  GliderCore_OnTextFrame(s_decoded_frame.text);
}

static void process_rx_byte(const uint8_t byte)
{
  if (s_line_overflowed)
  {
    if (byte == '\n')
    {
      s_line_len = 0U;
      s_line_overflowed = false;
    }
    return;
  }

  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    if (s_line_len > 0)
    {
      s_line_buffer[s_line_len] = '\0';
      process_line(s_line_buffer);
      s_line_len = 0;
    }
    return;
  }

  if (s_line_len >= (GLIDER_RX_LINE_SIZE - 1U))
  {
    s_line_len = 0U;
    s_line_overflowed = true;
    return;
  }

  s_line_buffer[s_line_len++] = (char)byte;
}

void GliderCore_Init(UART_HandleTypeDef *uart_handle)
{
  s_uart = uart_handle;
  s_rx_head = 0;
  s_rx_tail = 0;
  s_line_len = 0;
  s_line_overflowed = false;
  s_rx_overflow_count = 0;
  s_rx_line_count = 0;
  s_tx_attempt_count = 0;
  s_tx_ok_count = 0;
  s_tx_fail_count = 0;
  s_tx_last_hal_status = 0;
  s_tx_last_uart_error = 0;
  s_bdpwi_signal_available = 0U;
  s_signal_good = 0U;
  s_tx_mode = GLIDER_TX_MODE_SIGNAL_POLL;
  s_shortmsg_sent_count = 0U;
  s_local_card_id_known = 0U;
  s_transparent_mode_ready = 0U;
  s_tc_mode_requested = 0U;
  s_tc_mode_active = 0U;
  s_rn_mode_active = 0U;
  s_rd_mode_active = 0U;
  s_local_card_mismatch = 0U;
  s_shortmsg_pending = 0U;
  s_shortmsg_waiting_ack = 0U;
  s_current_target_card_known = 0U;
  s_current_target_card_match = 0U;
  s_target_card_query_attempted = 0U;
  s_has_cached_fix = 0U;
  s_signal_good_streak = 0U;
  s_signal_bad_streak = 0U;
  s_mode_switch_cooldown = 0U;
  s_card_query_cooldown = 0U;
  s_last_valid_fix_tick = 0U;
  s_last_rn_request_tick = 0U;
  s_last_shortmsg_try_tick = 0U;
  s_last_tc_request_tick = 0U;
  s_last_rd_request_tick = 0U;
  s_last_tx_text[0] = '\0';
  s_last_shortmsg_payload[0] = '\0';
  s_pending_shortmsg_payload[0] = '\0';
  s_cached_shortmsg_payload[0] = '\0';
  s_local_card_id[0] = '\0';
  s_current_target_card[0] = '\0';

  if (s_uart != NULL)
  {
    (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  }
}

void GliderCore_RunRxTask(void)
{
  uint8_t byte = 0;

  for (;;)
  {
    while (ring_pop(&byte))
    {
      process_rx_byte(byte);
    }

    osDelay(2U);
  }
}

void GliderCore_RunTxTask(void)
{
  size_t idx = 0;
  uint32_t now = 0U;

  for (idx = 0; idx < (sizeof(s_boot_test_cmds) / sizeof(s_boot_test_cmds[0])); idx++)
  {
    (void)GliderCore_SendText(s_boot_test_cmds[idx]);
    osDelay(500U);
  }

  (void)GliderCore_SendText(GLIDER_RD_MODE_CMD);
  s_last_rd_request_tick = HAL_GetTick();
  osDelay(500U);

  for (;;)
  {
    now = HAL_GetTick();

    if (s_card_query_cooldown > 0U)
    {
      s_card_query_cooldown--;
    }

    if ((s_local_card_id_known == 0U || s_local_card_mismatch != 0U) && s_card_query_cooldown == 0U)
    {
      request_rd_mode_soft(now);
      (void)GliderCore_SendText(GLIDER_QUERY_LOCAL_CARD_CMD);
      s_card_query_cooldown = GLIDER_CARD_QUERY_COOLDOWN_LOOPS;
      osDelay(500U);
      continue;
    }

    try_queue_cached_shortmsg();

    if (s_shortmsg_pending != 0U || s_tc_mode_requested != 0U || s_tc_mode_active != 0U)
    {
      (void)try_send_pending_shortmsg();
      osDelay(500U);
      continue;
    }

    if (s_tx_mode == GLIDER_TX_MODE_RECEIVE_ONLY)
    {
      osDelay(1000U);
      continue;
    }

    if (s_tx_mode == GLIDER_TX_MODE_POSITION_TEST)
    {
      if (s_last_rn_request_tick == 0U || (now - s_last_rn_request_tick) >= GLIDER_RN_REFRESH_INTERVAL_MS)
      {
        (void)GliderCore_SendText(GLIDER_POSITION_TEST_CMD);
        s_last_rn_request_tick = now;
      }
    }
    else
    {
      request_rd_mode_soft(now);
      (void)GliderCore_SendText(GLIDER_QUERY_SIGNAL_CMD);
    }
    osDelay(5000U);
  }
}

HAL_StatusTypeDef GliderCore_SendText(const char *text)
{
  size_t len = 0;
  size_t copy_len = 0;
  size_t i = 0;
  HAL_StatusTypeDef ret = HAL_ERROR;

  if (s_uart == NULL || text == NULL)
  {
    s_tx_fail_count++;
    s_tx_last_hal_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  len = strlen(text);
  if (len == 0U)
  {
    return HAL_OK;
  }

  copy_len = (len < (sizeof(s_last_tx_text) - 1U)) ? len : (sizeof(s_last_tx_text) - 1U);
  for (i = 0U; i < copy_len; i++)
  {
    if (text[i] == '\r' || text[i] == '\n')
    {
      break;
    }
    s_last_tx_text[i] = text[i];
  }
  s_last_tx_text[i] = '\0';

  s_tx_attempt_count++;
  ret = HAL_UART_Transmit(s_uart, (uint8_t *)text, (uint16_t)len, 200U);
  s_tx_last_hal_status = (uint32_t)ret;
  s_tx_last_uart_error = (uint32_t)s_uart->ErrorCode;

  if (ret == HAL_OK)
  {
    s_tx_ok_count++;
  }
  else
  {
    s_tx_fail_count++;
  }

  return ret;
}

uint32_t GliderCore_GetRxLineCount(void)
{
  return s_rx_line_count;
}

uint32_t GliderCore_GetRxOverflowCount(void)
{
  return s_rx_overflow_count;
}

uint32_t GliderCore_GetTxAttemptCount(void)
{
  return s_tx_attempt_count;
}

uint32_t GliderCore_GetTxOkCount(void)
{
  return s_tx_ok_count;
}

uint32_t GliderCore_GetTxFailCount(void)
{
  return s_tx_fail_count;
}

uint32_t GliderCore_GetLastTxHalStatus(void)
{
  return s_tx_last_hal_status;
}

uint32_t GliderCore_GetLastUartErrorCode(void)
{
  return s_tx_last_uart_error;
}

uint32_t GliderCore_GetSignalGood(void)
{
  return s_signal_good;
}

uint32_t GliderCore_GetTxMode(void)
{
  return s_tx_mode;
}

const char *GliderCore_GetLastTxText(void)
{
  return s_last_tx_text;
}

uint32_t GliderCore_GetShortMsgSentCount(void)
{
  return s_shortmsg_sent_count;
}

const char *GliderCore_GetLastShortMsgPayload(void)
{
  return s_last_shortmsg_payload;
}

const char *GliderCore_GetCardId(void)
{
  return s_local_card_id;
}

uint32_t GliderCore_GetCardIdKnown(void)
{
  return s_local_card_id_known;
}

const char *GliderCore_GetPlatformCardId(void)
{
  return GLIDER_PLATFORM_CARD_ID;
}

uint32_t GliderCore_GetCardMismatch(void)
{
  return s_local_card_mismatch;
}

uint32_t GliderCore_GetHasCachedFix(void)
{
  return s_has_cached_fix;
}

uint32_t GliderCore_GetShortMsgPending(void)
{
  return s_shortmsg_pending;
}

uint32_t GliderCore_GetReadyToSend(void)
{
  return (s_signal_good != 0U && s_local_card_id_known != 0U && s_local_card_mismatch == 0U && s_has_cached_fix != 0U) ? 1U : 0U;
}

__weak void GliderCore_OnTextFrame(const char *line)
{
  (void)line;
}

__weak void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len)
{
  (void)payload;
  (void)payload_len;
}

__weak void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame)
{
  (void)frame;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (s_uart != NULL && huart->Instance == s_uart->Instance)
  {
    ring_push(s_rx_byte);
    (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rx_byte, 1U);
  }

  PressureSensorDriver_OnRxComplete(huart);
  SbusDriver_OnRxComplete(huart);
  EthernetUartDriver_OnRxComplete(huart);

  /* Forward to IMU module if it's UART4 */
  IMU_OnUART_RxComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  PressureSensorDriver_OnTxComplete(huart);
  EthernetUartDriver_OnTxComplete(huart);
}
