#ifndef GLIDER_CORE_H
#define GLIDER_CORE_H

#include "main.h"
#include "glider_rdss_codec.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
  volatile uint32_t valid;
  volatile uint32_t update_count;
  volatile uint32_t last_update_tick;
  char source_card[24];
  volatile int32_t latitude_deg_e7;
  volatile int32_t longitude_deg_e7;
  volatile float latitude_deg;
  volatile float longitude_deg;
} GliderTargetCoordinate;

typedef enum
{
  GLIDER_SHORTMSG_RESULT_IDLE = 0U,
  GLIDER_SHORTMSG_RESULT_PENDING = 1U,
  GLIDER_SHORTMSG_RESULT_WAITING_ACK = 2U,
  GLIDER_SHORTMSG_RESULT_SUCCESS = 3U,
  GLIDER_SHORTMSG_RESULT_REJECTED = 4U,
  GLIDER_SHORTMSG_RESULT_TIMEOUT = 5U,
  GLIDER_SHORTMSG_RESULT_UART_ERROR = 6U
} GliderShortMsgResult;

typedef struct
{
  uint32_t signal_good;
  uint32_t ready_to_send;
  uint32_t pending;
  uint32_t waiting_ack;
  uint32_t last_result;
  uint32_t attempt_count;
  uint32_t sent_count;
  uint32_t reject_count;
  uint32_t timeout_count;
  uint32_t uart_fail_count;
  uint32_t last_hal_status;
  uint32_t last_uart_error;
  uint32_t tx_mode;
  uint32_t card_id_known;
  uint32_t card_mismatch;
  uint32_t has_cached_fix;
  uint32_t send_ok_count;
  int32_t local_latitude_deg_e7;
  int32_t local_longitude_deg_e7;
  uint32_t target_valid;
  uint32_t goto_received_count;
  int32_t target_latitude_deg_e7;
  int32_t target_longitude_deg_e7;
} GliderShortMsgStatus;

extern volatile GliderTargetCoordinate g_target_coordinate;
/* Counts valid GNRMC fixes, including unchanged coordinates. */
extern volatile uint32_t g_rdss_fix_update_count;
extern volatile uint32_t g_rdss_rn_request_count;

void GliderCore_Init(UART_HandleTypeDef *uart_handle);
void GliderCore_RunRxTask(void);
void GliderCore_RunTxTask(void);
HAL_StatusTypeDef GliderCore_SendText(const char *text);

uint32_t GliderCore_GetRxLineCount(void);
uint32_t GliderCore_GetRxOverflowCount(void);
uint32_t GliderCore_GetTxAttemptCount(void);
uint32_t GliderCore_GetTxOkCount(void);
uint32_t GliderCore_GetTxFailCount(void);
uint32_t GliderCore_GetLastTxHalStatus(void);
uint32_t GliderCore_GetLastUartErrorCode(void);
uint32_t GliderCore_GetSignalGood(void);
uint32_t GliderCore_GetTxMode(void);
const char *GliderCore_GetLastTxText(void);
uint32_t GliderCore_GetShortMsgSentCount(void);
const char *GliderCore_GetLastShortMsgPayload(void);
const char *GliderCore_GetCardId(void);
uint32_t GliderCore_GetCardIdKnown(void);
const char *GliderCore_GetPlatformCardId(void);
uint32_t GliderCore_GetCardMismatch(void);
uint32_t GliderCore_GetHasCachedFix(void);
uint32_t GliderCore_GetShortMsgPending(void);
uint32_t GliderCore_GetReadyToSend(void);
void GliderCore_CopyShortMsgStatus(GliderShortMsgStatus *out_status);

void GliderCore_OnTextFrame(const char *line);
void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len);
void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame);

#endif
