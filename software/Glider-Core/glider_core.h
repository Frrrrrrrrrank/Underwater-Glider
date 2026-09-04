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

extern volatile GliderTargetCoordinate g_target_coordinate;

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

void GliderCore_OnTextFrame(const char *line);
void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len);
void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame);

#endif
