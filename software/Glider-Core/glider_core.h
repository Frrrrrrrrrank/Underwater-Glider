#ifndef GLIDER_CORE_H
#define GLIDER_CORE_H

#include "main.h"
#include "glider_rdss_codec.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

void GliderCore_OnTextFrame(const char *line);
void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len);
void GliderCore_OnDecodedFrame(const GliderRdssDecodedFrame *frame);

#endif
