#ifndef GLIDER_CORE_H
#define GLIDER_CORE_H

#include "main.h"
#include "glider_c01_proto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GLIDER_C01_ROLE_MASTER 1U
#define GLIDER_C01_ROLE_SLAVE 2U

#ifndef GLIDER_C01_DEVICE_ROLE
#define GLIDER_C01_DEVICE_ROLE GLIDER_C01_ROLE_MASTER
#endif

#if (GLIDER_C01_DEVICE_ROLE != GLIDER_C01_ROLE_MASTER) && (GLIDER_C01_DEVICE_ROLE != GLIDER_C01_ROLE_SLAVE)
#error "GLIDER_C01_DEVICE_ROLE must be GLIDER_C01_ROLE_MASTER or GLIDER_C01_ROLE_SLAVE"
#endif

typedef struct
{
	uint32_t device_role;
	uint32_t tx_enabled;
	uint32_t rx_byte_count;
	uint32_t rx_overflow_count;
	uint32_t rx_line_count;
	uint32_t tx_attempt_count;
	uint32_t tx_ok_count;
	uint32_t tx_fail_count;
	uint32_t tx_last_hal_status;
	uint32_t tx_last_uart_error;
	uint32_t rx_frame_count;
	uint32_t rx_frame_checksum_error_count;
	uint32_t rx_last_function_code;
	uint32_t rx_remote_ack_count;
	uint32_t rx_remote_data_count;
	uint32_t rx_last_decoded_status;
	uint32_t rx_last_decoded_data;
	uint32_t tx_last_encoded_len;
	uint8_t tx_last_encoded_frame[GLIDER_C01_MAX_FRAME_SIZE];
	uint32_t rx_last_encoded_len;
	uint8_t rx_last_encoded_frame[GLIDER_C01_MAX_FRAME_SIZE];
	uint32_t rx_last_decoded_payload_len;
	uint8_t rx_last_decoded_payload[GLIDER_C01_MAX_FRAME_SIZE - 5U];
} GliderCoreDiag;

extern volatile GliderCoreDiag g_glider_core_diag;

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
uint32_t GliderCore_GetRxFrameCount(void);
uint32_t GliderCore_GetRxFrameChecksumErrorCount(void);
uint32_t GliderCore_GetLastRxFunctionCode(void);
uint32_t GliderCore_GetRxRemoteAckCount(void);
uint32_t GliderCore_GetRxRemoteDataCount(void);
uint32_t GliderCore_GetRxLastDecodedStatus(void);
uint32_t GliderCore_GetRxLastDecodedData(void);
const volatile GliderCoreDiag *GliderCore_GetDiag(void);

void GliderCore_OnTextFrame(const char *line);
void GliderCore_OnBinaryFrame(const uint8_t *payload, size_t payload_len);

#endif
