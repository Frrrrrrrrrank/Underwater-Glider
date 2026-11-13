#ifndef GLIDER_RAW_CAPTURE_H
#define GLIDER_RAW_CAPTURE_H

#include <stdint.h>

#define RAW_CAPTURE_KEEP_MAX 384U
#define RAW_CAPTURE_QUEUE_CAPACITY 8U
#define RAW_CAPTURE_LEASE_MS 15000U
#define RAW_CAPTURE_WIRE_BUDGET 2048U
#define RAW_CAPTURE_LINE_MAX 1024U

/* 仅Ethernet task调用发送入口；返回1表示底层已接管静态缓冲，0表示未发送。
 * 回调必须是非阻塞提交，不得等待TX完成；在短临界区内调用。 */
typedef int (*RawCaptureSendFn)(const uint8_t *data, uint16_t length);

/* 仅调度器启动前调用；无RTOS临界区，不支持运行中重置模块。 */
void RawCapture_Init(void);
void RawCapture_SetEnabled(uint8_t enabled, uint32_t now_ms);
void RawCapture_Poll(uint32_t now_ms);
/* RX task只做有界复制。seq对本次capture启用期间的完整LF行递增。
 * src_len包括CR但不包括终结LF；超长计数饱和，不从kept_len反推原长。 */
void RawCapture_Observe(const uint8_t *data, uint32_t src_len, uint16_t kept_len,
                        uint64_t rx_us, uint32_t uart_drop, uint32_t line_drop,
                        uint32_t now_ms);
/* 调用前须确认UART空闲；HAL异步发送期间不得再次格式化/改写静态发送缓冲。 */
int RawCapture_TryTransmit(uint32_t now_ms, uint64_t tx_us, RawCaptureSendFn send);

#endif
