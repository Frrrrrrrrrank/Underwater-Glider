#include "raw_capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
  uint64_t rx_us;
  uint32_t capture_id, seq, uart_drop, line_drop, src_len;
  uint16_t kept_len;
  uint8_t data[RAW_CAPTURE_KEEP_MAX];
} RawCaptureRecord;

static RawCaptureRecord s_queue[RAW_CAPTURE_QUEUE_CAPACITY];
/* Ethernet task独占这两个暂存区；避免约1.5KiB局部变量挤占其2KiB任务栈。 */
static RawCaptureRecord s_snapshot;
static char s_wire[RAW_CAPTURE_LINE_MAX];
static uint32_t s_capture_id, s_seq, s_queue_drop;
static uint32_t s_lease_tick, s_window_tick, s_wire_used;
static uint8_t s_head, s_count, s_enabled;

static void expire_locked(uint32_t now_ms)
{
  const uint32_t elapsed = now_ms - s_lease_tick;
  /* RX取时后可被Ethernet续租抢占，较旧now不能用无符号下溢误判超时。 */
  if (s_enabled && elapsed >= RAW_CAPTURE_LEASE_MS && elapsed < 0x80000000U)
  {
    s_enabled = 0U;
    s_head = s_count = 0U;
  }
}

void RawCapture_Init(void)
{
  /* 仅main启动调度器前调用，此时没有RX task写者；不能用FreeRTOS临界区。
   * 端口初始critical nesting尚未归零，会把BASEPRI留住并阻断后续HAL_Delay。 */
  s_capture_id = s_seq = s_queue_drop = 0U;
  s_lease_tick = s_window_tick = s_wire_used = 0U;
  s_head = s_count = s_enabled = 0U;
}

void RawCapture_SetEnabled(uint8_t enabled, uint32_t now_ms)
{
  taskENTER_CRITICAL();
  expire_locked(now_ms);
  if (enabled)
  {
    if (!s_enabled)
    {
      ++s_capture_id;
      s_seq = s_queue_drop = 0U;
      s_head = s_count = 0U;
    }
    s_enabled = 1U;
    s_lease_tick = now_ms;
  }
  else
  {
    s_enabled = 0U;
    s_head = s_count = 0U;
  }
  /* 续租不重置capture_id、seq、FIFO或预算；重开也不能绕过带宽预算。
   * 关闭不改正在TX的s_wire；queue_drop累计本次capture中FIFO满时挤掉的旧行。 */
  taskEXIT_CRITICAL();
}

void RawCapture_Poll(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  expire_locked(now_ms);
  taskEXIT_CRITICAL();
}

void RawCapture_Observe(const uint8_t *data, uint32_t src_len, uint16_t kept_len,
                        uint64_t rx_us, uint32_t uart_drop, uint32_t line_drop,
                        uint32_t now_ms)
{
  RawCaptureRecord *record;
  if (kept_len != (src_len < RAW_CAPTURE_KEEP_MAX ? src_len : RAW_CAPTURE_KEEP_MAX)) return;
  if (kept_len && data == NULL) return;
  taskENTER_CRITICAL();
  expire_locked(now_ms);
  if (s_enabled)
  {
    ++s_seq;
    if (s_count == RAW_CAPTURE_QUEUE_CAPACITY)
    {
      s_head = (uint8_t)((s_head + 1U) % RAW_CAPTURE_QUEUE_CAPACITY);
      --s_count;
      ++s_queue_drop;
    }
    record = &s_queue[(s_head + s_count) % RAW_CAPTURE_QUEUE_CAPACITY];
    record->capture_id = s_capture_id;
    record->seq = s_seq;
    record->rx_us = rx_us;
    record->uart_drop = uart_drop;
    record->line_drop = line_drop;
    record->src_len = src_len;
    record->kept_len = kept_len;
    if (kept_len) memcpy(record->data, data, kept_len);
    ++s_count;
  }
  taskEXIT_CRITICAL();
}

static uint16_t format_snapshot(uint64_t tx_us, uint32_t queue_drop)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t checksum = 0U;
  uint16_t i;
  int n = snprintf(s_wire, sizeof(s_wire),
      "$UGRAW,1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
      (unsigned long)s_snapshot.capture_id, (unsigned long)s_snapshot.seq,
      (unsigned long)(uint32_t)(s_snapshot.rx_us >> 32), (unsigned long)(uint32_t)s_snapshot.rx_us,
      (unsigned long)(uint32_t)(tx_us >> 32), (unsigned long)(uint32_t)tx_us,
      (unsigned long)queue_drop, (unsigned long)s_snapshot.uart_drop,
      (unsigned long)s_snapshot.line_drop,
      (unsigned long)(s_snapshot.src_len > s_snapshot.kept_len ? 1U : 0U),
      (unsigned long)s_snapshot.src_len, (unsigned long)s_snapshot.kept_len);
  if (n <= 0 || (uint32_t)n + 2U * s_snapshot.kept_len + 5U >= sizeof(s_wire)) return 0U;
  for (i = 0U; i < s_snapshot.kept_len; ++i)
  {
    s_wire[n++] = hex[s_snapshot.data[i] >> 4];
    s_wire[n++] = hex[s_snapshot.data[i] & 15U];
  }
  for (i = 1U; i < (uint16_t)n; ++i) checksum ^= (uint8_t)s_wire[i];
  s_wire[n++] = '*';
  s_wire[n++] = hex[checksum >> 4];
  s_wire[n++] = hex[checksum & 15U];
  s_wire[n++] = '\r';
  s_wire[n++] = '\n';
  s_wire[n] = '\0';
  return (uint16_t)n;
}

int RawCapture_TryTransmit(uint32_t now_ms, uint64_t tx_us, RawCaptureSendFn send)
{
  uint32_t queue_drop;
  uint16_t length;
  int sent = 0;
  if (!send) return 0;
  taskENTER_CRITICAL();
  expire_locked(now_ms);
  if (!s_enabled || !s_count)
  {
    taskEXIT_CRITICAL();
    return 0;
  }
  s_snapshot = s_queue[s_head];
  queue_drop = s_queue_drop;
  taskEXIT_CRITICAL();

  length = format_snapshot(tx_us, queue_drop);
  if (!length) return 0;

  taskENTER_CRITICAL();
  /* 格式化时RX可能已挤掉旧头或采集已重新开始；不能发送旧快照后误取新头。
   * 核身份→非阻塞HAL提交→成功出队在同一短临界区，RX不会夹入其中。 */
  expire_locked(now_ms);
  if (s_enabled && s_count && s_queue[s_head].capture_id == s_snapshot.capture_id &&
      s_queue[s_head].seq == s_snapshot.seq)
  {
    if ((uint32_t)(now_ms - s_window_tick) >= 1000U)
    {
      s_window_tick = now_ms;
      s_wire_used = 0U;
    }
    if (length <= RAW_CAPTURE_WIRE_BUDGET - s_wire_used &&
        send((const uint8_t *)s_wire, length))
    {
      s_wire_used += length;
      s_head = (uint8_t)((s_head + 1U) % RAW_CAPTURE_QUEUE_CAPACITY);
      --s_count;
      sent = 1;
    }
  }
  taskEXIT_CRITICAL();
  return sent;
}
