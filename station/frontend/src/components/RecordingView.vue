<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
const props = defineProps<{ coreLive: boolean }>()
interface RecordingState {
  enabled: boolean; status: 'idle' | 'starting' | 'recording' | 'error' | 'stopped'; session_id: string | null;
  started_at: string | null; directory: string | null; files: Record<string, string>; rows: Record<string, number>;
  raw_lines: number; dropped_records: number; last_error: string | null
}
const recording = ref<RecordingState | null>(null)
const requestError = ref<string | null>(null)
const confirmed = ref(false)
let timer: number | undefined
let controller: AbortController | null = null
let disposed = false
const statusLabels = { idle: '等待会话', starting: '正在准备记录', recording: '正在记录', error: '记录异常', stopped: '记录已停止' }
const statusText = computed(() => !props.coreLive || !confirmed.value ? '记录状态不可确认' : !recording.value?.enabled ? '记录未启用' : statusLabels[recording.value.status])
function valid(value: unknown): value is RecordingState {
  if (!value || typeof value !== 'object') return false
  const item = value as Partial<RecordingState>
  return typeof item.enabled === 'boolean' && typeof item.status === 'string' && Object.hasOwn(statusLabels, item.status)
    && ['session_id', 'started_at', 'directory', 'last_error'].every(key => (item as Record<string, unknown>)[key] === null || typeof (item as Record<string, unknown>)[key] === 'string')
    && !!item.files && typeof item.files === 'object' && !Array.isArray(item.files) && Object.values(item.files).every(file => typeof file === 'string')
    && !!item.rows && typeof item.rows === 'object' && !Array.isArray(item.rows) && Object.values(item.rows).every(count => typeof count === 'number' && Number.isFinite(count) && count >= 0)
    && typeof item.raw_lines === 'number' && Number.isFinite(item.raw_lines) && typeof item.dropped_records === 'number' && Number.isFinite(item.dropped_records)
}
async function refresh() {
  if (controller || disposed || !props.coreLive) return
  const current = new AbortController(); controller = current
  const timeout = window.setTimeout(() => current.abort(), 2_500)
  try {
    const response = await fetch('/api/v1/recording', { signal: current.signal, cache: 'no-store' })
    if (!response.ok) throw new Error('记录接口 HTTP ' + response.status)
    const data: unknown = await response.json()
    if (!valid(data)) throw new Error('记录状态格式不兼容')
    if (!disposed && props.coreLive) { recording.value = data; confirmed.value = true; requestError.value = null }
  } catch (error) { if (!disposed) { confirmed.value = false; requestError.value = error instanceof Error && error.name === 'AbortError' ? '记录状态读取超时' : error instanceof Error ? error.message : '记录状态读取失败' } }
  finally { window.clearTimeout(timeout); controller = null }
}
watch(() => props.coreLive, value => { confirmed.value = false; if (!value) controller?.abort(); else void refresh() })
onMounted(() => { void refresh(); timer = window.setInterval(refresh, 2_000) })
onUnmounted(() => { disposed = true; window.clearInterval(timer); controller?.abort() })
</script>
<template>
  <section class="panel recording-view">
    <header class="panel-heading"><h2>会话记录</h2><span :class="coreLive && confirmed && recording?.status === 'recording' ? 'tone-ok' : 'tone-warn'">{{ statusText }}</span></header>
    <div class="panel-scroll recording-content">
      <p v-if="requestError || recording?.last_error" class="inline-alert">{{ requestError ?? recording?.last_error }}</p>
      <dl class="recording-details"><div><dt>会话编号</dt><dd class="mono">{{ recording?.session_id ?? '—' }}</dd></div><div><dt>开始时间</dt><dd>{{ recording?.started_at ? new Date(recording.started_at).toLocaleString('zh-CN', { hour12: false }) : '—' }}</dd></div><div><dt>保存目录</dt><dd class="mono">{{ recording?.directory ?? '等待设备会话建立' }}</dd></div><div><dt>原始报文 / 丢弃记录</dt><dd>{{ recording?.raw_lines ?? '—' }} / <span :class="recording?.dropped_records ? 'tone-bad' : ''">{{ recording?.dropped_records ?? '—' }}</span></dd></div></dl>
      <table class="data-table"><thead><tr><th>记录项</th><th>文件</th><th class="numeric">记录行数</th></tr></thead><tbody><tr v-for="(file, key) in recording?.files ?? {}" :key="key"><td>{{ key }}</td><td class="mono break-word">{{ file }}</td><td class="numeric mono">{{ recording?.rows[key] ?? '—' }}</td></tr></tbody></table>
      <p v-if="!recording || Object.keys(recording.files).length === 0" class="empty-message">设备建立会话后，记录文件和写入数量会显示在这里。</p>
      <p class="recording-note">文件保存在运行 URGC 核心的电脑上。此处显示实际记录状态；离线分析可使用现有日志工具。</p>
    </div>
  </section>
</template>
