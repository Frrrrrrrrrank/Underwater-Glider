<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import type { TelemetrySample } from '../types'
import { numeric } from '../domain/fields'
const props = defineProps<{ estimate?: TelemetrySample; control?: TelemetrySample; now: number; active: boolean }>()
type Point = { time: number; depth: number | null; pitch: number | null; targetDepth: number | null; targetPitch: number | null }
const points = ref<Point[]>([])
let lastSession: string | null | undefined
let lastReceipt: string | undefined
let lastTick: number | null | undefined
watch(() => props.estimate, sample => {
  if (!sample) return
  // 会话切换与板端时钟回退都切断曲线，避免把不同上电过程连成一条线。
  if (sample.session_id !== lastSession || (sample.device_tick_ms !== null && lastTick != null && sample.device_tick_ms < lastTick)) {
    points.value = []; lastReceipt = undefined
  }
  lastSession = sample.session_id; lastTick = sample.device_tick_ms
  if (sample.received_at === lastReceipt) return
  lastReceipt = sample.received_at
  const enabled = props.control?.data.output_valid === 1
  points.value = [...points.value.filter(point => props.now - point.time < 60_000), {
    time: props.now,
    depth: numeric(sample, 'ugekf', 'depth'), pitch: numeric(sample, 'ugekf', 'pitch_deg'),
    targetDepth: enabled ? numeric(props.control, 'ugctl', 'target_depth') : null,
    targetPitch: enabled ? numeric(props.control, 'ugctl', 'pitch_sp_deg') : null,
  }].slice(-600)
}, { immediate: true })
const charts = computed(() => (['depth', 'pitch'] as const).map(key => {
  const target = key === 'depth' ? 'targetDepth' : 'targetPitch'
  const visible = points.value.filter(point => props.now - point.time <= 60_000)
  const values = visible.flatMap(point => [point[key], point[target]]).filter((value): value is number => value !== null)
  const center = values.length ? (Math.min(...values) + Math.max(...values)) / 2 : 0
  const range = Math.max(key === 'depth' ? 0.2 : 4, values.length ? Math.max(...values) - Math.min(...values) : 0) * 1.2
  const bottom = center - range / 2; const top = center + range / 2
  function path(field: typeof key | typeof target) {
    let drawing = ''; let previous: Point | null = null
    for (const point of visible) {
      const value = point[field]
      if (value === null) { previous = null; continue }
      const x = 44 + (60_000 - (props.now - point.time)) / 60_000 * 502
      const y = 13 + (top - value) / range * 78
      drawing += (previous && point.time - previous.time < 1_000 ? ' L' : ' M') + x.toFixed(1) + ',' + y.toFixed(1)
      previous = point
    }
    return drawing
  }
  return { key, label: key === 'depth' ? '深度 / m' : '俯仰 / °', top: top.toFixed(key === 'depth' ? 2 : 1), bottom: bottom.toFixed(key === 'depth' ? 2 : 1), line: path(key), targetLine: path(target), hasData: values.length > 0 }
}))
</script>
<template>
  <section class="panel trend-panel" aria-label="短时趋势">
    <header class="panel-heading"><h2>短时趋势</h2><span class="panel-meta"><i class="legend actual"></i>估计反馈 <i class="legend target"></i>有效控制目标 · 最近 60 s</span></header>
    <div class="trend-grid" :class="{ historical: !active }">
      <div v-for="chart in charts" :key="chart.key" class="trend-chart">
        <div class="chart-title">{{ chart.label }}<span v-if="!active && chart.hasData">历史片段</span></div>
        <svg viewBox="0 0 560 110" preserveAspectRatio="none" role="img" :aria-label="chart.label + ' 最近60秒趋势'">
          <path class="chart-grid" d="M44 13H546 M44 52H546 M44 91H546 M44 13V91 M295 13V91 M546 13V91" />
          <text x="2" y="17">{{ chart.hasData ? chart.top : '—' }}</text><text x="2" y="94">{{ chart.hasData ? chart.bottom : '—' }}</text>
          <text x="44" y="107">−60 s</text><text x="508" y="107">现在</text>
          <path class="chart-line target-line" :d="chart.targetLine" /><path class="chart-line" :d="chart.line" />
          <text v-if="!chart.hasData" x="295" y="56" text-anchor="middle" class="chart-empty">等待有效数据</text>
        </svg>
      </div>
    </div>
  </section>
</template>
