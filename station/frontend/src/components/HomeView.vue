<script setup lang="ts">
import { computed, ref } from 'vue'
import type { DeviceLinkStatus, GamepadPreview, TelemetrySample } from '../types'
import { gamepadStatusLabel } from '../domain/gamepad'
import { numeric } from '../domain/fields'
import DataValue from './DataValue.vue'

const props = defineProps<{
  telemetry: Record<string, TelemetrySample>
  coreLive: boolean
  deviceStatus: DeviceLinkStatus | 'unknown'
  gamepad: GamepadPreview | null
  gamepadIssue: string | null
  gamepadSlot: number
}>()
const emit = defineEmits<{ navigate: [page: 'monitor' | 'diagnostic' | 'gamepad'] }>()
const inputSlot = computed(() => props.gamepad?.slots.find(item => item.index === props.gamepadSlot))
const selected = ref<'overview' | 'calibration'>('overview')
const roll = computed(() => numeric(props.telemetry.ugekf, 'ugekf', 'roll_deg'))
const pitch = computed(() => numeric(props.telemetry.ugekf, 'ugekf', 'pitch_deg'))
const yaw = computed(() => numeric(props.telemetry.ugekf, 'ugekf', 'yaw_deg'))
const poseValid = computed(() => props.coreLive && props.deviceStatus === 'connected' && roll.value !== null && pitch.value !== null)
const headingValid = computed(() => poseValid.value && yaw.value !== null)
const heading = computed(() => yaw.value === null ? null : ((yaw.value % 360) + 360) % 360)
const homingActive = computed(() => {
  const sample = props.telemetry.ugwrm
  if (sample?.data.schema === 3) return numeric(sample, 'ugwrm', 'calib_state') === 1
  const pitchState = numeric(sample, 'ugwrm', 'home_state')
  return pitchState === 1 || pitchState === 2
})
const horizonTransform = computed(() => 'rotate(' + -(roll.value ?? 0) + ' 300 160) translate(0 ' + (pitch.value ?? 0) * 4 + ')')
const attitudeTicks = [-30, -20, -10, 0, 10, 20, 30]
const metrics = [
  { topic: 'ugekf' as const, key: 'depth', label: '深度', unit: 'm' },
  { topic: 'ugekf' as const, key: 'pitch_deg', label: '俯仰角', unit: '°' },
  { topic: 'ugekf' as const, key: 'roll_deg', label: '横滚角', unit: '°' },
  { topic: 'ugekf' as const, key: 'vD', label: '下向速度', unit: 'm/s' },
  { topic: 'ugwrm' as const, key: 'stroke_percent', label: '俯仰轴位置', unit: '%' },
  { topic: 'ugwrm' as const, key: 'roll_percent', label: '横滚轴位置（估算）', unit: '%' },
  { topic: 'ugctl' as const, key: 'displacement_mm', label: '油囊位移', unit: 'mm' },
]
// 仪表图形随容器缩放；读数和空态使用 HTML，保持固定可读字号。
</script>

<template>
  <section class="home-view" data-testid="home-view">
    <div class="home-stage">
      <section class="situation-view" aria-label="机器人态势仪表">
        <header class="situation-heading"><h3>姿态与航向</h3><span>估计器反馈</span></header>
        <div class="instrument-layout">
          <div class="attitude" data-testid="home-attitude" :data-valid="poseValid">
            <div class="instrument-label">姿态</div>
            <div class="attitude-drawing">
              <svg viewBox="0 0 600 320" role="img" aria-label="横滚俯仰仪表">
                <defs><clipPath id="urgc-home-attitude-clip"><rect x="35" y="20" width="530" height="280" /></clipPath></defs>
                <g class="instrument-frame"><path d="M70 60V32H102 M498 32H530V60 M70 260V288H102 M498 288H530V260" /><path d="M300 22V33 M300 288V299" /></g>
                <g v-if="poseValid" clip-path="url(#urgc-home-attitude-clip)">
                  <g :transform="horizonTransform" class="pitch-ladder">
                    <g v-for="angle in attitudeTicks" :key="angle" :transform="'translate(0 ' + -angle * 4 + ')'">
                      <path :d="angle === 0 ? 'M70 160H264 M336 160H530' : 'M232 160H278 M322 160H368'" :class="{ horizon: angle === 0 }" :stroke-dasharray="angle < 0 ? '5 5' : undefined" />
                    </g>
                  </g>
                  <path class="vehicle-reticle" d="M232 160H275L287 172 M368 160H325L313 172 M300 147V173" />
                  <circle class="vehicle-center" cx="300" cy="160" r="4" />
                </g>
                <path v-else class="instrument-empty" d="M262 160H338 M300 135V185" />
              </svg>
              <p v-if="!poseValid" class="attitude-empty">未收到有效估计</p>
            </div>
            <dl class="instrument-readouts" data-testid="home-attitude-readout">
              <div><dt>横滚</dt><dd :class="{ unavailable: !poseValid }">{{ poseValid ? roll?.toFixed(1) + '°' : '—' }}</dd></div>
              <div><dt>俯仰</dt><dd :class="{ unavailable: !poseValid }">{{ poseValid ? pitch?.toFixed(1) + '°' : '—' }}</dd></div>
            </dl>
          </div>
          <div class="heading-instrument">
            <div class="instrument-label">航向</div>
            <div class="compass-drawing">
              <span class="compass-direction north">N</span><span class="compass-direction east">E</span><span class="compass-direction south">S</span><span class="compass-direction west">W</span>
              <svg viewBox="0 0 200 200" role="img" aria-label="估计航向罗盘">
                <circle cx="100" cy="100" r="76" class="compass-ring" /><circle cx="100" cy="100" r="57" class="compass-inner" />
                <path class="compass-ticks" d="M100 24V35 M176 100H165 M100 176V165 M24 100H35" />
                <g v-if="headingValid" :transform="'rotate(' + heading + ' 100 100)'" data-testid="home-heading-pointer"><path class="heading-pointer" d="M100 42L111 112L100 105L89 112Z" /><path class="heading-tail" d="M100 105V143" /></g>
              </svg>
              <span v-if="!headingValid" class="compass-missing unavailable">—</span>
            </div>
            <strong class="heading-number" data-testid="home-heading-readout" :class="{ unavailable: !headingValid }">{{ headingValid ? heading?.toFixed(1) + '°' : '—' }}</strong>
          </div>
        </div>
      </section>

      <aside class="operation-panel" aria-label="机器人状态">
        <nav class="status-tabs" aria-label="状态分组">
          <button data-home-tab="overview" :class="{ selected: selected === 'overview' }" :aria-pressed="selected === 'overview'" @click="selected = 'overview'">状态</button>
          <button data-home-tab="calibration" :class="{ selected: selected === 'calibration' }" :aria-pressed="selected === 'calibration'" @click="selected = 'calibration'">双轴</button>
        </nav>
        <div class="operation-content">
          <div v-if="selected === 'overview'" data-testid="home-status-panel">
            <h4>Glider <small>水下滑翔机</small></h4>
            <dl class="home-details">
              <div><dt>PID 模式</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="mode" /></dd></div>
              <div><dt>PID 使能</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="armed" /></dd></div>
              <div><dt>PID 输出有效</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="output_valid" /></dd></div>
              <div><dt>PID 状态</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="fault" /></dd></div>
              <div><dt>双轴校准</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_state" /></dd></div>
              <div><dt>控制来源</dt><dd class="unavailable" title="当前遥测未提供控制来源">—</dd></div>
            </dl>
            <div class="home-input-summary" data-testid="home-gamepad-summary">
              <div><span>手柄 {{ gamepadSlot }}</span><strong :class="inputSlot?.fresh ? 'tone-ok' : 'tone-muted'" :title="gamepadIssue ?? undefined">{{ inputSlot ? gamepadStatusLabel[inputSlot.status] : '输入不可用' }}</strong></div>
              <button class="details-link" data-testid="home-gamepad-link" @click="emit('navigate', 'gamepad')">手柄输入 ↗</button>
            </div>
          </div>
          <div v-else data-testid="home-calibration-panel">
            <h4 :class="{ 'tone-warn': homingActive }">双轴校准</h4>
            <dl class="home-details">
              <div><dt>总状态</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_state" /></dd></div>
              <div><dt>失败原因</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_fail" /></dd></div>
              <div><dt>俯仰轴阶段</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="home_state" /></dd></div>
              <div><dt>俯仰轴故障</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="home_fail" /></dd></div>
              <div><dt>横滚轴阶段</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_home_state" /></dd></div>
              <div><dt>横滚轴故障</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_home_fail" /></dd></div>
              <div><dt>横滚轴位置（估算）</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_percent" /> %</dd></div>
              <div><dt>校准控制保持</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_hold" /></dd></div>
            </dl>
          </div>
        </div>
        <footer class="status-links"><button @click="emit('navigate', 'monitor')">运行监控 ↗</button><button @click="emit('navigate', 'diagnostic')">遥测诊断 ↗</button></footer>
      </aside>
    </div>
    <div class="home-telemetry" aria-label="主页关键遥测">
      <div v-for="metric in metrics" :key="metric.key" class="home-metric" :data-testid="'home-metric-' + metric.key">
        <span>{{ metric.label }}</span><strong><DataValue :sample="telemetry[metric.topic]" :topic="metric.topic" :field="metric.key" /><small>{{ metric.unit }}</small></strong>
      </div>
    </div>
  </section>
</template>

<style scoped>
.home-view { min-height: 0; min-width: 0; display: grid; grid-template-rows: minmax(0, 1fr) 86px; gap: 10px; background: #000; }
.home-stage { min-height: 0; min-width: 0; display: grid; grid-template-columns: minmax(0, 1fr) 340px; gap: 10px; }
.situation-view, .operation-panel { min-width: 0; min-height: 0; border: 1px solid #28252f; background: #000; overflow: hidden; }
.situation-view { display: flex; flex-direction: column; }
.situation-heading { display: flex; align-items: center; justify-content: space-between; min-height: 46px; padding: 0 18px; gap: 10px; border-bottom: 1px solid #28252f; flex-shrink: 0; }
.situation-heading h3 { margin: 0; font-size: 14px; font-weight: 500; }
.situation-heading > span { color: #a59aaf; font-size: 12px; }
.instrument-layout { display: grid; grid-template-columns: minmax(0, 1fr) minmax(160px, 25%); align-items: center; min-height: 0; flex: 1; padding: 20px; gap: 20px; }
.attitude, .heading-instrument { min-width: 0; min-height: 0; }
.instrument-label { color: #a99fb3; font-size: 13px; padding: 0 6px 12px; }
.attitude-drawing { position: relative; }
.attitude svg { width: 100%; display: block; max-height: 500px; }
.instrument-frame, .pitch-ladder path { fill: none; stroke: #666173; stroke-width: 1.3; vector-effect: non-scaling-stroke; }
.pitch-ladder .horizon { stroke: #a79bb9; stroke-width: 1.5; }
.vehicle-reticle { stroke: #dfc7ff; fill: none; stroke-width: 3; }
.vehicle-center { fill: #dfc7ff; }
.instrument-empty { fill: none; stroke: #51485e; stroke-width: 1.3; }
.attitude-empty { position: absolute; top: 64%; left: 0; right: 0; text-align: center; color: #a99fb3; font-size: 14px; }
.instrument-readouts { display: flex; justify-content: space-between; gap: 20px; margin: 18px 9% 0; }
.instrument-readouts > div { display: flex; align-items: baseline; gap: 12px; }
.instrument-readouts dt { color: #a99fb3; font-size: 13px; }
.instrument-readouts dd, .heading-number { font: 28px Consolas, monospace; font-variant-numeric: tabular-nums; }
.heading-instrument { display: flex; flex-direction: column; align-items: center; gap: 10px; }
.heading-instrument .instrument-label { padding: 0; }
.compass-drawing { position: relative; width: 100%; max-width: 240px; aspect-ratio: 1; }
.compass-drawing svg { display: block; width: 100%; height: 100%; }
.compass-direction { position: absolute; font: 14px Consolas, monospace; color: #aca1b7; }
.compass-direction.north { top: 0; left: 50%; transform: translateX(-50%); }
.compass-direction.south { bottom: 0; left: 50%; transform: translateX(-50%); }
.compass-direction.east { right: 0; top: 50%; transform: translateY(-50%); }
.compass-direction.west { left: 0; top: 50%; transform: translateY(-50%); }
.compass-ring, .compass-inner, .compass-ticks { stroke: #665373; fill: none; stroke-width: 1.5; vector-effect: non-scaling-stroke; }
.compass-inner { stroke: #352c3e; stroke-dasharray: 2 8; }
.heading-pointer { fill: #cfb3f3; }
.heading-tail { stroke: #8a729e; stroke-width: 2; }
.compass-missing { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font: 26px Consolas, monospace; }
.operation-panel { display: flex; flex-direction: column; }
.status-tabs { display: grid; grid-template-columns: 1fr 1fr; border-bottom: 1px solid #28252f; flex-shrink: 0; }
.status-tabs button { min-height: 45px; border: 0; border-bottom: 2px solid transparent; border-radius: 0; background: #000; color: #a99fb3; font-size: 13px; }
.status-tabs button:hover { background: #130f19; }
.status-tabs button.selected { background: #21172e; border-bottom-color: #a586bf; color: #e4d3ff; }
.operation-content { min-height: 0; flex: 1; padding: 16px; }
.operation-content h4 { font-size: 21px; margin: 0 0 14px; font-weight: 500; }
.operation-content h4 small { color: #a396af; font-size: 12px; margin-left: 7px; }
.home-details > div { display: flex; justify-content: space-between; align-items: baseline; gap: 12px; padding: 11px 0; border-bottom: 1px solid #25202b; }
.home-details dt { color: #a99fb3; font-size: 12px; flex-shrink: 0; }
.home-details dd { font-size: 13px; text-align: right; overflow-wrap: anywhere; }
.home-input-summary { margin-top: 18px; }
.home-input-summary > div { display: flex; justify-content: space-between; gap: 12px; align-items: baseline; }
.home-input-summary span { color: #a99fb3; font-size: 12px; }
.home-input-summary strong { font-size: 13px; font-weight: 500; }
.details-link, .status-links button { background: #000; border: 0; color: #c2abd9; font-size: 12px; padding: 0; }
.details-link { margin-top: 12px; padding-block: 4px; }
.status-links { display: flex; align-items: center; justify-content: space-between; padding: 12px 16px; border-top: 1px solid #28252f; flex-shrink: 0; }
.home-telemetry { min-width: 0; display: grid; grid-template-columns: repeat(7, minmax(0, 1fr)); border: 1px solid #302737; align-items: center; }
.home-metric { min-width: 0; padding: 0 16px; border-right: 1px solid #302737; }
.home-metric:last-child { border-right: 0; }
.home-metric > span { font-size: 12px; color: #a99fb3; white-space: nowrap; }
.home-metric strong { display: flex; align-items: baseline; gap: 6px; font: 25px Consolas, monospace; margin-top: 8px; white-space: nowrap; }
.home-metric small { font: 12px 'Microsoft YaHei', sans-serif; color: #a99fb3; }
@media (max-width: 1450px) {
  .home-stage { grid-template-columns: minmax(0, 1fr) 300px; gap: 8px; }
  .instrument-layout { grid-template-columns: minmax(0, 1fr) 160px; padding: 15px; gap: 12px; }
  .home-metric { padding-inline: 11px; }
  .home-metric strong { font-size: 22px; }
  .instrument-readouts { margin-inline: 6%; }
}
@media (max-height: 800px) {
  .home-view { grid-template-rows: minmax(0, 1fr) 76px; gap: 8px; }
  .situation-heading { min-height: 40px; }
  .instrument-layout { padding-block: 12px; }
  .attitude svg { max-height: 300px; }
  .instrument-label { padding-bottom: 8px; }
  .instrument-readouts { margin-top: 12px; }
  .instrument-readouts dd, .heading-number { font-size: 25px; }
  .status-tabs button { min-height: 40px; }
  .operation-content { padding: 12px 14px; }
  .operation-content h4 { margin-bottom: 10px; font-size: 19px; }
  .home-details > div { padding-block: 8px; }
  .home-input-summary { margin-top: 13px; }
  .details-link { margin-top: 7px; }
  .status-links { padding: 10px 14px; }
}
@media (max-width: 1150px) {
  .home-stage { grid-template-columns: minmax(0, 1fr) 280px; }
  .instrument-layout { grid-template-columns: minmax(0, 1fr) 130px; padding-inline: 10px; gap: 8px; }
  .instrument-readouts { margin-inline: 0; gap: 10px; }
  .instrument-readouts > div { gap: 6px; }
  .home-metric { padding-inline: 8px; }
  .home-metric strong { font-size: 20px; }
}
</style>
