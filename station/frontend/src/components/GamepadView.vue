<script setup lang="ts">
import { computed } from 'vue'
import type { GamepadInputSample, GamepadIntent, GamepadPreview } from '../types'
import { gamepadStatusLabel } from '../domain/gamepad'

const props = defineProps<{ gamepad: GamepadPreview | null; issue: string | null; selectedSlot: number }>()
const emit = defineEmits<{ selectSlot: [index: number] }>()
const slot = computed(() => props.gamepad?.slots.find(item => item.index === props.selectedSlot))
const sample = computed(() => slot.value?.fresh ? slot.value.sample : null)
const intent = computed(() => slot.value?.fresh ? slot.value.intent : null)
const serviceLabel = computed(() => props.gamepad ? { running: '采样运行', disabled: '采样禁用', stopped: '采样停止', error: '采样异常' }[props.gamepad.status] : '采样未知')
const axes: { key: 'left_x' | 'forward' | 'left_trigger' | 'right_trigger'; label: string; range: string }[] = [
  { key: 'forward', label: '左摇杆前后', range: '−1 后退 / +1 前进' },
  { key: 'left_x', label: '左摇杆左右', range: '−1 左 / +1 右' },
  { key: 'left_trigger', label: 'LT 左扳机', range: '0 松开 / 1 满按' },
  { key: 'right_trigger', label: 'RT 右扳机', range: '0 松开 / 1 满按' },
]
const buttons: { key: keyof Pick<GamepadInputSample, 'button_y' | 'button_a' | 'dpad_left' | 'dpad_right' | 'button_b' | 'button_x'>; label: string }[] = [
  { key: 'button_y', label: 'Y' }, { key: 'button_a', label: 'A' }, { key: 'dpad_left', label: '十字左' },
  { key: 'dpad_right', label: '十字右' }, { key: 'button_b', label: 'B' }, { key: 'button_x', label: 'X' },
]
const targets: { key: keyof Pick<GamepadIntent, 'left_percent' | 'right_percent' | 'pitch_rpm' | 'roll_rpm' | 'rudder_rate_deg_s'>; label: string; unit: string }[] = [
  { key: 'left_percent', label: '左推进目标', unit: '%' }, { key: 'right_percent', label: '右推进目标', unit: '%' },
  { key: 'pitch_rpm', label: '俯仰轴目标转速', unit: 'RPM' }, { key: 'roll_rpm', label: '横滚轴目标转速', unit: 'RPM' },
  { key: 'rudder_rate_deg_s', label: '尾舵目标变化率', unit: '°/s' },
]
const settings = computed(() => {
  const config = props.gamepad?.config
  return [
    ['摇杆死区', config ? (config.deadzone * 100).toFixed(0) + '%' : '—'],
    ['扳机死区', config ? (config.trigger_deadzone * 100).toFixed(0) + '%' : '—'],
    ['推进目标上限', config ? config.max_percent + '%' : '—'],
    ['俯仰轴 / 横滚轴转速', config ? config.pitch_rpm + ' / ' + config.roll_rpm + ' RPM' : '—'],
    ['尾舵目标变化率上限', config ? config.rudder_rate_deg_s + ' °/s' : '—'],
    ['输入超时 / 采样周期', config ? Math.round(config.input_timeout_s * 1000) + ' / ' + Math.round(config.poll_period_s * 1000) + ' ms' : '—'],
  ]
})
const mappings = [
  ['左摇杆', '前后双向推进；左右差动；仅横拨不原地转'],
  ['Y / A', '按住俯仰轴负 / 正转；松开或同按停止'],
  ['十字左 / 右', '按住横滚轴正 / 负转；松开或同按停止'],
  ['RT / LT', '尾舵目标正 / 负向改变；松开或双按保持目标'],
  ['B / X', 'B 尾舵回中；X 退出遥控'],
  ['泵 / 校准', '未映射'],
]
function format(value: number | null | undefined, digits = 1) { return typeof value === 'number' ? value.toFixed(digits) : '—' }
// 页面只展示核心提供的基础映射，不重复混控、不积分舵角、不发控制请求。
</script>

<template>
  <section class="gamepad-view" data-testid="gamepad-view" :data-available="slot?.fresh === true">
    <div class="gamepad-toolbar">
      <div><strong>手柄预览</strong></div>
      <div class="gamepad-slots" role="group" aria-label="选择预览手柄槽位">
        <button v-for="index in [0, 1, 2, 3]" :key="index" :data-testid="'gamepad-slot-' + index" :class="{ selected: selectedSlot === index }" :aria-pressed="selectedSlot === index" @click="emit('selectSlot', index)">
          <i :class="gamepad?.slots.find(item => item.index === index)?.fresh ? 'tone-ok' : 'tone-muted'"></i>槽位 {{ index }}
        </button>
      </div>
    </div>
    <div class="gamepad-state-strip" role="status">
      <span data-testid="gamepad-service-status">{{ serviceLabel }}</span>
      <strong data-testid="gamepad-status" :class="slot?.fresh ? 'tone-ok' : slot && ['stale', 'invalid'].includes(slot.status) ? 'tone-warn' : 'tone-muted'">{{ slot ? gamepadStatusLabel[slot.status] : '输入不可用' }}</strong>
      <span data-testid="gamepad-age">距上次采样 {{ slot?.age_ms === null || slot?.age_ms === undefined ? '—' : slot.age_ms + ' ms' }}</span>
      <span>中立状态 {{ slot?.neutral === null || slot?.neutral === undefined ? '—' : slot.neutral ? '已回中 / 松开' : '有输入' }}</span>
      <span v-if="issue || slot?.reason" class="gamepad-state-detail">{{ issue ?? slot?.reason }}</span>
    </div>
    <div class="gamepad-grid">
      <section class="panel gamepad-input-panel" data-testid="gamepad-input-panel">
        <header class="panel-heading"><h2>实时输入</h2><span class="panel-meta">归一化值 · 死区前</span></header>
        <div class="panel-scroll gamepad-table-scroll">
          <table class="data-table gamepad-table"><thead><tr><th>输入</th><th class="numeric">当前值</th><th>方向 / 范围</th></tr></thead><tbody>
            <tr v-for="axis in axes" :key="axis.key"><td>{{ axis.label }}</td><td class="numeric mono" :class="{ unavailable: !sample }" :data-testid="'gamepad-axis-' + axis.key" :data-value="sample?.[axis.key] ?? ''" :data-available="!!sample">{{ format(sample?.[axis.key], 3) }}</td><td class="subtle">{{ axis.range }}</td></tr>
          </tbody></table>
          <div class="gamepad-buttons"><div v-for="button in buttons" :key="button.key" :data-testid="'gamepad-button-' + button.key" :data-value="sample?.[button.key] ?? ''" :data-available="!!sample" :class="{ pressed: sample?.[button.key] }"><strong>{{ button.label }}</strong><span>{{ !sample ? '—' : sample[button.key] ? '按住' : '松开' }}</span></div></div>
        </div>
        <p v-if="slot?.status === 'disconnected'" class="gamepad-panel-note">将手柄连接到运行核心的电脑。</p>
      </section>
      <section class="panel gamepad-intent-panel" data-testid="gamepad-intent-panel">
        <header class="panel-heading"><h2>映射目标</h2></header>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table"><thead><tr><th>目标量</th><th class="numeric">当前值</th><th>单位</th></tr></thead><tbody>
          <tr v-for="target in targets" :key="target.key"><td>{{ target.label }}</td><td class="numeric mono" :class="{ unavailable: !intent }" :data-testid="'gamepad-intent-' + target.key" :data-value="intent?.[target.key] ?? ''" :data-available="!!intent">{{ format(intent?.[target.key], target.unit === 'RPM' ? 0 : 1) }}</td><td class="subtle">{{ target.unit }}</td></tr>
          <tr><td>B 回中 / X 退出</td><td class="numeric" data-testid="gamepad-intent-rudder_center" :data-value="intent?.rudder_center ?? ''" :data-available="!!intent">{{ !intent || !sample ? '—' : (intent.rudder_center ? '回中' : '无') + ' / ' + (sample.button_x ? '退出' : '无') }}</td><td class="subtle">—</td></tr>
        </tbody></table></div>
      </section>
      <details class="panel gamepad-reference gamepad-config-panel" data-testid="gamepad-config-panel">
        <summary>映射参数</summary>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table"><tbody><tr v-for="setting in settings" :key="setting[0]"><th>{{ setting[0] }}</th><td class="numeric mono">{{ setting[1] }}</td></tr></tbody></table></div>
      </details>
      <details class="panel gamepad-reference gamepad-mapping-panel" data-testid="gamepad-mapping-panel">
        <summary>按键映射</summary>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table gamepad-mapping-table"><tbody><tr v-for="mapping in mappings" :key="mapping[0]"><th>{{ mapping[0] }}</th><td>{{ mapping[1] }}</td></tr></tbody></table></div>
      </details>
    </div>
  </section>
</template>

<style scoped>
.gamepad-toolbar strong { font-size: 14px; }
.gamepad-state-strip { font-size: 12px; }
.gamepad-slots button { font-size: 12px; }
.gamepad-grid { grid-template-rows: minmax(0, 1fr) auto; }
.panel-heading h2 { font-size: 14px; }
.panel-meta { font-size: 12px; }
.gamepad-table { font-size: 13px; }
.gamepad-table th { font-size: 12px; }
.gamepad-table th, .gamepad-table td { padding: 10px 12px; }
.gamepad-buttons { padding: 12px; }
.gamepad-buttons strong, .gamepad-buttons span { font-size: 12px; }
.gamepad-panel-note { font-size: 12px; }
/* 参考内容按需展开，滚动限制在详情内。 */
.gamepad-reference { display: block; align-self: start; }
.gamepad-reference summary { padding: 11px 12px; cursor: pointer; color: var(--text); font-size: 13px; }
.gamepad-reference summary:focus-visible { outline: 2px solid var(--blue); outline-offset: -2px; }
.gamepad-reference[open] summary { border-bottom: 1px solid var(--line); }
.gamepad-reference .gamepad-table-scroll { max-height: min(230px, 28vh); }
@media (max-height: 800px) {
  .gamepad-table th, .gamepad-table td { padding-block: 7px; }
  .gamepad-reference .gamepad-table-scroll { max-height: 180px; }
  .gamepad-buttons { padding-block: 9px; }
}
</style>
