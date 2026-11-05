<script setup lang="ts">
import { computed } from 'vue'
import type { GamepadInputSample, GamepadIntent, GamepadPreview } from '../types'
import { gamepadStatusLabel } from '../domain/gamepad'

const props = defineProps<{ gamepad: GamepadPreview | null; issue: string | null; selectedSlot: number }>()
const emit = defineEmits<{ selectSlot: [index: number] }>()
const slot = computed(() => props.gamepad?.slots.find(item => item.index === props.selectedSlot))
const sample = computed(() => slot.value?.fresh ? slot.value.sample : null)
const intent = computed(() => slot.value?.fresh ? slot.value.intent : null)
const serviceLabel = computed(() => props.gamepad ? { running: '采样运行', disabled: '采样已禁用', stopped: '采样已停止', error: '采样异常' }[props.gamepad.status] : '采样状态未知')
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
  { key: 'pitch_rpm', label: 'Pitch 逻辑转速', unit: 'RPM' }, { key: 'roll_rpm', label: 'Roll 逻辑转速', unit: 'RPM' },
  { key: 'rudder_rate_deg_s', label: '尾舵目标变化率', unit: '°/s' },
]
const settings = computed(() => {
  const config = props.gamepad?.config
  return [
    ['摇杆死区', config ? (config.deadzone * 100).toFixed(0) + '%' : '—'],
    ['扳机死区', config ? (config.trigger_deadzone * 100).toFixed(0) + '%' : '—'],
    ['推进目标上限', config ? config.max_percent + '%' : '—'],
    ['Pitch / Roll 幅值', config ? config.pitch_rpm + ' / ' + config.roll_rpm + ' RPM' : '—'],
    ['尾舵目标变化率上限', config ? config.rudder_rate_deg_s + ' °/s' : '—'],
    ['输入超龄阈值 / 采样周期', config ? Math.round(config.input_timeout_s * 1000) + ' / ' + Math.round(config.poll_period_s * 1000) + ' ms' : '—'],
  ]
})
const mappings = [
  ['左摇杆', '前后双向推进；左右差动；仅横拨不原地转'],
  ['Y / A', '按住 Pitch 负 / 正转；松开或同按停止'],
  ['十字左 / 右', '按住 Roll 正 / 负转；松开或同按停止'],
  ['RT / LT', '尾舵目标正 / 负向改变；松开或双按保持目标'],
  ['B / X', 'B 回中意图；X 退出意图；本页均不执行'],
  ['泵 / 校准', '没有手柄映射；不分配空闲按键'],
]
function format(value: number | null | undefined, digits = 1) { return typeof value === 'number' ? value.toFixed(digits) : '—' }
// 页面只展示核心提供的基础映射，不重复混控、不积分舵角、不发控制请求。
</script>

<template>
  <section class="gamepad-view" data-testid="gamepad-view" :data-available="slot?.fresh === true">
    <div class="gamepad-toolbar">
      <div><strong>核心电脑的 XInput 手柄</strong><span>手柄须插在运行核心的 Windows 电脑；无需连接控制板。</span></div>
      <div class="gamepad-slots" role="group" aria-label="选择预览手柄槽位">
        <button v-for="index in [0, 1, 2, 3]" :key="index" :data-testid="'gamepad-slot-' + index" :class="{ selected: selectedSlot === index }" :aria-pressed="selectedSlot === index" @click="emit('selectSlot', index)">
          <i :class="gamepad?.slots.find(item => item.index === index)?.fresh ? 'tone-ok' : 'tone-muted'"></i>槽位 {{ index }}
        </button>
      </div>
    </div>
    <div class="gamepad-state-strip" role="status">
      <span data-testid="gamepad-service-status">{{ serviceLabel }}</span>
      <strong data-testid="gamepad-status" :class="slot?.fresh ? 'tone-ok' : 'tone-warn'">{{ slot ? gamepadStatusLabel[slot.status] : '输入不可用' }}</strong>
      <span data-testid="gamepad-age">样本年龄 {{ slot?.age_ms === null || slot?.age_ms === undefined ? '—' : slot.age_ms + ' ms' }}</span>
      <span>中立检查 {{ slot?.neutral === null || slot?.neutral === undefined ? '—' : slot.neutral ? '全部松开' : '输入偏离中立' }}</span>
      <span class="gamepad-state-detail">{{ issue ?? slot?.reason ?? '仅预览；选槽位不接管，也不自动切换到其它手柄。' }}</span>
    </div>
    <div class="gamepad-grid">
      <section class="panel gamepad-input-panel" data-testid="gamepad-input-panel">
        <header class="panel-heading"><h2>输入采样</h2><span class="panel-meta">核心归一化值 · 死区前</span></header>
        <div class="panel-scroll gamepad-table-scroll">
          <table class="data-table gamepad-table"><thead><tr><th>输入</th><th class="numeric">当前值</th><th>方向 / 范围</th></tr></thead><tbody>
            <tr v-for="axis in axes" :key="axis.key"><td>{{ axis.label }}</td><td class="numeric mono" :class="{ unavailable: !sample }" :data-testid="'gamepad-axis-' + axis.key" :data-value="sample?.[axis.key] ?? ''" :data-available="!!sample">{{ format(sample?.[axis.key], 3) }}</td><td class="subtle">{{ axis.range }}</td></tr>
          </tbody></table>
          <div class="gamepad-buttons"><div v-for="button in buttons" :key="button.key" :data-testid="'gamepad-button-' + button.key" :data-value="sample?.[button.key] ?? ''" :data-available="!!sample" :class="{ pressed: sample?.[button.key] }"><strong>{{ button.label }}</strong><span>{{ !sample ? '—' : sample[button.key] ? '按住' : '松开' }}</span></div></div>
        </div>
        <p class="gamepad-panel-note">未连接、超龄或核心失联时清空输入，不把未知显示为松开。</p>
      </section>
      <section class="panel gamepad-intent-panel" data-testid="gamepad-intent-panel">
        <header class="panel-heading"><h2>基础映射意图</h2><span class="panel-meta tone-warn">未发送 · 非执行器反馈</span></header>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table"><thead><tr><th>目标量</th><th class="numeric">当前意图</th><th>单位</th></tr></thead><tbody>
          <tr v-for="target in targets" :key="target.key"><td>{{ target.label }}</td><td class="numeric mono" :class="{ unavailable: !intent }" :data-testid="'gamepad-intent-' + target.key" :data-value="intent?.[target.key] ?? ''" :data-available="!!intent">{{ format(intent?.[target.key], target.unit === 'RPM' ? 0 : 1) }}</td><td class="subtle">{{ target.unit }}</td></tr>
          <tr><td>B 回中 / X 退出</td><td class="numeric" data-testid="gamepad-intent-rudder_center" :data-value="intent?.rudder_center ?? ''" :data-available="!!intent">{{ !intent || !sample ? '—' : (intent.rudder_center ? '回中意图' : '无回中') + ' / ' + (sample.button_x ? '退出意图' : '无退出') }}</td><td class="subtle">未执行</td></tr>
        </tbody></table></div>
        <p class="gamepad-panel-note">基础映射预览，接管 / 角度积分未启用；B 后等待全松等会话逻辑未运行。</p>
      </section>
      <section class="panel gamepad-config-panel" data-testid="gamepad-config-panel">
        <header class="panel-heading"><h2>当前核心配置</h2><span class="panel-meta">只读 · 复用 GamepadConfig</span></header>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table"><tbody><tr v-for="setting in settings" :key="setting[0]"><th>{{ setting[0] }}</th><td class="numeric mono">{{ setting[1] }}</td></tr></tbody></table></div>
        <p class="gamepad-panel-note">限幅仅描述控制目标；本页不能改配置，也不会创建控制会话。</p>
      </section>
      <section class="panel gamepad-mapping-panel" data-testid="gamepad-mapping-panel">
        <header class="panel-heading"><h2>沿用终端映射</h2><span class="panel-meta">auto_sti</span></header>
        <div class="panel-scroll gamepad-table-scroll"><table class="data-table gamepad-table gamepad-mapping-table"><tbody><tr v-for="mapping in mappings" :key="mapping[0]"><th>{{ mapping[0] }}</th><td>{{ mapping[1] }}</td></tr></tbody></table></div>
        <p class="gamepad-panel-note">尾舵松开是保持目标角，不是回中或断电；此处不显示虚构的实际舵角。</p>
      </section>
    </div>
  </section>
</template>
