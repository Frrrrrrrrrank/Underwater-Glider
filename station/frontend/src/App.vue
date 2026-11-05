<script setup lang="ts">
import { computed, ref } from 'vue'
import { useStationState } from './composables/useStationState'
import { definition, fieldIssue, formatField, topics, type Topic } from './domain/fields'
import DataValue from './components/DataValue.vue'
import TrendPanel from './components/TrendPanel.vue'
import DiagnosticView from './components/DiagnosticView.vue'
import RecordingView from './components/RecordingView.vue'
import HomeView from './components/HomeView.vue'
import GamepadView from './components/GamepadView.vue'

const { snapshot, coreConnection, coreLive, deviceStatus, lastEventAt, errorMessage, telemetry, allFresh, missingTopics, events, now, gamepad, gamepadIssue } = useStationState()
type Page = 'home' | 'gamepad' | 'monitor' | 'diagnostic' | 'recording'
const page = ref<Page>('home')
// 仅保存本浏览器的预览选择，不请求接管，也不因插拔自动选其它槽位。
const selectedGamepadSlot = ref(0)
const pages: { key: Page; label: string; description: string }[] = [
  { key: 'home', label: '主页', description: '设备概览 · 工作台入口' },
  { key: 'gamepad', label: '手柄输入', description: '核心 XInput · 采样与基础映射预览' },
  { key: 'monitor', label: '运行监控', description: '台架观测 · 姿态 / 深度 / 执行反馈' },
  { key: 'diagnostic', label: '遥测诊断', description: '按协议主题检查字段与有效性' },
  { key: 'recording', label: '会话记录', description: '按设备会话保存遥测与连接事件' },
]
const coreLabel = computed(() => coreLive.value ? '核心在线' : { connecting: '核心连接中', live: '核心推送超时', reconnecting: '核心重连中', offline: '核心离线' }[coreConnection.value])
const deviceLabel = computed(() => ({ unknown: '设备状态未知', stopped: '设备监听未启动', waiting: '等待设备', connected: '设备已连接', disconnected: '设备已断开', error: '设备链路异常' }[deviceStatus.value]))
const validity = computed(() => {
  if (fieldIssue(telemetry.value.ugekf, 'ugekf', 'fl')) return '估计就绪未知'
  return (Number(telemetry.value.ugekf?.data.fl) & 2) !== 0 ? '估计可用于控制' : '估计尚未就绪'
})
const dataSummary = computed(() => {
  if (!coreLive.value) return '连接恢复后核实设备与数据状态'
  if (deviceStatus.value !== 'connected') return deviceLabel.value
  if (missingTopics.value.length) return '待接收 ' + missingTopics.value.map(topic => topic.toUpperCase()).join(' / ')
  if (!allFresh.value) return '部分遥测过期，请检查数据流'
  const invalid = Object.values(telemetry.value).some(sample => sample.quality?.status === 'invalid' || Object.keys(sample.invalid_fields ?? {}).length)
  return invalid ? '遥测已更新 · 部分字段不可用，详见诊断' : '遥测持续更新'
})
const kpis = [{ key: 'depth', label: '深度', unit: 'm' }, { key: 'pitch_deg', label: '俯仰', unit: '°' }, { key: 'roll_deg', label: '横滚', unit: '°' }, { key: 'yaw_deg', label: '航向', unit: '°' }, { key: 'vD', label: '下向速度', unit: 'm/s' }]
const controls = [
  { label: '深度', target: 'target_depth', feedback: 'depth', error: 'depth_error', unit: 'm' },
  { label: '俯仰', target: 'pitch_sp_deg', feedback: 'pitch_deg', error: 'pitch_error_deg', unit: '°' },
  { label: '油囊位移', target: 'target_displacement_mm', feedback: 'displacement_mm', error: 'displacement_error_mm', unit: 'mm' },
]
const actuatorRows: { topic: Topic; keys: string[] }[] = [
  { topic: 'ugwrm', keys: ['stroke_percent', 'roll_percent'] }, { topic: 'ugwrm', keys: ['cmd_rpm', 'fb_rpm'] },
  { topic: 'ugwrm', keys: ['home_state', 'roll_home_state'] }, { topic: 'ugctl', keys: ['pump_enable', 'pump_direction'] },
  { topic: 'ugwrm', keys: ['calib_state', 'calib_hold'] },
  { topic: 'ugctl', keys: ['worm_rpm', 'thrust_pct'] },
]
const controlFields = ['mode', 'armed', 'output_valid', 'fault', 'core_fault', 'pid_valid']
function valueLabel(topic: Topic, key: string): string {
  return fieldIssue(telemetry.value[topic], topic, key) ? '—' : formatField(topic, key, telemetry.value[topic]?.data[key])
}
function timeLabel(value: string | null | undefined) {
  if (!value) return '—'
  return new Date(value).toLocaleTimeString('zh-CN', { hour12: false })
}
</script>
<template>
  <div class="app-shell" data-testid="workbench" :data-core-live="coreLive">
    <header class="topbar">
      <div class="brand" title="UnderwaterRobotGroundControl"><img src="/Redbird.png?v=2" alt="Redbird" /><div><h1>URGC</h1><span>UnderwaterRobotGroundControl</span></div></div>
      <div class="topbar-status">
        <span class="status-label" :class="coreLive ? 'tone-ok' : 'tone-warn'" data-testid="core-status"><i></i>{{ coreLabel }}</span>
        <span class="status-label" :class="deviceStatus === 'connected' ? 'tone-ok' : 'tone-warn'" data-testid="device-status"><i></i>{{ deviceLabel }}</span>
        <span class="status-label" :class="allFresh ? 'tone-ok' : 'tone-muted'" data-testid="freshness-status"><i></i>{{ allFresh ? '遥测新鲜' : '遥测未就绪' }}</span>
        <span class="readonly-label">只读观测</span>
      </div>
    </header>
    <div class="workspace">
      <aside class="navigation"><div class="workspace-label">工作台侧栏</div><nav aria-label="任务视图"><button v-for="item in pages" :key="item.key" :class="{ active: page === item.key }" :aria-current="page === item.key ? 'page' : undefined" @click="page = item.key">{{ item.label }}</button></nav><div class="nav-footer"><span>ETHERNET</span><strong>本机核心</strong><small>v{{ snapshot?.backend.version ?? '—' }}</small></div></aside>
      <main class="main-workspace">
        <div class="workspace-heading"><div><h2>{{ pages.find(item => item.key === page)?.label }}</h2><span>{{ pages.find(item => item.key === page)?.description }}</span></div><span class="time-label">最近推送 {{ timeLabel(lastEventAt) }}</span></div>
        <div class="notice-bar" :class="{ 'notice-warning': !allFresh || !!errorMessage }" role="status"><span class="notice-dot"></span><span>{{ errorMessage ?? snapshot?.device_link.last_error ?? dataSummary }}</span><span class="notice-validity">{{ validity }}</span></div>
        <HomeView v-if="page === 'home'" :telemetry="telemetry" :core-live="coreLive" :device-status="deviceStatus" :gamepad="gamepad" :gamepad-issue="gamepadIssue" :gamepad-slot="selectedGamepadSlot" @navigate="page = $event" />
        <GamepadView v-if="page === 'gamepad'" :gamepad="gamepad" :issue="gamepadIssue" :selected-slot="selectedGamepadSlot" @select-slot="selectedGamepadSlot = $event" />
        <div v-show="page === 'monitor'" class="monitor-grid">
          <section class="panel measurements-panel">
            <header class="panel-heading"><h2>姿态与深度</h2><span class="panel-meta">估计器 <DataValue :sample="telemetry.ugekf" topic="ugekf" field="status" /></span></header>
            <div class="kpi-strip"><div v-for="item in kpis" :key="item.key" class="kpi" :data-testid="'metric-' + item.key"><span>{{ item.label }}</span><strong><DataValue :sample="telemetry.ugekf" topic="ugekf" :field="item.key" /><small>{{ item.unit }}</small></strong></div></div>
            <div class="section-caption"><span>控制目标与反馈</span><span>同一 UGCTL 快照 · 误差为板端报告</span></div>
            <div class="panel-scroll"><table class="data-table comparison-table"><thead><tr><th>量</th><th class="numeric">目标</th><th class="numeric">反馈</th><th class="numeric">误差</th><th>单位</th></tr></thead><tbody><tr v-for="item in controls" :key="item.label"><td>{{ item.label }}</td><td class="numeric"><DataValue :sample="telemetry.ugctl" topic="ugctl" :field="item.target" /></td><td class="numeric"><DataValue :sample="telemetry.ugctl" topic="ugctl" :field="item.feedback" /></td><td class="numeric"><DataValue :sample="telemetry.ugctl" topic="ugctl" :field="item.error" /></td><td class="subtle">{{ item.unit }}</td></tr></tbody></table></div>
            <div class="panel-note">深度向下为正；目标值表示板端当前快照，不代表从本页发出了指令。</div>
          </section>
          <section class="panel actuator-panel">
            <header class="panel-heading"><h2>双轴与浮力系统</h2><span class="panel-meta">pitch 参考 <DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="span_valid" /> · 反馈新鲜 <DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="data_fresh" /></span></header>
            <div class="panel-scroll"><table class="data-table paired-table"><tbody><tr v-for="(row, index) in actuatorRows" :key="index"><template v-for="key in row.keys" :key="key"><th>{{ definition(row.topic, key).label }}<small v-if="definition(row.topic, key).unit"> {{ definition(row.topic, key).unit }}</small></th><td class="numeric"><DataValue :sample="telemetry[row.topic]" :topic="row.topic" :field="key" /></td></template></tr></tbody></table>
            <div class="worm-track"><span>前端 0%</span><div><i v-if="!fieldIssue(telemetry.ugwrm, 'ugwrm', 'stroke_percent')" :style="{ left: Math.max(0, Math.min(100, Number(telemetry.ugwrm?.data.stroke_percent))) + '%' }"></i></div><span>后端 100%</span></div>
            <div class="worm-track" data-testid="roll-track"><span>PB3 0%</span><div><i v-if="!fieldIssue(telemetry.ugwrm, 'ugwrm', 'roll_percent')" :style="{ left: Math.max(0, Math.min(100, Number(telemetry.ugwrm?.data.roll_percent))) + '%' }"></i></div><span>PB5 100%</span></div>
            <p class="roll-note">roll 为脉冲估算；50% 是行程中点，不代表艇体水平。</p>
            </div>
          </section>
          <TrendPanel :estimate="telemetry.ugekf" :control="telemetry.ugctl" :now="now" :active="!!telemetry.ugekf?.fresh" />
          <section class="panel control-panel"><header class="panel-heading"><h2>控制器状态</h2><span class="panel-meta">UGCTL</span></header><div class="panel-scroll"><dl class="compact-details"><div v-for="key in controlFields" :key="key"><dt>{{ definition('ugctl', key).label }}</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" :field="key" /></dd></div></dl><p class="panel-note">控制输出有效只描述控制器状态；泵和推进量是输出命令，不是执行器实测。</p></div></section>
          <section class="panel link-panel"><header class="panel-heading"><h2 :title="snapshot?.device_link.peer ?? '未连接'">链路与数据流</h2><span class="panel-meta">相位 {{ valueLabel('ugctl', 'phase') }} · 接收 {{ timeLabel(snapshot?.device_link.last_rx_at) }}</span></header><div class="panel-scroll"><table class="data-table"><thead><tr><th>数据流</th><th>接收状态</th><th class="numeric">年龄</th><th>字段质量</th></tr></thead><tbody><tr v-for="topic in topics" :key="topic.key"><td>{{ topic.label }}</td><td :class="telemetry[topic.key]?.fresh ? 'tone-ok' : 'tone-warn'">{{ !coreLive ? '不可确认' : !telemetry[topic.key] ? '未收到' : telemetry[topic.key].fresh ? '新鲜' : '过期' }}</td><td class="numeric mono">{{ telemetry[topic.key] ? telemetry[topic.key].age_ms + ' ms' : '—' }}</td><td>{{ !telemetry[topic.key] ? '—' : telemetry[topic.key].quality?.status === 'invalid' ? '部分不可用' : telemetry[topic.key].quality?.status === 'valid' ? '有效' : '待判定' }}</td></tr></tbody></table></div></section>
          <section class="panel event-panel"><header class="panel-heading"><h2>连接事件</h2><span class="panel-meta">当前浏览器 · 最近 {{ events.length }} 条</span></header><ol class="panel-scroll event-list"><li v-for="event in events" :key="event.id"><time>{{ event.time }}</time><span :class="'tone-' + event.tone">{{ event.text }}</span></li></ol></section>
        </div>
        <DiagnosticView v-if="page === 'diagnostic'" :telemetry="telemetry" :snapshot="snapshot" :core-live="coreLive" />
        <RecordingView v-if="page === 'recording'" :core-live="coreLive" />
      </main>
    </div>
    <footer class="statusbar"><span>URGC · 网线台架观测</span><span>— 表示未收到、已过期或字段无效；在遥测诊断中查看原因。</span><span class="mono">{{ coreLive ? 'LIVE FEED' : 'AWAITING CONNECTION' }}</span></footer>
  </div>
</template>
