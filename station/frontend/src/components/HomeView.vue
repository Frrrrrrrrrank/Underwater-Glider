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
const selected = ref('overview')
const actions = [
  { key: 'overview', title: '操控概览', subtitle: '对象与控制状态' },
  { key: 'direct', title: '直接遥控', subtitle: '推进 / 蜗杆 / 泵' },
  { key: 'pid', title: '闭环控制', subtitle: '俯仰 / 深度目标' },
  { key: 'homing', title: '双轴校准', subtitle: 'pitch / roll 参考与进度' },
  { key: 'maintenance', title: '参数与标定', subtitle: '调参与维护方案' },
]
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
const horizonTransform = computed(() => 'rotate(' + -(roll.value ?? 0) + ' 300 170) translate(0 ' + (pitch.value ?? 0) * 4 + ')')
const attitudeTicks = [-30, -20, -10, 0, 10, 20, 30]
const metrics = [
  { topic: 'ugekf' as const, key: 'depth', label: '深度', unit: 'm' },
  { topic: 'ugekf' as const, key: 'pitch_deg', label: '俯仰', unit: '°' },
  { topic: 'ugekf' as const, key: 'roll_deg', label: '横滚', unit: '°' },
  { topic: 'ugekf' as const, key: 'vD', label: '下向速度', unit: 'm/s' },
  { topic: 'ugwrm' as const, key: 'stroke_percent', label: '蜗杆行程', unit: '%' },
  { topic: 'ugwrm' as const, key: 'roll_percent', label: 'roll 位置估算', unit: '%' },
  { topic: 'ugctl' as const, key: 'displacement_mm', label: '油囊位移', unit: 'mm' },
]
// 本页只消费 App 的共享遥测；操作入口只切换说明面板，不创建任何写请求。
</script>

<template>
  <section class="home-view" data-testid="home-view">
    <div class="home-stage">
      <aside class="action-rail" aria-label="主页操作分组">
        <div class="rail-title">操作区 <span>布局预览</span></div>
        <button v-for="action in actions" :key="action.key" :data-home-action="action.key"
          :class="{ selected: selected === action.key }" :aria-pressed="selected === action.key" @click="selected = action.key">
          <strong>{{ action.title }}</strong><small>{{ action.subtitle }}</small>
        </button>
        <div class="rail-note">当前只读<br />点击分组查看方案<br />不会发送控制指令</div>
        <button class="monitor-link" @click="emit('navigate', 'monitor')">详细监控 <span>↗</span></button>
      </aside>

      <section class="situation-view" aria-label="机器人态势仪表">
        <header class="situation-heading"><div><span class="instrument-tag">HUD</span><h3>姿态与航向</h3></div><span>估计器反馈 · 非导航定位</span></header>
        <div class="instrument-layout">
          <div class="attitude" data-testid="home-attitude" :data-valid="poseValid">
            <div class="instrument-label"><span>姿态仪</span><span :class="poseValid ? 'tone-ok' : 'tone-warn'">{{ poseValid ? '实时估计' : '等待有效姿态' }}</span></div>
            <svg viewBox="0 0 600 360" role="img" aria-label="由有效估计驱动的横滚俯仰仪表">
              <defs><clipPath id="urgc-home-attitude-clip"><rect x="35" y="32" width="530" height="276" /></clipPath></defs>
              <g class="instrument-frame"><path d="M70 70V42H102 M498 42H530V70 M70 270V298H102 M498 298H530V270" /><path d="M300 32V43 M300 298V309" /></g>
              <g v-if="poseValid" clip-path="url(#urgc-home-attitude-clip)">
                <g :transform="horizonTransform" class="pitch-ladder">
                  <g v-for="angle in attitudeTicks" :key="angle" :transform="'translate(0 ' + -angle * 4 + ')'">
                    <path :d="angle === 0 ? 'M70 170H264 M336 170H530' : 'M232 170H278 M322 170H368'" :class="{ horizon: angle === 0 }" :stroke-dasharray="angle < 0 ? '5 5' : undefined" />
                    <template v-if="angle !== 0"><text x="216" y="174" text-anchor="end">{{ angle }}</text><text x="384" y="174">{{ angle }}</text></template>
                  </g>
                </g>
                <path class="vehicle-reticle" d="M232 170H275L287 182 M368 170H325L313 182 M300 157V183" />
                <circle class="vehicle-center" cx="300" cy="170" r="4" />
              </g>
              <g v-else class="instrument-empty"><path d="M262 170H338 M300 145V195" /><text x="300" y="224" text-anchor="middle">未收到有效估计，不显示默认水平姿态</text></g>
              <text class="axis-label" x="72" y="333">ROLL <tspan>{{ poseValid ? roll?.toFixed(1) + '°' : '—' }}</tspan></text>
              <text class="axis-label" x="528" y="333" text-anchor="end">PITCH <tspan>{{ poseValid ? pitch?.toFixed(1) + '°' : '—' }}</tspan></text>
            </svg>
          </div>
          <div class="heading-instrument">
            <div class="instrument-label"><span>航向估计</span><span>°</span></div>
            <svg viewBox="0 0 200 220" role="img" aria-label="估计航向罗盘">
              <circle cx="100" cy="96" r="69" class="compass-ring" /><circle cx="100" cy="96" r="51" class="compass-inner" />
              <path class="compass-ticks" d="M100 27V36 M169 96H160 M100 165V156 M31 96H40" />
              <text x="100" y="19" text-anchor="middle">N</text><text x="183" y="100" text-anchor="middle">E</text><text x="100" y="184" text-anchor="middle">S</text><text x="17" y="100" text-anchor="middle">W</text>
              <g v-if="headingValid" :transform="'rotate(' + heading + ' 100 96)'" data-testid="home-heading-pointer"><path class="heading-pointer" d="M100 42L111 108L100 101L89 108Z" /><path class="heading-tail" d="M100 101V135" /></g>
              <text v-else x="100" y="102" text-anchor="middle" class="compass-missing">—</text>
              <text x="100" y="216" text-anchor="middle" class="heading-number">{{ headingValid ? heading?.toFixed(1) + '°' : '—' }}</text>
            </svg>
          </div>
        </div>
        <footer class="situation-footer"><span>地图 / 视频：暂未接入</span><button @click="emit('navigate', 'diagnostic')">查看数据有效性 ↗</button></footer>
      </section>

      <aside class="operation-panel" aria-label="操作状态与接入方案">
        <header><h3>{{ actions.find(action => action.key === selected)?.title }}</h3><span>只读</span></header>
        <div class="operation-content panel-scroll">
          <template v-if="selected === 'overview'">
            <p class="section-eyebrow">当前适配对象</p><h4>Glider <small>水下滑翔机</small></h4>
            <dl class="home-details">
              <div><dt>控制模式</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="mode" /></dd></div>
              <div><dt>控制器使能</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="armed" /></dd></div>
              <div><dt>控制输出有效</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="output_valid" /></dd></div>
              <div><dt>服务状态</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="fault" /></dd></div>
              <div><dt>双轴校准</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_state" /></dd></div>
            </dl>
            <p class="operation-note">实际控制来源尚待系统快照接入，不能由网页选中的模式推断。</p>
            <div class="home-input-summary" data-testid="home-gamepad-summary">
              <span>输入预览 · 核心 XInput / 槽位 {{ gamepadSlot }}</span>
              <strong :class="inputSlot?.fresh ? 'tone-ok' : 'tone-muted'">{{ inputSlot ? gamepadStatusLabel[inputSlot.status] : '输入不可用' }}</strong>
              <small>{{ gamepadIssue ?? '未接管；输入新鲜不代表机器人已就绪。' }}</small>
              <button class="details-link" data-testid="home-gamepad-link" @click="emit('navigate', 'gamepad')">打开手柄输入 ↗</button>
            </div>
            <p class="access-note">控制服务未接入<br /><span>当前入口用于确认操作布局；不接管、不解锁、不驱动执行器。</span></p>
          </template>
          <template v-else-if="selected === 'direct'">
            <p class="section-eyebrow">建议操作：接管后低限幅遥控</p>
            <dl class="home-details">
              <div><dt>蜗杆反馈转速</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="fb_rpm" /> rpm</dd></div>
              <div><dt>蜗杆运动状态</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="moving" /></dd></div>
              <div><dt>油囊位移</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="displacement_mm" /> mm</dd></div>
            </dl>
            <ul class="operation-list"><li>推进：总推力、差动与单独停止。</li><li>蜗杆：前移 / 后移；已确认按住才动、松开即请求停止，网页控制尚未开放。</li><li>泵：两个方向与停泵，展示位移和联锁。</li><li>横滚步进轴：展开设置使能与转速。</li></ul>
            <p class="access-note">接管 / 交还遥控器需要独立确认<br /><span>旧 AUTO 是持续手动遥控，不是自主航行。停止输出和交还 SBUS 必须分开。</span></p>
          </template>
          <template v-else-if="selected === 'pid'">
            <p class="section-eyebrow">建议操作：选择模式 → 给定目标 → 确认启动</p>
            <dl class="home-details">
              <div><dt>俯仰给定</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="pitch_sp_deg" /> °</dd></div>
              <div><dt>目标深度</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="target_depth" /> m</dd></div>
              <div><dt>参数有效</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="pid_valid" /></dd></div>
              <div><dt>控制器使能</dt><dd><DataValue :sample="telemetry.ugctl" topic="ugctl" field="armed" /></dd></div>
            </dl>
            <ul class="operation-list"><li>主页保留定俯仰 / 深度串级及目标、反馈。</li><li>PID 参数和 P/I/D 曲线放调参视图。</li><li>目标编辑、发送请求、板端确认分开显示。</li></ul>
            <p class="access-note">启动闭环不等于选择模式<br /><span>先确认参数身份和估计有效，再允许 ARM；网页失联需停止核心续发。</span></p>
          </template>
          <template v-else-if="selected === 'homing'">
            <p class="section-eyebrow" :class="{ 'tone-warn': homingActive }">{{ homingActive ? '板端报告校准进行中' : '执行器准备 · 建立双轴参考' }}</p>
            <dl class="home-details">
              <div><dt>总状态</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_state" /></dd></div>
              <div><dt>总失败码</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_fail" /></dd></div>
              <div><dt>pitch 阶段</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="home_state" /></dd></div>
              <div><dt>pitch 原因</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="home_fail" /></dd></div>
              <div><dt>roll 阶段</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_home_state" /></dd></div>
              <div><dt>roll 原因</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_home_fail" /></dd></div>
              <div><dt>roll 位置估算</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="roll_percent" /> %</dd></div>
              <div><dt>校准控制保持</dt><dd><DataValue :sample="telemetry.ugwrm" topic="ugwrm" field="calib_hold" /></dd></div>
            </dl>
            <p class="homing-flow">pitch：前端释放点 → 0%<br />roll：PB3 0% → PB5 100% → 50%</p>
            <p class="access-note">两轴全部到达目标才报告完成<br /><span>roll 是脉冲估算，50% 不代表艇体水平。旧协议未提供 roll 和总状态时显示未知。</span></p>
            <p class="operation-note">网页只读；终端 calib 同时开始两轴，calibstop 取消两轴。结束后须中立重新接管；断开网页不代表校准停止。</p>
          </template>
          <template v-else>
            <p class="section-eyebrow">建议独立调参 / 维护视图</p>
            <ul class="operation-list"><li>PID 参数：编辑值、已确认值、应用结果。</li><li>磁标定：采集、拟合、质量检查、下发确认。</li><li>单执行器测试：限幅、现场检查与明确退出。</li><li>原始快照、遥测订阅归入诊断。</li></ul>
            <p class="access-note">校准与日常遥控分开<br /><span>不把所有会动执行器的命令铺成一排普通按钮。</span></p>
            <button class="details-link" @click="emit('navigate', 'diagnostic')">打开现有遥测诊断 ↗</button>
          </template>
        </div>
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
.home-view { min-height: 0; min-width: 0; display: grid; grid-template-rows: minmax(0, 1fr) 82px; gap: 10px; background: #000; }
.home-stage { min-height: 0; min-width: 0; display: grid; grid-template-columns: 160px minmax(0, 1fr) 270px; gap: 10px; }
.action-rail, .situation-view, .operation-panel { min-width: 0; min-height: 0; border: 1px solid #28252f; background: #000; }
.action-rail { display: flex; flex-direction: column; padding: 14px 0 0; }
.rail-title { margin: 0 13px 16px; font-size: 12px; }
.rail-title span { display: block; color: #918497; font-size: 10px; margin-top: 5px; }
.action-rail > button { text-align: left; min-height: 54px; background: #000; border: 0; border-block: 1px solid transparent; padding: 9px 13px; }
.action-rail > button strong { display: block; font-size: 12px; font-weight: 500; }
.action-rail > button small { display: block; color: #918c9d; margin-top: 5px; font-size: 10px; }
.action-rail > button:hover { background: #130f19; }
.action-rail > button.selected { background: #21172e; border-block-color: #49395d; color: #e4d3ff; }
.rail-note { color: #928a9d; font-size: 10px; line-height: 1.7; margin: 15px 13px; }
.action-rail > .monitor-link { margin-top: auto; border-top: 1px solid #28252f; min-height: 40px; font-size: 11px; }
.monitor-link span { float: right; color: #beaad8; }
.situation-view { display: flex; flex-direction: column; overflow: hidden; }
.situation-heading { display: flex; align-items: center; justify-content: space-between; padding: 13px 16px; gap: 10px; border-bottom: 1px solid #28252f; }
.situation-heading > div { display: flex; align-items: center; gap: 10px; }
.situation-heading h3, .operation-panel h3 { margin: 0; font-size: 12px; font-weight: 500; }
.instrument-tag { color: #cdb9ed; font: 10px Consolas, monospace; border: 1px solid #544063; padding: 3px 5px; }
.situation-heading > span { color: #8d8598; font-size: 10px; }
.instrument-layout { display: grid; grid-template-columns: minmax(0, 1fr) 142px; align-items: center; min-height: 0; flex: 1; padding: 18px 18px 12px; gap: 8px; }
.attitude, .heading-instrument { min-width: 0; min-height: 0; }
.instrument-label { display: flex; justify-content: space-between; gap: 10px; color: #9a94a3; font-size: 10px; padding: 0 6px 10px; }
.attitude svg { width: 100%; display: block; max-height: 320px; }
.instrument-frame, .pitch-ladder path { fill: none; stroke: #535262; stroke-width: 1.3; }
.pitch-ladder .horizon { stroke: #8d819f; stroke-width: 1.5; }
.pitch-ladder text { fill: #aaa3b5; font: 12px Consolas, monospace; }
.vehicle-reticle { stroke: #dfc7ff; fill: none; stroke-width: 3; }
.vehicle-center { fill: #dfc7ff; }
.instrument-empty path { fill: none; stroke: #4b4256; stroke-width: 1.3; }
.instrument-empty text { fill: #8e8599; font-size: 13px; }
.axis-label { fill: #91869f; font: 13px Consolas, monospace; }
.axis-label tspan { fill: #e1d7ed; }
.heading-instrument svg { width: 100%; display: block; max-height: 190px; }
.compass-ring, .compass-inner, .compass-ticks { stroke: #52455d; fill: none; stroke-width: 1.5; }
.compass-inner { stroke: #28222e; stroke-dasharray: 2 8; }
.heading-instrument text { fill: #aca1b7; font: 13px Consolas, monospace; }
.heading-pointer { fill: #cfb3f3; }
.heading-tail { stroke: #8a729e; stroke-width: 2; }
.heading-instrument .heading-number { font-size: 24px; fill: #e3d9ef; }
.heading-instrument .compass-missing { font-size: 25px; fill: #73657f; }
.situation-footer { display: flex; justify-content: space-between; gap: 8px; align-items: center; padding: 12px 15px; border-top: 1px solid #28252f; color: #887f92; font-size: 10px; }
.situation-footer button, .details-link { background: #000; border: 0; color: #bca4d7; font-size: 10px; padding: 0; }
.operation-panel { display: flex; flex-direction: column; overflow: hidden; }
.operation-panel > header { display: flex; justify-content: space-between; align-items: center; min-height: 45px; padding: 0 14px; border-bottom: 1px solid #28252f; }
.operation-panel > header > span { color: #a090af; font-size: 10px; }
.operation-content { flex: 1; padding: 14px; }
.section-eyebrow { font-size: 10px; color: #a38ab9; line-height: 1.6; }
.operation-content h4 { font-size: 20px; margin: 9px 0 14px; font-weight: 500; }
.operation-content h4 small { color: #a396af; font-size: 10px; margin-left: 5px; }
.home-details { margin-top: 8px; }
.home-details > div { display: flex; justify-content: space-between; gap: 10px; padding: 10px 0; border-bottom: 1px solid #25202b; }
.home-details dt { color: #a29aa9; font-size: 10px; }
.home-details dd { font-size: 11px; text-align: right; }
.operation-note, .operation-list { color: #a299ad; font-size: 10px; line-height: 1.8; margin-top: 13px; }
.operation-list { padding-left: 14px; }
.operation-list li + li { margin-top: 7px; }
.access-note { color: #d8bd8e; font-size: 11px; line-height: 1.7; border-top: 1px solid #3b3023; margin-top: 17px; padding-top: 11px; }
.access-note span { color: #a49684; font-size: 10px; }
.homing-flow { color: #b3a5c0; font-size: 10px; margin-top: 17px; line-height: 1.8; }
.details-link { margin-top: 20px; }
.home-input-summary { display: grid; gap: 5px; margin-top: 12px; padding-top: 10px; border-top: 1px solid #302737; }
.home-input-summary > span, .home-input-summary > small { color: #a299ad; font-size: 10px; line-height: 1.5; }
.home-input-summary > strong { font-size: 11px; font-weight: 500; }
.home-input-summary .details-link { margin-top: 3px; text-align: left; padding-block: 4px; }
.home-telemetry { min-width: 0; display: grid; grid-template-columns: repeat(7, minmax(0, 1fr)); border: 1px solid #302737; align-items: center; }
.home-metric { min-width: 0; padding: 0 16px; border-right: 1px solid #302737; }
.home-metric:last-child { border-right: 0; }
.home-metric > span { font-size: 10px; color: #a097ab; }
.home-metric strong { display: flex; align-items: baseline; gap: 6px; font: 24px Consolas, monospace; margin-top: 6px; }
.home-metric small { font: 10px 'Microsoft YaHei', sans-serif; color: #9b90a8; }
@media (max-width: 1450px) {
  .home-stage { grid-template-columns: 136px minmax(0, 1fr) 246px; gap: 8px; }
  .instrument-layout { grid-template-columns: minmax(0, 1fr); padding: 10px 18px; }
  .attitude svg { max-height: 215px; }
  .heading-instrument { display: flex; align-items: center; justify-content: center; gap: 18px; }
  .heading-instrument .instrument-label { flex-direction: column; }
  .heading-instrument svg { width: 100px; max-height: 110px; }
  .situation-heading { padding: 11px; }
  .situation-heading > span { max-width: 90px; line-height: 1.6; }
  .home-metric strong { font-size: 21px; }
}
@media (max-height: 800px) {
  .home-view { grid-template-rows: minmax(0, 1fr) 70px; gap: 8px; }
  .action-rail { padding-top: 10px; }
  .rail-title { margin-bottom: 10px; }
  .action-rail > button { min-height: 48px; padding-block: 7px; }
  .rail-note { margin-block: 10px; }
  .operation-content { padding: 11px 13px; }
  .home-details > div { padding-block: 8px; }
  .operation-content h4 { margin: 6px 0 10px; }
  .access-note { margin-top: 12px; padding-top: 9px; }
  .attitude svg { max-height: 190px; }
  .heading-instrument svg { max-height: 95px; }
  .situation-footer { padding: 10px; }
}
@media (max-width: 1150px) {
  .home-stage { grid-template-columns: 115px minmax(0, 1fr) 225px; }
  .action-rail > button { padding-inline: 9px; }
  .home-metric { padding-inline: 10px; }
  .home-metric strong { font-size: 18px; }
}
</style>
