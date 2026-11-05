import { computed, onMounted, onUnmounted, ref, shallowRef, watch } from 'vue'
import type { CoreConnectionStatus, GamepadPreview, GroundStationSnapshot, TelemetrySample } from '../types'
import { ageGamepadPreview, parseGamepadPreview } from '../domain/gamepad'

const HEARTBEAT_TIMEOUT_MS = 3_500
const REQUEST_TIMEOUT_MS = 3_000
const LINK_STATUSES = ['stopped', 'waiting', 'connected', 'disconnected', 'error']
const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && !Array.isArray(value)
const isFiniteNumber = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
const isStrings = (value: unknown): value is string[] => Array.isArray(value) && value.every(item => typeof item === 'string')
const isNullableString = (value: unknown) => value === null || typeof value === 'string'

// 网络输入逐层验证；不能把一个缺少子结构的对象断言成完整快照。
export function isSnapshot(value: unknown): value is GroundStationSnapshot {
  if (!isRecord(value) || value.schema_version !== 1 || typeof value.generated_at !== 'string') return false
  const { backend, device_link: link, telemetry, state } = value
  if (!isRecord(backend) || !['name', 'version', 'mode', 'started_at'].every(key => typeof backend[key] === 'string')) return false
  if (!isRecord(link) || !LINK_STATUSES.includes(String(link.status)) || typeof link.transport !== 'string') return false
  if (!['peer', 'session_id', 'connected_at', 'last_rx_at', 'last_tx_at', 'last_error'].every(key => isNullableString(link[key]))) return false
  if (!isRecord(state) || !isFiniteNumber(state.generation) || typeof state.overall_fresh !== 'boolean' || !isNullableString(state.stale_reason)) return false
  if (!isStrings(state.stale_topics) || !isRecord(state.capabilities) || state.command_capability !== 'disabled') return false
  if (state.required_topics !== undefined && !isStrings(state.required_topics)) return false
  if (state.missing_topics !== undefined && !isStrings(state.missing_topics)) return false
  if (!isRecord(telemetry)) return false
  return Object.values(telemetry).every(sample => {
    if (!isRecord(sample) || !isRecord(sample.data) || typeof sample.source !== 'string' || typeof sample.received_at !== 'string') return false
    if (!isFiniteNumber(sample.age_ms) || sample.age_ms < 0 || !isFiniteNumber(sample.stale_after_ms) || sample.stale_after_ms <= 0 || typeof sample.fresh !== 'boolean') return false
    if (!isNullableString(sample.session_id) || !isNullableString(sample.sampled_at)) return false
    if (sample.device_tick_ms !== null && !isFiniteNumber(sample.device_tick_ms)) return false
    if (sample.invalid_fields !== undefined && (!isRecord(sample.invalid_fields) || !Object.values(sample.invalid_fields).every(reason => typeof reason === 'string'))) return false
    if (sample.quality !== undefined && (!isRecord(sample.quality) || !['valid', 'invalid', 'unknown'].includes(String(sample.quality.status)) || !isStrings(sample.quality.reasons))) return false
    return true
  })
}

export interface StationLogEvent { id: number; time: string; text: string; tone: string }

// 只在 App 创建一次；所有面板接收同一份状态，不各自打开 WebSocket。
export function useStationState() {
  const snapshot = shallowRef<GroundStationSnapshot | null>(null)
  const gamepadSnapshot = shallowRef<GamepadPreview | null>(null)
  const gamepadParseIssue = ref<string | null>(null)
  const coreConnection = ref<CoreConnectionStatus>('connecting')
  const lastEventAt = ref<string | null>(null)
  const errorMessage = ref<string | null>(null)
  const now = ref(performance.now())
  const acceptedAt = ref<number | null>(null)
  const events = ref<StationLogEvent[]>([])
  let eventId = 0
  let socket: WebSocket | null = null
  let reconnectTimer: number | null = null
  let clockTimer: number | null = null
  let reconnectAttempt = 0
  let disposed = false
  let initialRequest: AbortController | null = null
  let socketStartedAt = performance.now()

  const coreLive = computed(() => coreConnection.value === 'live' && acceptedAt.value !== null && now.value - acceptedAt.value < HEARTBEAT_TIMEOUT_MS)
  const gamepad = computed(() => ageGamepadPreview(gamepadSnapshot.value, acceptedAt.value === null ? 0 : Math.max(0, now.value - acceptedAt.value), coreLive.value))
  const gamepadIssue = computed(() => !coreLive.value ? '核心未在线，无法确认手柄输入' : gamepadParseIssue.value ?? gamepad.value?.error ?? null)
  const deviceStatus = computed(() => coreLive.value ? snapshot.value?.device_link.status ?? 'unknown' : 'unknown')
  const telemetry = computed<Record<string, TelemetrySample>>(() => {
    const elapsed = acceptedAt.value === null ? 0 : Math.max(0, now.value - acceptedAt.value)
    return Object.fromEntries(Object.entries(snapshot.value?.telemetry ?? {}).map(([topic, sample]) => {
      const age = sample.age_ms + elapsed
      return [topic, { ...sample, age_ms: Math.round(age), fresh: coreLive.value && deviceStatus.value === 'connected' && sample.fresh && age <= sample.stale_after_ms }]
    }))
  })
  const requiredTopics = computed(() => snapshot.value?.state.required_topics ?? ['ugekf', 'ugctl', 'ugwrm'])
  const missingTopics = computed(() => requiredTopics.value.filter(topic => !telemetry.value[topic]))
  const allFresh = computed(() => coreLive.value && deviceStatus.value === 'connected' && missingTopics.value.length === 0 && requiredTopics.value.every(topic => telemetry.value[topic]?.fresh))

  function recordEvent(text: string, tone = 'muted') {
    events.value = [{ id: ++eventId, time: new Date().toLocaleTimeString('zh-CN', { hour12: false }), text, tone }, ...events.value].slice(0, 80)
  }
  watch(coreConnection, value => recordEvent({ connecting: '连接地面站核心', live: '状态推送已建立', reconnecting: '核心连接中断，正在重连', offline: '地面站核心离线' }[value], value === 'live' ? 'ok' : 'warn'))
  watch(deviceStatus, value => recordEvent({ stopped: '设备监听未启动', waiting: '等待滑翔机接入', connected: '滑翔机已连接', disconnected: '滑翔机已断开', error: '设备链路异常', unknown: '设备状态不可确认' }[value], value === 'connected' ? 'ok' : 'warn'))
  watch(errorMessage, value => { if (value) recordEvent(value, 'bad') })

  function acceptSnapshot(value: unknown): void {
    if (!isSnapshot(value)) throw new Error('收到不兼容的状态数据')
    // 手柄不通过板端 fresh 门禁；旧核心或坏字段只清除此输入源。
    gamepadSnapshot.value = parseGamepadPreview(value.gamepad)
    gamepadParseIssue.value = gamepadSnapshot.value ? null : value.gamepad === undefined ? '当前核心未提供手柄预览，请更新核心' : '手柄预览格式无效，已清除输入意图'
    snapshot.value = value
    acceptedAt.value = performance.now()
    now.value = acceptedAt.value
    lastEventAt.value = new Date().toISOString()
  }
  async function fetchInitialState() {
    const controller = new AbortController()
    initialRequest = controller
    const timer = window.setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS)
    try {
      const response = await fetch('/api/v1/state', { signal: controller.signal, cache: 'no-store', headers: { Accept: 'application/json' } })
      if (!response.ok) throw new Error('状态接口 HTTP ' + response.status)
      const data: unknown = await response.json()
      // 较慢的初始 REST 不能覆盖已经抵达的新推送。
      if (!disposed && snapshot.value === null) acceptSnapshot(data)
    } catch (error) {
      if (!disposed && !coreLive.value) errorMessage.value = error instanceof Error && error.name === 'AbortError' ? '读取初始状态超时' : '初始状态暂不可用'
    } finally { window.clearTimeout(timer); initialRequest = null }
  }
  function scheduleReconnect() {
    if (disposed || reconnectTimer !== null) return
    coreConnection.value = snapshot.value ? 'reconnecting' : 'offline'
    reconnectTimer = window.setTimeout(() => { reconnectTimer = null; connectWebSocket() }, Math.min(500 * 2 ** reconnectAttempt++, 10_000))
  }
  function connectWebSocket() {
    if (disposed) return
    coreConnection.value = snapshot.value ? 'reconnecting' : 'connecting'
    socketStartedAt = performance.now()
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const connection = new WebSocket(protocol + '//' + window.location.host + '/api/v1/ws/state')
    socket = connection
    connection.addEventListener('message', event => {
      if (socket !== connection || disposed) return
      try {
        const message: unknown = JSON.parse(String(event.data))
        if (!isRecord(message)) throw new Error('状态消息格式错误')
        if (message.type === 'state.snapshot' || message.type === 'heartbeat') {
          acceptSnapshot(message.data)
          reconnectAttempt = 0
          errorMessage.value = null
          coreConnection.value = 'live'
        } else if (message.type === 'error' && typeof message.message === 'string') {
          gamepadSnapshot.value = null
          gamepadParseIssue.value = '核心报告异常，手柄输入不可确认'
          errorMessage.value = message.message
        }
      } catch (error) {
        gamepadSnapshot.value = null
        gamepadParseIssue.value = '状态消息无效，已清除手柄输入意图'
        errorMessage.value = error instanceof Error ? error.message : '无法解析状态消息'
      }
    })
    connection.addEventListener('error', () => {
      if (socket === connection && !disposed) {
        gamepadSnapshot.value = null
        gamepadParseIssue.value = '状态推送连接异常，手柄输入不可确认'
        errorMessage.value = '状态推送连接异常'
      }
    })
    connection.addEventListener('close', () => { if (socket === connection) { socket = null; scheduleReconnect() } })
  }
  onMounted(() => {
    recordEvent('工作台已打开；等待真实遥测')
    void fetchInitialState()
    connectWebSocket()
    clockTimer = window.setInterval(() => {
      now.value = performance.now()
      const lastActivity = Math.max(socketStartedAt, acceptedAt.value ?? socketStartedAt)
      if (socket && now.value - lastActivity >= HEARTBEAT_TIMEOUT_MS) {
        errorMessage.value = '状态推送超时，设备状态不可确认'
        const expired = socket
        socket = null
        expired.close()
        scheduleReconnect()
      }
    }, 50)
  })
  onUnmounted(() => {
    disposed = true
    initialRequest?.abort()
    if (reconnectTimer !== null) window.clearTimeout(reconnectTimer)
    if (clockTimer !== null) window.clearInterval(clockTimer)
    socket?.close()
  })
  return { snapshot, coreConnection, coreLive, deviceStatus, lastEventAt, errorMessage, telemetry, allFresh, missingTopics, events, now, gamepad, gamepadIssue }
}
