export type DeviceLinkStatus =
  | 'stopped'
  | 'waiting'
  | 'connected'
  | 'disconnected'
  | 'error'

export interface BackendInfo {
  name: string
  version: string
  mode: string
  started_at: string
}

export interface DeviceLinkState {
  transport: string
  status: DeviceLinkStatus
  peer: string | null
  session_id: string | null
  connected_at: string | null
  last_rx_at: string | null
  last_tx_at: string | null
  last_error: string | null
}

export interface TelemetrySample {
  source: string
  sampled_at: string | null
  received_at: string
  device_tick_ms: number | null
  session_id: string | null
  age_ms: number
  stale_after_ms: number
  fresh: boolean
  invalid_fields?: Record<string, string>
  quality?: { status: 'valid' | 'invalid' | 'unknown'; reasons: string[] }
  data: Record<string, unknown>
}

export interface CapabilityState {
  observe: boolean
  realtime_control: boolean
  parameter_write: boolean
  autonomous_mission: boolean
  intermittent_command: boolean
}

export interface StationState {
  generation: number
  overall_fresh: boolean
  stale_reason: string | null
  stale_topics: string[]
  required_topics?: string[]
  missing_topics?: string[]
  command_capability: 'disabled'
  capabilities: CapabilityState
}

export interface GroundStationSnapshot {
  schema_version: number
  generated_at: string
  backend: BackendInfo
  device_link: DeviceLinkState
  telemetry: Record<string, TelemetrySample>
  state: StationState
  // 独立输入源，不受板端连接状态约束；旧核心允许没有此字段。
  gamepad?: GamepadPreview | null
}

export interface GamepadConfig {
  deadzone: number
  poll_period_s: number
  send_period_s: number
  input_timeout_s: number
  telemetry_timeout_s: number
  prepare_timeout_s: number
  max_percent: number
  pitch_rpm: number
  motor1_is_left: boolean
  motor1_forward_dir: number
  motor2_forward_dir: number
  roll_rpm: number
  trigger_deadzone: number
  rudder_rate_deg_s: number
}

export interface GamepadInputSample {
  left_x: number
  forward: number
  left_trigger: number
  right_trigger: number
  button_x: boolean
  button_y: boolean
  button_a: boolean
  button_b: boolean
  dpad_left: boolean
  dpad_right: boolean
  exit_seq: number
  loss_seq: number
  rudder_center_seq: number
}

export interface GamepadIntent {
  left_percent: number
  right_percent: number
  pitch_enable: number
  pitch_rpm: number
  roll_enable: number
  roll_rpm: number
  rudder_rate_deg_s: number
  rudder_center: boolean
}

export type GamepadSlotStatus = 'fresh' | 'stale' | 'disconnected' | 'unavailable' | 'waiting' | 'invalid'
export interface GamepadSlot {
  index: number
  connected: boolean
  fresh: boolean
  age_ms: number | null
  status: GamepadSlotStatus
  reason: string | null
  sample: GamepadInputSample | null
  neutral: boolean | null
  intent: GamepadIntent | null
}

export interface GamepadPreview {
  schema_version: 1
  mode: 'preview_only'
  source: 'core_xinput'
  status: 'running' | 'disabled' | 'stopped' | 'error'
  error: string | null
  config: GamepadConfig
  slots: GamepadSlot[]
}

export interface SnapshotEvent {
  type: 'state.snapshot'
  data: GroundStationSnapshot
}

export interface HeartbeatEvent {
  type: 'heartbeat'
  generated_at: string
  generation: number
  data: GroundStationSnapshot
}

export interface ErrorEvent {
  type: 'error'
  message: string
}

export type StationEvent = SnapshotEvent | HeartbeatEvent | ErrorEvent
export type CoreConnectionStatus = 'connecting' | 'live' | 'reconnecting' | 'offline'
