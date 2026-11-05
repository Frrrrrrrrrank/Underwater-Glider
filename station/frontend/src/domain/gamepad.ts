import type { GamepadConfig, GamepadInputSample, GamepadIntent, GamepadPreview, GamepadSlotStatus } from '../types'

const record = (value: unknown): value is Record<string, unknown> => typeof value === 'object' && value !== null && !Array.isArray(value)
const number = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
const bounded = (value: unknown, min: number, max: number): value is number => number(value) && value >= min && value <= max
const integer = (value: unknown, min: number, max = Number.MAX_SAFE_INTEGER) => bounded(value, min, max) && Number.isSafeInteger(value)
const nullableString = (value: unknown) => value === null || typeof value === 'string'

function validConfig(value: unknown): value is GamepadConfig {
  if (!record(value)) return false
  if (!['deadzone', 'trigger_deadzone'].every(key => bounded(value[key], 0, 1) && value[key] !== 1)) return false
  if (!['poll_period_s', 'send_period_s', 'input_timeout_s', 'telemetry_timeout_s', 'prepare_timeout_s', 'rudder_rate_deg_s'].every(key => number(value[key]) && value[key] > 0)) return false
  return bounded(value.max_percent, 0, 100) && integer(value.pitch_rpm, 1, 32767) && integer(value.roll_rpm, 1, 32767)
    && typeof value.motor1_is_left === 'boolean' && integer(value.motor1_forward_dir, 0, 1) && integer(value.motor2_forward_dir, 0, 1)
}

function validSample(value: unknown): value is GamepadInputSample {
  if (!record(value)) return false
  return ['left_x', 'forward'].every(key => bounded(value[key], -1, 1))
    && ['left_trigger', 'right_trigger'].every(key => bounded(value[key], 0, 1))
    && ['button_x', 'button_y', 'button_a', 'button_b', 'dpad_left', 'dpad_right'].every(key => typeof value[key] === 'boolean')
    && ['exit_seq', 'loss_seq', 'rudder_center_seq'].every(key => integer(value[key], 0))
}

function validIntent(value: unknown, config: GamepadConfig): value is GamepadIntent {
  if (!record(value)) return false
  return ['left_percent', 'right_percent'].every(key => bounded(value[key], -config.max_percent, config.max_percent))
    && integer(value.pitch_enable, 0, 1) && integer(value.roll_enable, 0, 1)
    && integer(value.pitch_rpm, -config.pitch_rpm, config.pitch_rpm) && integer(value.roll_rpm, -config.roll_rpm, config.roll_rpm)
    && bounded(value.rudder_rate_deg_s, -config.rudder_rate_deg_s, config.rudder_rate_deg_s) && typeof value.rudder_center === 'boolean'
}

// 手柄协议独立校验；不兼容输入不能破坏遥测，也不能沿用上一帧意图。
export function parseGamepadPreview(value: unknown): GamepadPreview | null {
  if (!record(value) || value.schema_version !== 1 || value.mode !== 'preview_only' || value.source !== 'core_xinput') return null
  if (!['running', 'disabled', 'stopped', 'error'].includes(String(value.status)) || !nullableString(value.error) || !validConfig(value.config)) return null
  if (!Array.isArray(value.slots) || value.slots.length !== 4) return null
  const config = value.config
  const seen = new Set<number>()
  for (const slot of value.slots) {
    if (!record(slot) || !integer(slot.index, 0, 3) || typeof slot.connected !== 'boolean' || typeof slot.fresh !== 'boolean') return null
    if (seen.has(slot.index as number)) return null
    seen.add(slot.index as number)
    if (slot.age_ms !== null && (!number(slot.age_ms) || slot.age_ms < 0)) return null
    if (!['fresh', 'stale', 'disconnected', 'unavailable', 'waiting', 'invalid'].includes(String(slot.status)) || !nullableString(slot.reason)) return null
    if (slot.sample !== null && !validSample(slot.sample)) return null
    if (slot.intent !== null && !validIntent(slot.intent, config)) return null
    if (slot.neutral !== null && typeof slot.neutral !== 'boolean') return null
    if ((slot.status === 'fresh') !== slot.fresh) return null
    if (slot.fresh && (!slot.connected || slot.age_ms === null || slot.sample === null || slot.intent === null || slot.neutral === null)) return null
  }
  return value as unknown as GamepadPreview
}

// 输入时效单独推进，板端离线不影响预览；核心离线或输入超龄则清空可用值。
export function ageGamepadPreview(preview: GamepadPreview | null, elapsedMs: number, coreLive: boolean): GamepadPreview | null {
  if (!preview || !coreLive || !number(elapsedMs) || elapsedMs < 0) return null
  return {
    ...preview,
    slots: preview.slots.map(slot => {
      const age = slot.age_ms === null ? null : slot.age_ms + elapsedMs
      const fresh = preview.status === 'running' && slot.connected && slot.fresh && slot.status === 'fresh' && age !== null && age <= preview.config.input_timeout_s * 1000
      const expired = slot.fresh && !fresh && preview.status === 'running'
      return {
        ...slot, age_ms: age === null ? null : Math.round(age), fresh,
        status: (expired ? 'stale' : preview.status !== 'running' ? 'unavailable' : slot.status) as GamepadSlotStatus,
        reason: expired ? '输入样本已超龄，意图不可用' : slot.reason,
        sample: fresh ? slot.sample : null, intent: fresh ? slot.intent : null, neutral: fresh ? slot.neutral : null,
      }
    }),
  }
}

export const gamepadStatusLabel: Record<GamepadSlotStatus, string> = {
  fresh: '输入新鲜', stale: '输入已过期', disconnected: '未连接', unavailable: '不可用', waiting: '等待采样', invalid: '输入无效',
}
