import type { TelemetrySample } from '../types'

export type Topic = 'ugekf' | 'ugctl' | 'ugwrm'
export interface FieldDefinition { key: string; label: string; unit: string; precision: number; values?: Record<number, string> }
const field = (key: string, label: string, unit = '', precision = 0, values?: Record<number, string>): FieldDefinition => ({ key, label, unit, precision, values })
const binary = { 0: '否', 1: '是' }
export const topics: { key: Topic; label: string; description: string }[] = [
  { key: 'ugekf', label: '状态估计', description: 'UGEKF · 姿态、深度与估计质量' },
  { key: 'ugctl', label: '控制反馈', description: 'UGCTL · 控制器快照与回执' },
  { key: 'ugwrm', label: '双轴与总线', description: 'UGWRM · pitch / roll 位置与校准' },
]

// 显示名称、单位和枚举只有这一份定义，监控与诊断共同使用。
export const fields: Record<Topic, FieldDefinition[]> = {
  ugekf: [
    field('seq', '帧序号'), field('t_ms', '板端时间', 'ms'),
    field('status', '估计器状态', '', 0, { 0: '未初始化', 1: '等待启动', 2: '等待静止', 3: '对齐中', 4: '运行中', 5: '故障' }),
    field('fl', '健康与就绪标志'), field('roll_deg', '横滚', '°', 2), field('pitch_deg', '俯仰', '°', 2), field('yaw_deg', '航向', '°', 2),
    field('pN', '北向相对位置', 'm', 3), field('pE', '东向相对位置', 'm', 3), field('depth', '深度', 'm', 3),
    field('vN', '北向速度', 'm/s', 3), field('vE', '东向速度', 'm/s', 3), field('vD', '下向速度', 'm/s', 3),
    ...['x', 'y', 'z'].map(axis => field('bg' + axis, '陀螺零偏 ' + axis.toUpperCase(), 'rad/s', 5)),
    ...['x', 'y', 'z'].map(axis => field('ba' + axis, '加计零偏 ' + axis.toUpperCase(), 'm/s²', 5)),
    field('Ppos', '位置协方差迹', 'm²', 3), field('Pvel', '速度协方差迹', '(m/s)²', 3), field('Patt', '姿态协方差迹', 'rad²', 3),
    field('nisD', '深度归一化创新', '', 3), field('accD', '深度更新接受次数'), field('rejD', '深度更新拒绝次数'),
    field('dstreak', '深度连续拒绝次数'), field('dreset', '深度重置次数'), field('dmeas', '压力测量深度', 'm', 3),
    field('dt', '传播步长', 's', 4), field('imuhz', 'IMU 更新频率', 'Hz', 2), field('tlast', '最近 IMU 时间', 's', 3),
    field('cal_crc', '磁标定 CRC'), field('cal_valid', '磁标定有效', '', 0, binary),
  ],
  ugctl: [
    field('schema', '协议版本'), field('seq', '帧序号'), field('t_ms', '板端时间', 'ms'),
    field('mode', '控制模式', '', 0, { 0: '关闭', 1: '定俯仰', 2: '深度串级' }), field('phase', '航行相位', '', 0, { 0: '下潜', 1: '上浮' }),
    field('armed', '控制器使能', '', 0, binary), field('output_valid', '控制输出有效', '', 0, binary),
    field('fault', '控制服务状态', '', 0, { 0: '正常', 1: '未使能', 2: '指令超时', 3: 'PID 无效', 4: '参数 CRC 不匹配', 5: '估计无效', 6: '深度过期', 7: '指令无效', 8: '控制核心拒绝', 9: '位移无效' }),
    field('core_fault', '控制核心状态', '', 0, { 0: '正常', 1: '未使能', 2: '估计无效', 3: '配置无效', 4: '指令无效', 5: '步长无效', 6: '深度无效', 7: '位移无效' }),
    field('config_reject', '参数拒绝码'), field('command_id', '指令序号'), field('age_ms', '板端指令年龄', 'ms'),
    field('pid_valid', 'PID 参数有效', '', 0, binary), field('required_crc', '期望参数 CRC'), field('active_crc', '当前参数 CRC'), field('depth_seq', '深度采样序号'),
    field('target_depth', '目标深度', 'm', 3), field('depth', '控制器深度', 'm', 3), field('depth_error', '深度误差', 'm', 3),
    field('target_pitch_deg', '指令俯仰目标', '°', 2), field('pitch_sp_deg', '实际俯仰给定', '°', 2), field('pitch_deg', '控制器俯仰', '°', 2), field('pitch_error_deg', '俯仰误差', '°', 2),
    ...['p', 'i', 'd', 'out'].map(part => field('outer_' + part + '_mm', '深度环 ' + part.toUpperCase(), 'mm', 2)), field('outer_sat', '深度环饱和', '', 0, binary),
    field('target_displacement_mm', '目标油囊位移', 'mm', 2), field('displacement_mm', '实测油囊位移', 'mm', 2), field('displacement_error_mm', '油囊位移误差', 'mm', 2),
    ...['p', 'i', 'd', 'out'].map(part => field('inner_' + part + '_rpm', '俯仰环 ' + part.toUpperCase(), 'rpm', 1)), field('inner_sat', '俯仰环饱和', '', 0, binary),
    field('worm_rpm', '控制器蜗杆输出', 'rpm'), field('pump_enable', '控制器泵输出', '', 0, { 0: '停止', 1: '运行' }),
    field('pump_direction', '控制器泵方向', '', 0, { 0: '上浮方向', 1: '下潜方向' }), field('thrust_pct', '推进输出', '%', 1),
    field('config_req', '参数请求次数'), field('config_apply', '参数应用次数'), field('config_reject_count', '参数拒绝次数'),
  ],
  ugwrm: [
    field('schema', '协议版本'), field('seq', '帧序号'), field('t_ms', '板端时间', 'ms'), field('pulse', '当前位置脉冲', 'pulse'),
    field('stroke_percent', '蜗杆行程位置', '%', 2), field('mm_from_front', '距前端距离', 'mm', 2), field('span_valid', '行程参考有效', '', 0, binary),
    field('data_fresh', '驱动反馈新鲜', '', 0, binary), field('ref_kind', '位置参考', '', 0, { 0: '无参考', 1: '上电参考', 2: '回零参考' }), field('ref_lost', '参考丢失次数'),
    field('pulse_min', '最小原始脉冲', 'pulse'), field('pulse_max', '最大原始脉冲', 'pulse'), field('span_pulses', '行程跨度', 'pulse'),
    field('moving', '电机运动', '', 0, binary), field('alarm', '驱动器报警', '', 0, binary),
    field('home_state', 'pitch 校准阶段', '', 0, { 0: '未校准', 1: '趋近前限位', 2: '慢速退出', 3: 'pitch 完成', 4: 'pitch 失败' }),
    field('home_fail', 'pitch 失败原因', '', 0, { 0: '无', 1: '趋近超时', 2: '退出超时', 3: '反馈过期', 4: '已取消', 5: '端点超出脉冲范围', 6: '两端限位同时触发', 7: 'roll 失败，联合停止' }), field('pulse_home', 'pitch 零点脉冲', 'pulse'),
    field('cmd_rpm', '电机指令转速', 'rpm'), field('fb_rpm', '电机反馈转速', 'rpm'), field('status_word', '驱动状态字'), field('err', '驱动错误码'), field('err_sub', '驱动错误子码'),
    field('input_ports', '驱动输入位图'), field('limit_bits', '限位输入位图'), field('poll_ok', '总线轮询成功次数'), field('poll_fail', '总线轮询失败次数'),
    field('overrun', '串口溢出次数'), field('enable_resync', '使能重同步次数'), field('last_error', '最近总线错误码'),
    field('roll_pulse', 'roll 软件计数', 'pulse'), field('roll_pulse_min', 'roll PB3 零端', 'pulse'),
    field('roll_pulse_max', 'roll PB5 满端', 'pulse'), field('roll_span_pulses', 'roll 实测跨度', 'pulse'),
    field('roll_percent', 'roll 位置估算', '%', 2), field('roll_valid', 'roll 参考有效', '', 0, binary),
    field('roll_home_state', 'roll 校准阶段', '', 0, { 0: '未校准', 1: '趋近 PB3', 2: '退出 PB3', 3: '趋近 PB5', 4: '退出 PB5', 5: '返回 50%', 6: 'roll 完成', 7: 'roll 失败' }),
    field('roll_home_fail', 'roll 失败原因', '', 0, { 0: '无', 1: '寻端超时', 2: '退出超时', 3: '回中超时', 4: '两端限位冲突', 5: '驱动停止或启动失败', 6: '跨度无效', 7: '已取消', 8: '另一轴失败' }),
    field('calib_state', '双轴校准状态', '', 0, { 0: '未校准', 1: '校准中', 2: '双轴完成', 3: '失败', 4: '已取消' }),
    field('calib_fail', '双轴校准失败原因', '', 0, { 0: '无', 1: 'pitch 失败', 2: 'roll 失败', 3: '已取消', 4: 'PID 接管' }), field('calib_hold', '校准控制保持', '', 0, binary),
  ],
}

export const invalidReasonLabels: Record<string, string> = {
  non_finite: '数值无效', ekf_not_valid: '估计未就绪', worm_span_invalid: '行程参考未建立', worm_feedback_stale: '驱动反馈过期',
  roll_reference_invalid: 'roll 参考未建立',
}
export function definition(topic: Topic, key: string): FieldDefinition {
  return fields[topic].find(item => item.key === key) ?? field(key, key, '', 3)
}
export function fieldIssue(sample: TelemetrySample | undefined, topic: Topic, key: string): string | null {
  if (!sample) return '未收到'
  if (!sample.fresh) return '已过期 / 状态不可确认'
  const reason = sample.invalid_fields?.[key]
  if (reason) return invalidReasonLabels[reason] ?? reason
  const value = sample.data[key]
  if (value === null || value === undefined || (typeof value === 'number' && !Number.isFinite(value))) return '数值不可用'
  // 空 invalid_fields 不等于板端有效位存在；关键信息缺失时保守显示未知。
  if (topic === 'ugekf' && ['roll_deg', 'pitch_deg', 'yaw_deg', 'pN', 'pE', 'depth', 'vN', 'vE', 'vD', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz'].includes(key)) {
    const flags = sample.data.fl
    if (typeof flags !== 'number' || !Number.isInteger(flags) || flags < 0) return '估计有效性未知'
    if ((flags & 2) === 0) return '估计未就绪'
  }
  if (topic === 'ugwrm' && key === 'roll_percent') {
    if (sample.data.roll_valid !== 0 && sample.data.roll_valid !== 1) return 'roll 参考有效性未知'
    if (sample.data.roll_valid !== 1) return 'roll 参考未建立'
  }
  if (topic === 'ugwrm' && ['stroke_percent', 'mm_from_front'].includes(key)) {
    if (sample.data.span_valid !== 0 && sample.data.span_valid !== 1) return '行程参考有效性未知'
    if (sample.data.span_valid !== 1) return '行程参考未建立'
  }
  if (topic === 'ugwrm' && ['stroke_percent', 'mm_from_front', 'pulse', 'fb_rpm'].includes(key)) {
    if (sample.data.data_fresh !== 0 && sample.data.data_fresh !== 1) return '驱动反馈状态未知'
    if (sample.data.data_fresh !== 1) return '驱动反馈过期'
  }
  return null
}
export function numeric(sample: TelemetrySample | undefined, topic: Topic, key: string): number | null {
  if (fieldIssue(sample, topic, key)) return null
  const value = sample?.data[key]
  return typeof value === 'number' && Number.isFinite(value) ? value : null
}
export function formatField(topic: Topic, key: string, value: unknown): string {
  if (value === null || value === undefined) return '—'
  const meta = definition(topic, key)
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) return '—'
    return meta.values ? meta.values[value] ?? '未知 (' + value + ')' : value.toFixed(meta.precision)
  }
  return typeof value === 'object' ? JSON.stringify(value) : String(value)
}
