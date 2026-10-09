<script setup lang="ts">
import { computed, ref } from 'vue'
import { fields, topics, definition, formatField, fieldIssue, type Topic } from '../domain/fields'
import type { GroundStationSnapshot, TelemetrySample } from '../types'
const props = defineProps<{ telemetry: Record<string, TelemetrySample>; snapshot: GroundStationSnapshot | null; coreLive: boolean }>()
const selected = ref<Topic>('ugekf')
const raw = ref(false)
const search = ref('')
const sample = computed(() => props.telemetry[selected.value])
const rows = computed(() => {
  const known = fields[selected.value]
  const extra = Object.keys(sample.value?.data ?? {}).filter(key => !known.some(item => item.key === key)).map(key => definition(selected.value, key))
  const query = search.value.toLowerCase().trim()
  return [...known, ...extra].filter(item => (item.key + item.label).toLowerCase().includes(query))
})
</script>
<template>
  <section class="panel diagnostic-view">
    <header class="panel-heading">
      <div class="segmented" aria-label="遥测主题"><button v-for="topic in topics" :key="topic.key" :class="{ selected: selected === topic.key }" @click="selected = topic.key">{{ topic.label }} <small>{{ topic.key.toUpperCase() }}</small></button></div>
      <label class="search-field">筛选字段 <input v-model="search" placeholder="名称或协议字段" :disabled="raw" /></label>
      <button class="text-button" @click="raw = !raw">{{ raw ? '返回字段表' : '原始快照' }}</button>
    </header>
    <div class="diagnostic-summary"><span>{{ topics.find(topic => topic.key === selected)?.description }}</span><span :class="sample?.fresh ? 'tone-ok' : 'tone-warn'">{{ !coreLive ? '状态不可确认' : !sample ? '尚未收到' : sample.fresh ? '接收新鲜' : '已过期' }}</span><span>接收年龄 {{ sample ? sample.age_ms + ' ms' : '—' }}</span><span>诊断保留原值；接收新鲜不代表字段有效。</span></div>
    <div v-if="!raw" class="panel-scroll">
      <table class="data-table diagnostics-table"><thead><tr><th>名称</th><th>协议字段</th><th class="numeric">原始 / 解码值</th><th>单位</th><th>可用性</th></tr></thead>
        <tbody><tr v-for="item in rows" :key="item.key"><td>{{ item.label }}</td><td class="mono subtle">{{ item.key }}</td><td class="numeric mono">{{ formatField(selected, item.key, sample?.data[item.key]) }}</td><td class="subtle">{{ item.unit || '—' }}</td><td :class="fieldIssue(sample, selected, item.key) ? 'tone-warn' : 'subtle'">{{ fieldIssue(sample, selected, item.key) ?? '可读' }}</td></tr></tbody>
      </table><p v-if="rows.length === 0" class="empty-message">没有匹配的字段</p>
    </div>
    <pre v-else class="panel-scroll raw-snapshot">{{ JSON.stringify(snapshot, null, 2) }}</pre>
  </section>
</template>
