<script setup lang="ts">
import { computed } from 'vue'
import { fieldIssue, formatField, type Topic } from '../domain/fields'
import type { TelemetrySample } from '../types'
const props = defineProps<{ sample?: TelemetrySample; topic: Topic; field: string }>()
const issue = computed(() => fieldIssue(props.sample, props.topic, props.field))
</script>
<template>
  <span class="data-value" :class="{ unavailable: issue }" :title="issue ?? field">{{ issue ? '—' : formatField(topic, field, sample?.data[field]) }}</span>
</template>
