<script setup lang="ts">
import StatusPill from "./StatusPill.vue";
import type { LogEntry } from "../types";
import { logEventLabel, logLevelLabel } from "../i18n";
import { genericTaskDisplayName } from "../taskDisplay";

defineProps<{
  logs: LogEntry[];
}>();

function tone(level: string): "ok" | "warn" | "bad" | "neutral" {
  if (level === "error") {
    return "bad";
  }
  if (level === "warning") {
    return "warn";
  }
  if (level === "info") {
    return "ok";
  }
  return "neutral";
}

function logTaskDisplayName(log: LogEntry): string {
  return log.task_display_name?.trim() || genericTaskDisplayName(log.task_id);
}

function formatLogTime(value: string): string {
  const timestamp = Date.parse(value);
  if (Number.isNaN(timestamp)) {
    return value || "--:--:--";
  }
  return new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false
  }).format(new Date(timestamp));
}
</script>

<template>
  <div v-if="logs.length" class="log-list">
    <article v-for="log in logs" :key="log.log_id" class="log-row">
      <div class="log-time">{{ formatLogTime(log.timestamp) }}</div>
      <div class="log-main">
        <div class="row wrap">
          <strong>{{ logEventLabel(log.event) }}</strong>
          <StatusPill :label="logLevelLabel(log.level)" :tone="tone(log.level)" />
          <span v-if="log.robot_id" class="tag">{{ log.robot_id }}</span>
          <span v-if="log.task_id" class="tag">{{ logTaskDisplayName(log) }}</span>
        </div>
        <p>{{ log.message }}</p>
      </div>
    </article>
  </div>
  <p v-else class="empty">暂无日志</p>
</template>

<style scoped>
.log-list {
  display: grid;
  gap: 8px;
}

.log-row {
  display: grid;
  grid-template-columns: 94px 1fr;
  gap: 12px;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 12px;
  background: #ffffff;
}

.log-time {
  color: var(--muted);
  font-size: 13px;
  font-weight: 800;
}

.log-main {
  min-width: 0;
}

p {
  margin: 8px 0 0;
  color: var(--muted);
  overflow-wrap: anywhere;
}

.tag {
  border-radius: 999px;
  padding: 5px 8px;
  background: var(--surface-muted);
  color: var(--primary);
  font-size: 12px;
  font-weight: 800;
}

@media (max-width: 620px) {
  .log-row {
    grid-template-columns: 1fr;
  }
}
</style>
