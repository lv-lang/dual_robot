<script setup lang="ts">
import { computed } from "vue";
import { Wifi, WifiOff } from "lucide-vue-next";
import StatusPill from "./StatusPill.vue";
import { useConnectionStore } from "../stores/connection";
import { commandReasonLabel } from "../i18n";

const connection = useConnectionStore();

const statusTone = computed(() => (connection.commandDisabled ? "bad" : "ok"));
const statusLabel = computed(() => (connection.commandDisabled ? "操作已禁用" : "可以操作"));
</script>

<template>
  <section class="connection-banner" :class="{ degraded: connection.commandDisabled }">
    <div class="status-rail row wrap" aria-label="全局工业状态">
      <span class="rail-icon">
        <Wifi v-if="!connection.commandDisabled" :size="20" aria-hidden="true" />
        <WifiOff v-else :size="20" aria-hidden="true" />
      </span>
      <span class="rail-label">全局状态</span>
      <StatusPill label="实车部署" tone="info" />
      <StatusPill label="视觉画面接入" tone="info" />
      <StatusPill :label="statusLabel" :tone="statusTone" />
      <StatusPill :label="connection.backendOnline ? '后端在线' : '后端离线'" :tone="connection.backendOnline ? 'ok' : 'bad'" />
      <StatusPill
        :label="connection.dispatchReady ? '调度在线' : '调度不可用'"
        :tone="connection.dispatchReady ? 'ok' : 'warn'"
      />
      <StatusPill
        :label="connection.websocketOnline ? '实时连接在线' : '实时连接离线'"
        :tone="connection.websocketOnline ? 'ok' : 'bad'"
      />
    </div>
    <div v-if="connection.commandDisabledReasons.length" class="reason-list">
      <span v-for="reason in connection.commandDisabledReasons" :key="reason">{{ commandReasonLabel(reason) }}</span>
    </div>
  </section>
</template>

<style scoped>
.connection-banner {
  position: sticky;
  top: 76px;
  z-index: 10;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 14px;
  width: 100%;
  margin: 0;
  border: 2px solid var(--line-strong);
  border-right: 0;
  border-left: 0;
  border-radius: 0;
  padding: 10px 12px;
  color: var(--primary);
  background: var(--surface-raised);
  box-shadow: none;
}

.connection-banner.degraded {
  border-color: var(--red);
  background: #fff2f1;
}

.status-rail {
  min-width: 0;
}

.rail-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 34px;
  height: 30px;
  border: 2px solid var(--line-strong);
  border-radius: var(--radius-sm);
  color: #ffffff;
  background: var(--primary);
}

.rail-label {
  color: var(--muted);
  font-size: 12px;
  font-weight: 900;
  white-space: nowrap;
}

.reason-list {
  display: flex;
  justify-content: flex-end;
  gap: 8px;
  flex-wrap: wrap;
  color: var(--muted);
  font-size: 13px;
}

.reason-list span {
  border: 1px solid var(--line);
  border-radius: var(--radius-sm);
  padding: 5px 8px;
  background: #ffffff;
}

@media (max-width: 900px) {
  .connection-banner {
    position: static;
    width: 100%;
    align-items: flex-start;
    flex-direction: column;
  }

  .reason-list {
    justify-content: flex-start;
  }
}
</style>
