<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from "vue";
import AppHeader from "./components/AppHeader.vue";
import ConnectionBanner from "./components/ConnectionBanner.vue";
import FireWarningOverlay from "./components/FireWarningOverlay.vue";
import { connectStatus } from "./api";
import { useAggregateStore } from "./stores/aggregate";
import { useConnectionStore } from "./stores/connection";
import { useLogsStore } from "./stores/logs";
import { useTemplatesStore } from "./stores/templates";
import type { StatusSocketHandle } from "./types";

const connection = useConnectionStore();
const aggregate = useAggregateStore();
const logs = useLogsStore();
const templates = useTemplatesStore();
const socket = ref<StatusSocketHandle>();
let refreshTimer: number | undefined;

async function refreshAll(): Promise<void> {
  await connection.refreshHealth();
  await Promise.allSettled([
    aggregate.refreshState(),
    templates.loadTemplates(),
    logs.refreshLogs()
  ]);
}

onMounted(async () => {
  await refreshAll();
  try {
    socket.value = connectStatus({
      onOpen: () => connection.setWebSocketOnline(true),
      onClose: () => connection.setWebSocketOnline(false),
      onError: (error) => {
        connection.setWebSocketOnline(false);
        console.warn("状态 WebSocket 错误", error);
      },
      onStateUpdate: aggregate.applyState,
      onLogUpdate: logs.applyLogs,
      onDemoWarningClear: logs.clearDemoWarning
    });
  } catch (error) {
    connection.setWebSocketOnline(false);
    console.warn("状态 WebSocket 不可用", error);
  }
  refreshTimer = window.setInterval(refreshAll, 15000);
});

onBeforeUnmount(() => {
  socket.value?.close();
  if (refreshTimer) {
    window.clearInterval(refreshTimer);
  }
});
</script>

<template>
  <div class="app-shell">
    <AppHeader />
    <ConnectionBanner />
    <main class="page-region">
      <RouterView />
    </main>
    <FireWarningOverlay :warning="logs.activeDemoWarning" />
  </div>
</template>
