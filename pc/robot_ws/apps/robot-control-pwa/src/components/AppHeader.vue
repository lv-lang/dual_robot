<script setup lang="ts">
import { computed, ref } from "vue";
import { AlertOctagon, ClipboardCheck, FileText, LayoutDashboard, ListChecks, Settings } from "lucide-vue-next";
import DangerConfirmModal from "./DangerConfirmModal.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useLogsStore } from "../stores/logs";

const connection = useConnectionStore();
const aggregate = useAggregateStore();
const logs = useLogsStore();
const confirmOpen = ref(false);
const busy = ref(false);
const error = ref<string>();

const emergencyDisabled = computed(() => connection.commandDisabled || busy.value);

async function confirmEmergencyStop(): Promise<void> {
  busy.value = true;
  error.value = undefined;
  try {
    await aggregate.emergencyStop();
    await logs.refreshLogs();
    confirmOpen.value = false;
  } catch (caught) {
    error.value = caught instanceof Error ? caught.message : "急停失败";
  } finally {
    busy.value = false;
  }
}
</script>

<template>
  <header class="app-header">
    <RouterLink class="brand" to="/" title="工业 HMI 控制台">
      <span class="brand-mark" aria-hidden="true">
        <span class="mark-strip strip-cyan"></span>
        <span class="mark-strip strip-blue"></span>
        <span class="mark-strip strip-red"></span>
      </span>
      <span class="brand-copy">
        <span class="brand-title">工业 HMI 控制台</span>
        <span class="brand-subtitle">mecanum / ackermann</span>
      </span>
    </RouterLink>
    <nav class="top-nav" aria-label="Primary">
      <RouterLink to="/" title="态势">
        <LayoutDashboard :size="19" aria-hidden="true" />
        <span>态势</span>
      </RouterLink>
      <RouterLink to="/tasks" title="任务">
        <ListChecks :size="19" aria-hidden="true" />
        <span>任务</span>
      </RouterLink>
      <RouterLink to="/confirmations" title="待确认">
        <ClipboardCheck :size="19" aria-hidden="true" />
        <span>待确认</span>
      </RouterLink>
      <RouterLink to="/logs" title="事件日志">
        <FileText :size="19" aria-hidden="true" />
        <span>事件日志</span>
      </RouterLink>
      <RouterLink to="/system" title="系统诊断">
        <Settings :size="19" aria-hidden="true" />
        <span>系统诊断</span>
      </RouterLink>
    </nav>
    <button
      class="button-danger emergency-button"
      type="button"
      :disabled="emergencyDisabled"
      title="全局急停"
      @click="confirmOpen = true"
    >
      <AlertOctagon :size="21" aria-hidden="true" />
      <span>全局急停</span>
    </button>
  </header>

  <p v-if="error" class="header-error">{{ error }}</p>

  <DangerConfirmModal
    :open="confirmOpen"
    title="全局急停"
    body="这会向所有机器人发送全局急停请求。解除急停和恢复操作在 PWA 外处理。"
    confirm-label="停止所有机器人"
    :busy="busy"
    @cancel="confirmOpen = false"
    @confirm="confirmEmergencyStop"
  />
</template>

<style scoped>
.app-header {
  position: sticky;
  top: 0;
  z-index: 20;
  display: grid;
  grid-template-columns: auto 1fr auto;
  align-items: center;
  gap: 16px;
  min-height: 76px;
  border-bottom: 3px solid var(--line-strong);
  padding: 10px 20px;
  color: #ffffff;
  background: var(--chrome);
}

.brand {
  display: inline-flex;
  align-items: center;
  gap: 10px;
  min-width: 270px;
  color: #ffffff;
  font-size: 17px;
  font-weight: 900;
  text-decoration: none;
  white-space: nowrap;
}

.brand-mark {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 3px;
  width: 40px;
  height: 40px;
  border: 2px solid #9baba2;
  border-radius: var(--radius-md);
  padding: 5px;
  background: #111f1a;
}

.mark-strip {
  display: block;
  border-radius: var(--radius-sm);
}

.strip-cyan {
  background: var(--cyan);
}

.strip-blue {
  background: var(--blue);
}

.strip-red {
  background: var(--red);
}

.brand-copy {
  display: grid;
  gap: 2px;
}

.brand-title {
  line-height: 1.05;
}

.brand-subtitle {
  color: var(--chrome-muted);
  font-size: 11px;
  font-weight: 800;
  line-height: 1.1;
}

.top-nav {
  display: flex;
  align-items: center;
  gap: 6px;
  justify-content: center;
}

.top-nav a {
  display: inline-flex;
  align-items: center;
  gap: 7px;
  min-height: 44px;
  border: 1px solid transparent;
  border-radius: var(--radius-md);
  padding: 0 12px;
  color: var(--chrome-muted);
  font-weight: 800;
  text-decoration: none;
  white-space: nowrap;
}

.top-nav a:hover {
  color: #ffffff;
  border-color: #465650;
  background: #16241f;
}

.top-nav a.router-link-active {
  color: #ffffff;
  border-color: var(--accent);
  background: #172820;
  box-shadow: inset 0 -4px 0 var(--accent);
}

.emergency-button {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  min-width: 184px;
  min-height: 52px;
  border-width: 3px;
  justify-content: center;
  font-weight: 900;
  box-shadow: inset 0 0 0 1px rgb(255 255 255 / 46%);
}

.header-error {
  width: 100%;
  margin: 0;
  border-bottom: 2px solid #f2b3ad;
  padding: 8px 20px;
  background: #fff0ee;
  color: var(--red);
  font-weight: 700;
}

@media (max-width: 920px) {
  .app-header {
    position: static;
    grid-template-columns: 1fr;
    gap: 10px;
  }

  .brand {
    min-width: 0;
  }

  .top-nav {
    justify-content: flex-start;
    overflow-x: auto;
  }

  .emergency-button {
    width: 100%;
  }
}
</style>
