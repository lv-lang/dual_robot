<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from "vue";
import { Play, RefreshCw, RotateCcw, Square } from "lucide-vue-next";
import DangerConfirmModal from "../components/DangerConfirmModal.vue";
import StatusPill from "../components/StatusPill.vue";
import LogList from "../components/LogList.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useSystemStore } from "../stores/system";
import {
  systemCategoryLabel,
  systemHealthItemLabel,
  systemHealthLabel,
  systemStatusLabel
} from "../i18n";

type DangerousAction = "stop" | "restart";

const SYSTEM_REFRESH_INTERVAL_MS = 1000;
const ACTION_FOLLOW_TIMEOUT_MS = 60000;
const STABLE_SYSTEM_STATUSES = new Set(["running", "stopped", "failed", "external"]);

const system = useSystemStore();
const aggregate = useAggregateStore();
const confirmAction = ref<DangerousAction>();
let autoRefreshTimer: number | undefined;
let actionFollowTimer: number | undefined;
let refreshInFlight = false;

const activeTaskCount = computed(() => aggregate.activeTasks.length);
const visibleHealthRows = computed(() =>
  system.status.health.filter((row) => row.id !== "process.rviz2" && row.label.toLowerCase() !== "rviz")
);
const statusTone = computed(() => {
  if (system.status.status === "running") {
    return "ok";
  }
  if (system.status.status === "failed") {
    return "bad";
  }
  if (["starting", "stopping", "degraded", "external"].includes(system.status.status)) {
    return "warn";
  }
  return "neutral";
});

const confirmTitle = computed(() =>
  confirmAction.value === "restart" ? "重启调度系统" : "停止调度系统"
);

const confirmBody = computed(() => {
  const taskText = activeTaskCount.value > 0
    ? `当前有 ${activeTaskCount.value} 个未结束任务。`
    : "";
  if (confirmAction.value === "restart") {
    return `${taskText}这会停止当前 App 管理的调度运行栈，然后重新启动；当前任务会中断。`;
  }
  return `${taskText}这会停止当前 App 管理的调度运行栈；当前任务会中断。`;
});

const confirmLabel = computed(() =>
  confirmAction.value === "restart" ? "确认重启" : "确认停止"
);

function formatDateTime(value?: string): string {
  if (!value) {
    return "-";
  }
  const timestamp = Date.parse(value);
  if (Number.isNaN(timestamp)) {
    return value;
  }
  return new Intl.DateTimeFormat("zh-CN", {
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false
  }).format(new Date(timestamp));
}

function healthTone(status: string): "ok" | "warn" | "bad" | "neutral" {
  if (status === "ok") {
    return "ok";
  }
  if (status === "failed") {
    return "bad";
  }
  if (status === "missing") {
    return "warn";
  }
  return "neutral";
}

async function refresh(silent = false): Promise<void> {
  if (refreshInFlight) {
    return;
  }
  refreshInFlight = true;
  try {
    await Promise.allSettled([system.refreshAll({ silent }), aggregate.refreshState()]);
  } finally {
    refreshInFlight = false;
  }
}

async function start(): Promise<void> {
  await system.start();
  await aggregate.refreshState();
  startActionFollow();
}

async function runDangerousAction(): Promise<void> {
  if (confirmAction.value === "restart") {
    await system.restart();
  } else if (confirmAction.value === "stop") {
    await system.stop();
  }
  confirmAction.value = undefined;
  await aggregate.refreshState();
  startActionFollow();
}

function stopAutoRefresh(): void {
  if (autoRefreshTimer !== undefined) {
    window.clearInterval(autoRefreshTimer);
    autoRefreshTimer = undefined;
  }
}

function startAutoRefresh(): void {
  stopAutoRefresh();
  autoRefreshTimer = window.setInterval(() => {
    void refresh(true);
  }, SYSTEM_REFRESH_INTERVAL_MS);
}

function stopActionFollow(): void {
  if (actionFollowTimer !== undefined) {
    window.clearInterval(actionFollowTimer);
    actionFollowTimer = undefined;
  }
}

function startActionFollow(): void {
  stopActionFollow();
  const deadline = Date.now() + ACTION_FOLLOW_TIMEOUT_MS;
  actionFollowTimer = window.setInterval(async () => {
    await refresh(true);
    if (STABLE_SYSTEM_STATUSES.has(system.status.status) || Date.now() >= deadline) {
      stopActionFollow();
    }
  }, SYSTEM_REFRESH_INTERVAL_MS);
}

onMounted(async () => {
  await refresh();
  startAutoRefresh();
});

onBeforeUnmount(() => {
  stopAutoRefresh();
  stopActionFollow();
});
</script>

<template>
  <section class="stack">
    <div class="page-heading">
      <h1>系统诊断</h1>
      <div class="button-row">
        <button class="button-secondary" type="button" :disabled="system.loading || system.logsLoading" @click="refresh()">
          <RefreshCw :size="18" aria-hidden="true" />
          <span>刷新</span>
        </button>
        <button type="button" :disabled="!system.canStart" @click="start">
          <Play :size="18" aria-hidden="true" />
          <span>启动调度系统</span>
        </button>
        <button class="button-warning" type="button" :disabled="!system.canRestart" @click="confirmAction = 'restart'">
          <RotateCcw :size="18" aria-hidden="true" />
          <span>重启调度系统</span>
        </button>
        <button class="button-danger" type="button" :disabled="!system.canStop" @click="confirmAction = 'stop'">
          <Square :size="18" aria-hidden="true" />
          <span>停止调度系统</span>
        </button>
      </div>
    </div>

    <p v-if="system.error || system.logsError" class="notice error">
      {{ system.error || system.logsError }}
    </p>
    <p v-if="system.actionMessage" class="notice">{{ system.actionMessage }}</p>
    <p v-if="system.status.external_running" class="notice">
      检测到外部运行中的调度系统。App 只读显示状态，不允许停止或重启外部进程。
    </p>

    <section class="panel system-summary">
      <div class="summary-main">
        <div class="row wrap">
          <StatusPill :label="systemStatusLabel(system.status.status)" :tone="statusTone" />
          <StatusPill :label="system.ownershipLabel" :tone="system.status.external_running ? 'warn' : system.status.managed ? 'info' : 'neutral'" />
        </div>
        <h2>{{ system.status.summary }}</h2>
        <p class="muted">{{ system.status.profile.name }}</p>
      </div>
      <dl class="system-meta">
        <div>
          <dt>PID</dt>
          <dd>{{ system.status.pid ?? "-" }}</dd>
        </div>
        <div>
          <dt>PGID</dt>
          <dd>{{ system.status.pgid ?? "-" }}</dd>
        </div>
        <div>
          <dt>启动时间</dt>
          <dd>{{ formatDateTime(system.status.started_at) }}</dd>
        </div>
        <div>
          <dt>更新时间</dt>
          <dd>{{ formatDateTime(system.status.updated_at) }}</dd>
        </div>
      </dl>
    </section>

    <section class="panel stack">
      <h2 class="section-title">节点状态</h2>
      <div v-if="visibleHealthRows.length" class="health-table" role="table" aria-label="节点状态">
        <div class="health-row health-head" role="row">
          <span>类型</span>
          <span>检查项</span>
          <span>状态</span>
        </div>
        <div v-for="row in visibleHealthRows" :key="row.id" class="health-row" role="row">
          <span>{{ systemCategoryLabel(row.category) }}</span>
          <strong>{{ systemHealthItemLabel(row.id, row.label) }}</strong>
          <StatusPill :label="systemHealthLabel(row.status)" :tone="healthTone(row.status)" />
        </div>
      </div>
      <p v-else class="empty">暂无节点状态</p>
    </section>

    <section class="grid two">
      <div class="panel stack">
        <div class="row between">
          <h2 class="section-title">系统运行日志</h2>
          <button class="button-secondary" type="button" :disabled="system.logsLoading" @click="system.refreshLogs()">
            <RefreshCw :size="18" aria-hidden="true" />
            <span>刷新日志</span>
          </button>
        </div>
        <div v-if="system.launchLogs.length" class="launch-log">
          <p v-for="line in system.launchLogs" :key="`${line.line_no}-${line.message}`">
            <span>{{ line.line_no }}</span>
            <code>{{ line.message }}</code>
          </p>
        </div>
        <p v-else class="empty">暂无运行日志</p>
      </div>

      <div class="panel stack">
        <h2 class="section-title">系统操作日志</h2>
        <LogList :logs="system.operationLogs" />
      </div>
    </section>

    <DangerConfirmModal
      :open="Boolean(confirmAction)"
      :title="confirmTitle"
      :body="confirmBody"
      :confirm-label="confirmLabel"
      :busy="system.actionBusy"
      @cancel="confirmAction = undefined"
      @confirm="runDangerousAction"
    />
  </section>
</template>

<style scoped>
.button-row button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

.system-summary {
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(260px, 420px);
  gap: 18px;
  align-items: start;
}

.summary-main {
  display: grid;
  gap: 10px;
  min-width: 0;
}

.summary-main h2 {
  margin: 0;
  font-size: 22px;
  line-height: 1.2;
}

.system-meta {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 10px;
  margin: 0;
}

.system-meta div {
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 10px;
  background: #ffffff;
}

.system-meta dt {
  color: var(--muted);
  font-size: 12px;
  font-weight: 800;
}

.system-meta dd {
  margin: 4px 0 0;
  overflow-wrap: anywhere;
  font-weight: 800;
}

.health-table {
  display: grid;
  gap: 6px;
}

.health-row {
  display: grid;
  grid-template-columns: 120px minmax(170px, 1fr) 96px;
  gap: 10px;
  align-items: center;
  min-height: 42px;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 8px 10px;
  background: #ffffff;
}

.health-head {
  min-height: 34px;
  color: var(--muted);
  background: var(--surface-muted);
  font-size: 12px;
  font-weight: 900;
}

.launch-log {
  display: grid;
  gap: 5px;
  max-height: 360px;
  overflow: auto;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 10px;
  background: #111b1a;
}

.launch-log p {
  display: grid;
  grid-template-columns: 46px 1fr;
  gap: 8px;
  margin: 0;
  color: #dce8e4;
  font-size: 12px;
}

.launch-log span {
  color: #8fa39d;
  text-align: right;
}

.launch-log code {
  white-space: pre-wrap;
  overflow-wrap: anywhere;
}

@media (max-width: 980px) {
  .system-summary,
  .grid.two {
    grid-template-columns: 1fr;
  }

  .health-row {
    grid-template-columns: 1fr;
    align-items: start;
  }

  .health-head {
    display: none;
  }
}
</style>
