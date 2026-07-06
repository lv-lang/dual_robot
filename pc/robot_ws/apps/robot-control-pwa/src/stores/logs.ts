import { computed, ref } from "vue";
import { defineStore } from "pinia";
import { apiClient } from "../api";
import type { LogEntry } from "../types";

export type DemoWarningSeverity = "danger" | "warning";

export interface ActiveDemoWarning {
  point_id: string;
  title: string;
  message: string;
  severity: DemoWarningSeverity;
  timestamp: string;
}

export const useLogsStore = defineStore("logs", () => {
  const logs = ref<LogEntry[]>([]);
  const manualClearedWarnings = ref<Map<string, string>>(new Map());
  const loading = ref(false);
  const error = ref<string>();

  const latest = computed(() => logs.value.slice(0, 100));
  const activeDemoWarning = computed<ActiveDemoWarning | undefined>(() => {
    const cleared = new Map<string, string>(manualClearedWarnings.value);
    for (const log of logs.value) {
      const detail = log.detail ?? {};
      const pointId = String(detail.point_id ?? "P3");
      const severity = String(detail.warning_severity ?? (log.event === "demo_fire_alert" ? "danger" : "warning"));
      const key = `${severity}:${pointId}`;
      if (detail.warning_active === false) {
        cleared.set(key, log.timestamp);
        continue;
      }
      const isWarningEvent = detail.warning_active === true || log.event === "demo_fire_alert";
      const clearedAt = cleared.get(key);
      if (isWarningEvent && (!clearedAt || log.timestamp.localeCompare(clearedAt) > 0)) {
        return {
          point_id: pointId,
          title: String(detail.warning_title ?? "WARNING"),
          message: String(detail.warning_message ?? log.message),
          severity: severity === "danger" ? "danger" : "warning",
          timestamp: log.timestamp
        };
      }
    }
    return undefined;
  });

  function applyLogs(nextLogs: LogEntry[]): void {
    if (nextLogs.length === 0) {
      logs.value = [];
      manualClearedWarnings.value = new Map();
      return;
    }
    const byId = new Map<string, LogEntry>();
    for (const log of [...nextLogs, ...logs.value]) {
      byId.set(log.log_id, log);
    }
    logs.value = [...byId.values()].sort((left, right) => right.timestamp.localeCompare(left.timestamp));
  }

  function clearDemoWarning(warning?: Record<string, unknown>): void {
    const pointId = String(warning?.point_id ?? "P3");
    const severity = String(warning?.warning_severity ?? "danger") === "danger" ? "danger" : "warning";
    const next = new Map(manualClearedWarnings.value);
    next.set(`${severity}:${pointId}`, new Date().toISOString());
    manualClearedWarnings.value = next;
  }

  async function refreshLogs(limit = 100): Promise<void> {
    loading.value = true;
    try {
      applyLogs((await apiClient.getLogs(limit)).logs);
      error.value = undefined;
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "日志刷新失败";
    } finally {
      loading.value = false;
    }
  }

  return {
    logs,
    latest,
    activeDemoWarning,
    loading,
    error,
    applyLogs,
    clearDemoWarning,
    refreshLogs
  };
});
