import { computed, ref } from "vue";
import { defineStore } from "pinia";
import { apiClient } from "../api";
import type { AggregateStatus, HealthResponse } from "../types";

export const useConnectionStore = defineStore("connection", () => {
  const backendOnline = ref(false);
  const dispatchOnline = ref(false);
  const dispatchDegraded = ref(true);
  const websocketOnline = ref(false);
  const updatedAt = ref<string>();
  const backendReason = ref<string>();
  const backendDisabledReasons = ref<string[]>([]);
  const healthLoading = ref(false);

  const dispatchReady = computed(() => dispatchOnline.value && !dispatchDegraded.value);

  const commandDisabledReasons = computed(() => {
    const reasons: string[] = [];
    if (!backendOnline.value) {
      reasons.push("Backend offline");
    }
    if (!dispatchReady.value) {
      reasons.push(dispatchOnline.value ? "Dispatch degraded" : "Dispatch offline");
    }
    if (!websocketOnline.value) {
      reasons.push("WebSocket disconnected");
    }
    return [...reasons, ...backendDisabledReasons.value];
  });

  const commandDisabled = computed(() => commandDisabledReasons.value.length > 0);

  function applyHealth(health: HealthResponse): void {
    backendOnline.value = health.backend_online;
    dispatchOnline.value = health.dispatch_online;
    dispatchDegraded.value = health.dispatch_degraded;
    updatedAt.value = health.updated_at;
    backendReason.value = health.reason;
    backendDisabledReasons.value = health.disabled_reasons ?? [];
  }

  function applyAggregateStatus(status: AggregateStatus): void {
    backendOnline.value = status.backend_online;
    dispatchOnline.value = status.dispatch_online;
    dispatchDegraded.value = status.dispatch_degraded;
    updatedAt.value = status.updated_at;
    backendDisabledReasons.value = status.disabled_reasons ?? [];
    if (typeof status.websocket_online === "boolean") {
      websocketOnline.value = status.websocket_online;
    }
  }

  async function refreshHealth(): Promise<void> {
    healthLoading.value = true;
    try {
      applyHealth(await apiClient.getHealth());
    } catch (error) {
      backendOnline.value = false;
      dispatchOnline.value = false;
      dispatchDegraded.value = true;
      backendReason.value = error instanceof Error ? error.message : "健康检查失败";
      backendDisabledReasons.value = [];
    } finally {
      healthLoading.value = false;
    }
  }

  function setWebSocketOnline(online: boolean): void {
    websocketOnline.value = online;
  }

  return {
    backendOnline,
    dispatchOnline,
    dispatchDegraded,
    dispatchReady,
    websocketOnline,
    updatedAt,
    backendReason,
    backendDisabledReasons,
    healthLoading,
    commandDisabled,
    commandDisabledReasons,
    applyHealth,
    applyAggregateStatus,
    refreshHealth,
    setWebSocketOnline
  };
});
