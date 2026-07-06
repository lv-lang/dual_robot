import { computed, ref } from "vue";
import { defineStore } from "pinia";
import { apiClient } from "../api";
import type {
  LogEntry,
  SystemActionResponse,
  SystemLaunchLogLine,
  SystemLogsResponse,
  SystemStatusResponse
} from "../types";

interface RefreshOptions {
  silent?: boolean;
}

const emptyStatus = (): SystemStatusResponse => ({
  status: "stopped",
  summary: "调度系统未启动",
  managed: false,
  external_running: false,
  can_start: true,
  can_stop: false,
  can_restart: false,
  profile: {
    id: "real_robot_control_plane",
    name: "Real Robot Control Plane",
    command: "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
  },
  pid: null,
  pgid: null,
  started_at: "",
  updated_at: new Date(0).toISOString(),
  health: []
});

export const useSystemStore = defineStore("system", () => {
  const status = ref<SystemStatusResponse>(emptyStatus());
  const launchLogs = ref<SystemLaunchLogLine[]>([]);
  const operationLogs = ref<LogEntry[]>([]);
  const loading = ref(false);
  const logsLoading = ref(false);
  const actionBusy = ref(false);
  const actionName = ref<"start" | "stop" | "restart" | undefined>();
  const error = ref<string>();
  const logsError = ref<string>();
  const actionMessage = ref<string>();

  const canStart = computed(() => status.value.can_start && !actionBusy.value);
  const canStop = computed(() => status.value.can_stop && !status.value.external_running && !actionBusy.value);
  const canRestart = computed(() => status.value.can_restart && !status.value.external_running && !actionBusy.value);
  const ownershipLabel = computed(() => {
    if (status.value.external_running) {
      return "外部运行";
    }
    if (status.value.managed) {
      return "App-managed";
    }
    return "停止";
  });

  function applyStatus(nextStatus: SystemStatusResponse): void {
    status.value = nextStatus;
  }

  function applyLogs(nextLogs: SystemLogsResponse): void {
    launchLogs.value = nextLogs.launch_logs;
    operationLogs.value = nextLogs.operation_logs;
  }

  async function refreshStatus(options: RefreshOptions = {}): Promise<void> {
    if (!options.silent) {
      loading.value = true;
    }
    try {
      applyStatus(await apiClient.getSystemStatus());
      error.value = undefined;
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "系统状态刷新失败";
    } finally {
      if (!options.silent) {
        loading.value = false;
      }
    }
  }

  async function refreshLogs(limit = 120, options: RefreshOptions = {}): Promise<void> {
    if (!options.silent) {
      logsLoading.value = true;
    }
    try {
      applyLogs(await apiClient.getSystemLogs(limit));
      logsError.value = undefined;
    } catch (caught) {
      logsError.value = caught instanceof Error ? caught.message : "系统日志刷新失败";
    } finally {
      if (!options.silent) {
        logsLoading.value = false;
      }
    }
  }

  async function refreshAll(options: RefreshOptions = {}): Promise<void> {
    await Promise.allSettled([refreshStatus(options), refreshLogs(120, options)]);
  }

  async function runAction(name: "start" | "stop" | "restart"): Promise<void> {
    actionBusy.value = true;
    actionName.value = name;
    error.value = undefined;
    try {
      const response = await callAction(name);
      applyStatus(response.status);
      actionMessage.value = response.message;
      await refreshLogs();
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "系统操作失败";
      throw caught;
    } finally {
      actionBusy.value = false;
      actionName.value = undefined;
    }
  }

  function callAction(name: "start" | "stop" | "restart"): Promise<SystemActionResponse> {
    if (name === "start") {
      return apiClient.startSystem();
    }
    if (name === "stop") {
      return apiClient.stopSystem();
    }
    return apiClient.restartSystem();
  }

  return {
    status,
    launchLogs,
    operationLogs,
    loading,
    logsLoading,
    actionBusy,
    actionName,
    error,
    logsError,
    actionMessage,
    canStart,
    canStop,
    canRestart,
    ownershipLabel,
    applyStatus,
    applyLogs,
    refreshStatus,
    refreshLogs,
    refreshAll,
    start: () => runAction("start"),
    stop: () => runAction("stop"),
    restart: () => runAction("restart")
  };
});
