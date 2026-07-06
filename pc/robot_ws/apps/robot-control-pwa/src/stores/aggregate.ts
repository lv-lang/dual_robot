import { computed, ref } from "vue";
import { defineStore } from "pinia";
import { apiClient } from "../api";
import { useConnectionStore } from "./connection";
import type {
  AggregateState,
  ConfirmationResult,
  ConfirmationStep,
  ResourceLock,
  RobotSummary,
  TaskSummary
} from "../types";

const TERMINAL_TASK_STATUSES = ["COMPLETED", "SUCCEEDED", "CANCELED", "FAILED"];

const emptyState = (): AggregateState => ({
  status: {
    backend_online: false,
    dispatch_online: false,
    dispatch_degraded: true,
    websocket_online: false,
    disabled_reasons: [],
    updated_at: new Date(0).toISOString()
  },
  robots: [],
  tasks: [],
  resource_locks: [],
  waiting_confirmations: []
});

export const useAggregateStore = defineStore("aggregate", () => {
  const state = ref<AggregateState>(emptyState());
  const loading = ref(false);
  const error = ref<string>();
  const actionMessage = ref<string>();

  const robots = computed<RobotSummary[]>(() => state.value.robots);
  const tasks = computed<TaskSummary[]>(() => state.value.tasks);
  const activeTasks = computed<TaskSummary[]>(() =>
    state.value.tasks.filter((task) => !TERMINAL_TASK_STATUSES.includes(task.status))
  );
  const resourceLocks = computed<ResourceLock[]>(() => state.value.resource_locks);
  const waitingConfirmations = computed<ConfirmationStep[]>(() => state.value.waiting_confirmations);

  function applyState(nextState: AggregateState): void {
    state.value = nextState;
    useConnectionStore().applyAggregateStatus(nextState.status);
  }

  async function refreshState(): Promise<void> {
    loading.value = true;
    try {
      applyState(await apiClient.getState());
      error.value = undefined;
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "状态刷新失败";
    } finally {
      loading.value = false;
    }
  }

  async function pauseTask(taskId: string): Promise<void> {
    actionMessage.value = (await apiClient.pauseTask(taskId)).message;
    await refreshState();
  }

  async function resumeTask(taskId: string): Promise<void> {
    actionMessage.value = (await apiClient.resumeTask(taskId)).message;
    await refreshState();
  }

  async function cancelTask(taskId: string): Promise<void> {
    actionMessage.value = (await apiClient.cancelTask(taskId)).message;
    await refreshState();
  }

  async function confirmStep(step: ConfirmationStep, result: ConfirmationResult): Promise<void> {
    actionMessage.value = (
      await apiClient.confirmStep({
        task_id: step.task_id,
        step_index: step.step_index,
        step_id: step.step_id,
        point_id: step.point_id,
        result
      })
    ).message;
    await refreshState();
  }

  async function emergencyStop(): Promise<void> {
    actionMessage.value = (await apiClient.emergencyStop()).message;
    await refreshState();
  }

  return {
    state,
    loading,
    error,
    actionMessage,
    robots,
    tasks,
    activeTasks,
    resourceLocks,
    waitingConfirmations,
    applyState,
    refreshState,
    pauseTask,
    resumeTask,
    cancelTask,
    confirmStep,
    emergencyStop
  };
});
