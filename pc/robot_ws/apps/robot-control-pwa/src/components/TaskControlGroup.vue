<script setup lang="ts">
import { computed, ref } from "vue";
import { CirclePause, CirclePlay, XCircle } from "lucide-vue-next";
import DangerConfirmModal from "./DangerConfirmModal.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useLogsStore } from "../stores/logs";

const props = defineProps<{
  taskId: string;
  taskName?: string;
  taskStatus?: string;
  compact?: boolean;
}>();

const aggregate = useAggregateStore();
const connection = useConnectionStore();
const logs = useLogsStore();
const busyAction = ref<"pause" | "resume" | "cancel">();
const cancelOpen = ref(false);
const error = ref<string>();

const paused = computed(() => props.taskStatus === "PAUSED");
const disabled = computed(() => connection.commandDisabled || Boolean(busyAction.value));

async function run(action: "pause" | "resume" | "cancel"): Promise<void> {
  busyAction.value = action;
  error.value = undefined;
  try {
    if (action === "pause") {
      await aggregate.pauseTask(props.taskId);
    } else if (action === "resume") {
      await aggregate.resumeTask(props.taskId);
    } else {
      await aggregate.cancelTask(props.taskId);
      cancelOpen.value = false;
    }
    await logs.refreshLogs();
  } catch (caught) {
    error.value = caught instanceof Error ? caught.message : "任务控制失败";
  } finally {
    busyAction.value = undefined;
  }
}
</script>

<template>
  <div class="task-controls" :class="{ compact }">
    <button
      v-if="!paused"
      class="button-secondary"
      type="button"
      :disabled="disabled"
      title="暂停任务"
      @click="run('pause')"
    >
      <CirclePause :size="18" aria-hidden="true" />
      <span>暂停</span>
    </button>
    <button
      v-else
      class="button-secondary"
      type="button"
      :disabled="disabled"
      title="恢复任务"
      @click="run('resume')"
    >
      <CirclePlay :size="18" aria-hidden="true" />
      <span>恢复</span>
    </button>
    <button
      class="button-danger"
      type="button"
      :disabled="disabled"
      title="取消任务"
      @click="cancelOpen = true"
    >
      <XCircle :size="18" aria-hidden="true" />
      <span>取消</span>
    </button>
  </div>
  <p v-if="error" class="control-error">{{ error }}</p>
  <DangerConfirmModal
    :open="cancelOpen"
    title="取消任务"
    :body="`取消 ${taskName || '当前任务'}。机器人会停止当前任务，并由调度系统执行收尾处理。`"
    confirm-label="确认取消"
    :busy="busyAction === 'cancel'"
    @cancel="cancelOpen = false"
    @confirm="run('cancel')"
  />
</template>

<style scoped>
.task-controls {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
}

.task-controls button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

.task-controls.compact button {
  min-height: 38px;
}

.control-error {
  margin: 6px 0 0;
  color: var(--red);
  font-size: 13px;
  font-weight: 700;
}
</style>
