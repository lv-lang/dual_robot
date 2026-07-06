<script setup lang="ts">
import { computed } from "vue";
import StatusPill from "./StatusPill.vue";
import TaskControlGroup from "./TaskControlGroup.vue";
import { useAggregateStore } from "../stores/aggregate";
import type { RobotSummary } from "../types";
import { chassisLabel, statusLabel } from "../i18n";
import { taskDisplayName } from "../taskDisplay";

const props = defineProps<{
  robot: RobotSummary;
}>();

const aggregate = useAggregateStore();
const currentTask = computed(() =>
  aggregate.tasks.find((task) => task.task_id === props.robot.current_task_id)
);

const statusTone = computed(() => {
  const status = props.robot.status.toUpperCase();
  if (status.includes("ERROR") || status.includes("EMERGENCY")) {
    return "bad";
  }
  if (status.includes("PAUSED") || status.includes("WAITING")) {
    return "warn";
  }
  if (status.includes("IDLE") || status.includes("EXECUTING")) {
    return "ok";
  }
  return "neutral";
});
</script>

<template>
  <article class="card robot-card">
    <div class="row between">
      <div>
        <h2>{{ robot.display_name }}</h2>
        <p>{{ chassisLabel(robot.chassis_type) }}</p>
      </div>
      <StatusPill :label="statusLabel(robot.status)" :tone="statusTone" />
    </div>

    <dl>
      <div>
        <dt>任务</dt>
        <dd>{{ currentTask ? taskDisplayName(currentTask) : robot.current_task_id ? "任务运行中" : "无" }}</dd>
      </div>
      <div>
        <dt>说明</dt>
        <dd>{{ robot.current_task_label || currentTask?.current_step_label || "空闲" }}</dd>
      </div>
      <div>
        <dt>更新</dt>
        <dd>{{ robot.last_update ? new Date(robot.last_update).toLocaleTimeString() : "未知" }}</dd>
      </div>
    </dl>

    <TaskControlGroup
      v-if="robot.current_task_id"
      :task-id="robot.current_task_id"
      :task-name="currentTask ? taskDisplayName(currentTask) : '当前任务'"
      :task-status="currentTask?.status || robot.status"
      compact
    />
  </article>
</template>

<style scoped>
.robot-card {
  display: grid;
  gap: 14px;
}

h2 {
  margin: 0;
  font-size: 24px;
  letter-spacing: 0;
}

p {
  margin: 4px 0 0;
  color: var(--muted);
  text-transform: capitalize;
}

dl {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 8px;
  margin: 0;
}

dt {
  color: var(--muted);
  font-size: 12px;
  font-weight: 800;
  text-transform: uppercase;
}

dd {
  margin: 4px 0 0;
  overflow-wrap: anywhere;
  font-weight: 800;
}

@media (max-width: 700px) {
  dl {
    grid-template-columns: 1fr;
  }
}
</style>
