<script setup lang="ts">
import StatusPill from "./StatusPill.vue";
import TaskControlGroup from "./TaskControlGroup.vue";
import type { TaskSummary } from "../types";
import { statusLabel, taskTypeLabel } from "../i18n";
import { taskDisplayName } from "../taskDisplay";

defineProps<{
  tasks: TaskSummary[];
  showControls?: boolean;
}>();
</script>

<template>
  <div v-if="tasks.length" class="task-list">
    <article v-for="task in tasks" :key="task.task_id" class="task-row">
      <div class="task-main">
        <div class="row wrap">
          <strong>{{ taskDisplayName(task) }}</strong>
          <StatusPill :label="statusLabel(task.status)" :tone="task.status === 'FAILED' ? 'bad' : 'info'" />
        </div>
        <div class="task-meta">
          <span>{{ taskTypeLabel(task.task_type) }}</span>
          <span>{{ task.robot_id || "未分配" }}</span>
          <span>{{ task.current_step_label || "无当前步骤" }}</span>
        </div>
      </div>
      <TaskControlGroup
        v-if="showControls && !['COMPLETED', 'SUCCEEDED', 'CANCELED', 'FAILED'].includes(task.status)"
        :task-id="task.task_id"
        :task-name="taskDisplayName(task)"
        :task-status="task.status"
        compact
      />
    </article>
  </div>
  <p v-else class="empty">暂无任务</p>
</template>

<style scoped>
.task-list {
  display: grid;
  gap: 10px;
}

.task-row {
  display: grid;
  grid-template-columns: 1fr auto;
  gap: 12px;
  align-items: center;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 12px;
  background: #ffffff;
}

.task-main {
  min-width: 0;
}

.task-meta {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  margin-top: 8px;
  color: var(--muted);
  font-size: 13px;
}

@media (max-width: 760px) {
  .task-row {
    grid-template-columns: 1fr;
  }
}
</style>
