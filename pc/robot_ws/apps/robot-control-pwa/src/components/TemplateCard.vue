<script setup lang="ts">
import { computed, ref } from "vue";
import { ArrowDown, ArrowUp, Pencil, Play, Trash2 } from "lucide-vue-next";
import DangerConfirmModal from "./DangerConfirmModal.vue";
import StatusPill from "./StatusPill.vue";
import type { TaskTemplate } from "../types";
import { robotPreferenceLabel, taskTypeLabel } from "../i18n";

const props = defineProps<{
  template: TaskTemplate;
  disabled?: boolean;
  first?: boolean;
  last?: boolean;
}>();

const emit = defineEmits<{
  trigger: [templateId: string];
  edit: [template: TaskTemplate];
  delete: [templateId: string];
  move: [templateId: string, direction: -1 | 1];
}>();

const deleteOpen = ref(false);
const isUnavailable = computed(() => props.template.available === false);
const triggerDisabled = computed(() => Boolean(props.disabled) || isUnavailable.value);
const missingPointIds = computed(() => props.template.missing_point_ids ?? []);
const unavailableMessage = computed(() => {
  if (!isUnavailable.value) {
    return "";
  }
  if (missingPointIds.value.length) {
    return `缺少业务点：${missingPointIds.value.join("、")}`;
  }
  return "当前模板引用的业务点不可用";
});
</script>

<template>
  <article class="card template-card">
    <div class="row between">
      <div class="template-title">
        <h3>{{ template.name }}</h3>
        <div class="row wrap">
          <StatusPill :label="taskTypeLabel(template.task_type)" tone="info" />
          <StatusPill :label="robotPreferenceLabel(template.robot_preference)" tone="neutral" />
          <StatusPill :label="template.readonly ? '内置' : '自定义'" :tone="template.readonly ? 'ok' : 'warn'" />
          <StatusPill v-if="isUnavailable" label="不可用" tone="bad" />
        </div>
      </div>
      <div class="order-controls">
        <button
          class="icon-button button-secondary"
          type="button"
          title="上移模板"
          :disabled="first || template.readonly"
          @click="emit('move', template.template_id, -1)"
        >
          <ArrowUp :size="17" aria-hidden="true" />
        </button>
        <button
          class="icon-button button-secondary"
          type="button"
          title="下移模板"
          :disabled="last || template.readonly"
          @click="emit('move', template.template_id, 1)"
        >
          <ArrowDown :size="17" aria-hidden="true" />
        </button>
      </div>
    </div>

    <div class="point-list">
      <span v-for="point in template.target_points" :key="point">{{ point }}</span>
    </div>

    <p v-if="isUnavailable" class="template-warning">{{ unavailableMessage }}</p>

    <div class="button-row">
      <button type="button" :disabled="triggerDisabled" title="触发模板" @click="emit('trigger', template.template_id)">
        <Play :size="18" aria-hidden="true" />
        <span>触发</span>
      </button>
      <button
        v-if="!template.readonly"
        class="button-secondary"
        type="button"
        title="编辑模板"
        @click="emit('edit', template)"
      >
        <Pencil :size="18" aria-hidden="true" />
        <span>编辑</span>
      </button>
      <button
        v-if="!template.readonly"
        class="button-danger"
        type="button"
        title="删除模板"
        @click="deleteOpen = true"
      >
        <Trash2 :size="18" aria-hidden="true" />
        <span>删除</span>
      </button>
    </div>

    <DangerConfirmModal
      :open="deleteOpen"
      title="删除模板"
      :body="`删除 ${template.name}。已经创建的任务不会受到影响。`"
      confirm-label="确认删除"
      @cancel="deleteOpen = false"
      @confirm="emit('delete', template.template_id); deleteOpen = false"
    />
  </article>
</template>

<style scoped>
.template-card {
  display: grid;
  gap: 12px;
}

.template-title {
  min-width: 0;
}

h3 {
  margin: 0 0 8px;
  font-size: 18px;
  letter-spacing: 0;
}

.order-controls {
  display: flex;
  gap: 6px;
}

.point-list {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
}

.point-list span {
  border-radius: 999px;
  padding: 6px 9px;
  color: var(--primary);
  background: var(--surface-muted);
  font-weight: 800;
  font-size: 13px;
}

.template-warning {
  margin: 0;
  color: var(--red);
  font-size: 13px;
  font-weight: 800;
  line-height: 1.35;
  overflow-wrap: anywhere;
}

.button-row button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}
</style>
