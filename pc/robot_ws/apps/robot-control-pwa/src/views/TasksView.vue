<script setup lang="ts">
import { computed, ref } from "vue";
import { Plus, RefreshCw } from "lucide-vue-next";
import TemplateCard from "../components/TemplateCard.vue";
import TemplateForm from "../components/TemplateForm.vue";
import TaskSummaryList from "../components/TaskSummaryList.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useTemplatesStore } from "../stores/templates";
import { commandReasonLabel } from "../i18n";
import type { TaskTemplate, TemplatePayload } from "../types";

const templates = useTemplatesStore();
const aggregate = useAggregateStore();
const connection = useConnectionStore();

const editing = ref<TaskTemplate>();
const creating = ref(false);
const operationError = ref<string>();

const showForm = computed(() => creating.value || Boolean(editing.value));

async function refresh(): Promise<void> {
  await Promise.all([templates.loadTemplates(), aggregate.refreshState()]);
}

function newTemplate(): void {
  editing.value = undefined;
  creating.value = true;
}

function closeForm(): void {
  editing.value = undefined;
  creating.value = false;
}

async function saveTemplate(payload: TemplatePayload): Promise<void> {
  operationError.value = undefined;
  try {
    if (editing.value) {
      await templates.updateTemplate(editing.value.template_id, payload);
    } else {
      await templates.createTemplate(payload);
    }
    closeForm();
  } catch (caught) {
    operationError.value = caught instanceof Error ? caught.message : "模板保存失败";
  }
}

async function trigger(templateId: string): Promise<void> {
  operationError.value = undefined;
  try {
    await templates.triggerTemplate(templateId);
  } catch (caught) {
    operationError.value = caught instanceof Error ? caught.message : "模板触发失败";
  }
}

async function remove(templateId: string): Promise<void> {
  operationError.value = undefined;
  try {
    await templates.deleteTemplate(templateId);
  } catch (caught) {
    operationError.value = caught instanceof Error ? caught.message : "模板删除失败";
  }
}
</script>

<template>
  <section class="stack">
    <div class="page-heading">
      <h1>任务</h1>
      <div class="button-row">
        <button class="button-secondary" type="button" :disabled="templates.loading" @click="refresh">
          <RefreshCw :size="18" aria-hidden="true" />
          <span>刷新</span>
        </button>
        <button type="button" @click="newTemplate">
          <Plus :size="18" aria-hidden="true" />
          <span>新建模板</span>
        </button>
      </div>
    </div>

    <p v-if="connection.commandDisabled" class="notice">
      {{ connection.commandDisabledReasons.map(commandReasonLabel).join("；") }}
    </p>
    <p v-if="templates.feedback" class="notice">{{ templates.feedback }}</p>
    <p v-if="templates.error || operationError" class="notice error">
      {{ operationError || templates.error }}
    </p>

    <TemplateForm
      v-if="showForm"
      :template="editing"
      :business-points="templates.businessPoints"
      :saving="templates.saving"
      @save="saveTemplate"
      @cancel="closeForm"
    />

    <section class="panel stack">
      <h2 class="section-title">任务模板</h2>
      <div v-if="templates.orderedTemplates.length" class="template-grid">
        <TemplateCard
          v-for="(template, index) in templates.orderedTemplates"
          :key="template.template_id"
          :template="template"
          :first="index === 0"
          :last="index === templates.orderedTemplates.length - 1"
          :disabled="connection.commandDisabled || templates.saving"
          @trigger="trigger"
          @edit="(templateToEdit) => { editing = templateToEdit; creating = false; }"
          @delete="remove"
          @move="templates.reorder"
        />
      </div>
      <p v-else class="empty">暂无模板</p>
    </section>

    <section class="panel stack">
      <h2 class="section-title">任务队列</h2>
      <TaskSummaryList :tasks="aggregate.tasks" show-controls />
    </section>
  </section>
</template>

<style scoped>
.button-row button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

.template-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

@media (max-width: 900px) {
  .template-grid {
    grid-template-columns: 1fr;
  }
}
</style>
