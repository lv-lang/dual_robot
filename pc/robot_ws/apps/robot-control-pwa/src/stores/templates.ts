import { computed, ref } from "vue";
import { defineStore } from "pinia";
import { apiClient } from "../api";
import { useAggregateStore } from "./aggregate";
import { useLogsStore } from "./logs";
import { taskDisplayName } from "../taskDisplay";
import type { BusinessPoint, RobotId, TaskSummary, TaskTemplate, TemplatePayload, TriggerTaskResult } from "../types";

const BUSY_TASK_TYPES = new Set(["DELIVERY", "INSPECTION", "RECHECK"]);
const BUSY_ROBOT_STATUSES = new Set([
  "ASSIGNED",
  "EXECUTING",
  "RUNNING",
  "WAITING_CONFIRMATION",
  "WAITING_RESOURCE",
  "PAUSED",
  "RESUMING",
  "ESTOP",
  "ERROR"
]);

function isRobotId(value: string): value is RobotId {
  return value === "mecanum" || value === "ackermann";
}

function isBusyBusinessTask(task?: TaskSummary): boolean {
  return Boolean(
    task &&
      BUSY_TASK_TYPES.has(String(task.task_type).toUpperCase()) &&
      BUSY_ROBOT_STATUSES.has(task.status.toUpperCase())
  );
}

export const useTemplatesStore = defineStore("templates", () => {
  const templates = ref<TaskTemplate[]>([]);
  const businessPoints = ref<BusinessPoint[]>([]);
  const loading = ref(false);
  const saving = ref(false);
  const error = ref<string>();
  const feedback = ref<string>();

  const orderedTemplates = computed(() =>
    [...templates.value].sort((left, right) => left.sort_order - right.sort_order)
  );
  const builtInTemplates = computed(() => orderedTemplates.value.filter((template) => template.readonly));
  const userTemplates = computed(() => orderedTemplates.value.filter((template) => !template.readonly));

  function preferenceFallbackWarning(template: TaskTemplate): string | undefined {
    if (!isRobotId(template.robot_preference)) {
      return undefined;
    }

    const aggregate = useAggregateStore();
    const robot = aggregate.robots.find((candidate) => candidate.robot_id === template.robot_preference);
    const currentTask = aggregate.tasks.find((task) => task.task_id === robot?.current_task_id);
    if (!robot || !isBusyBusinessTask(currentTask) || !BUSY_ROBOT_STATUSES.has(robot.status.toUpperCase())) {
      return undefined;
    }

    return `当前选择 ${template.robot_preference} 正在执行配送/巡检任务，调度系统将分配另一台机器人执行该任务。`;
  }

  async function loadTemplates(): Promise<void> {
    loading.value = true;
    try {
      const response = await apiClient.getTemplates();
      templates.value = response.templates;
      businessPoints.value = response.business_points;
      error.value = undefined;
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "模板加载失败";
    } finally {
      loading.value = false;
    }
  }

  async function createTemplate(payload: TemplatePayload): Promise<void> {
    saving.value = true;
    try {
      templates.value = [...templates.value, await apiClient.createTemplate(payload)];
      feedback.value = "模板已创建";
      error.value = undefined;
      await useLogsStore().refreshLogs();
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "模板创建失败";
      throw caught;
    } finally {
      saving.value = false;
    }
  }

  async function updateTemplate(templateId: string, payload: TemplatePayload): Promise<void> {
    saving.value = true;
    try {
      const updated = await apiClient.updateTemplate(templateId, payload);
      templates.value = templates.value.map((template) =>
        template.template_id === updated.template_id ? updated : template
      );
      feedback.value = "模板已更新";
      error.value = undefined;
      await useLogsStore().refreshLogs();
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "模板更新失败";
      throw caught;
    } finally {
      saving.value = false;
    }
  }

  async function deleteTemplate(templateId: string): Promise<void> {
    saving.value = true;
    try {
      await apiClient.deleteTemplate(templateId);
      templates.value = templates.value.filter((template) => template.template_id !== templateId);
      feedback.value = "模板已删除";
      error.value = undefined;
      await useLogsStore().refreshLogs();
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "模板删除失败";
      throw caught;
    } finally {
      saving.value = false;
    }
  }

  async function reorder(templateId: string, direction: -1 | 1): Promise<void> {
    const ordered = orderedTemplates.value;
    const index = ordered.findIndex((template) => template.template_id === templateId);
    const nextIndex = index + direction;
    if (index < 0 || nextIndex < 0 || nextIndex >= ordered.length) {
      return;
    }
    const copy = [...ordered];
    const [template] = copy.splice(index, 1);
    copy.splice(nextIndex, 0, template);
    const response = await apiClient.reorderTemplates(copy.map((item) => item.template_id));
    templates.value = response.templates;
    businessPoints.value = response.business_points;
  }

  async function triggerTemplate(templateId: string): Promise<TriggerTaskResult> {
    saving.value = true;
    try {
      const template = templates.value.find((candidate) => candidate.template_id === templateId);
      const warning = template ? preferenceFallbackWarning(template) : undefined;
      if (warning) {
        feedback.value = warning;
      }
      const result = await apiClient.triggerTemplate(templateId);
      await Promise.all([useAggregateStore().refreshState(), useLogsStore().refreshLogs()]);
      const aggregate = useAggregateStore();
      const createdTask = aggregate.tasks.find((task) => task.task_id === result.task_id);
      const displayName = taskDisplayName({
        task_id: result.task_id,
        task_type: createdTask?.task_type ?? template?.task_type ?? "UNKNOWN",
        display_name: result.display_name ?? createdTask?.display_name,
        label: createdTask?.label
      });
      const preferred = result.preferred_robot_id ?? template?.robot_preference;
      const assigned = result.assigned_robot_id ?? createdTask?.assigned_robot_id ?? createdTask?.robot_id;
      const assignedMessage =
        preferred && isRobotId(preferred) && assigned && assigned !== preferred
          ? `，已由 ${assigned} 执行`
          : "";
      feedback.value = `${warning ? `${warning} ` : ""}已创建 ${displayName}${assignedMessage}`;
      error.value = undefined;
      return result;
    } catch (caught) {
      error.value = caught instanceof Error ? caught.message : "模板触发失败";
      throw caught;
    } finally {
      saving.value = false;
    }
  }

  return {
    templates,
    businessPoints,
    orderedTemplates,
    builtInTemplates,
    userTemplates,
    loading,
    saving,
    error,
    feedback,
    loadTemplates,
    createTemplate,
    updateTemplate,
    deleteTemplate,
    reorder,
    triggerTemplate
  };
});
