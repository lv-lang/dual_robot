import type { ConfirmationStep, TaskSummary } from "./types";

const BUSINESS_TASK_PREFIX: Record<string, string> = {
  DELIVERY: "配送任务",
  INSPECTION: "巡检任务",
  RECHECK: "复查任务"
};

type DisplayableTask = Pick<TaskSummary | ConfirmationStep, "task_id" | "task_type"> & {
  display_name?: string;
  label?: string;
};

function trimmed(value?: string): string {
  return value?.trim() ?? "";
}

function taskSequence(taskId: string): string {
  const match = taskId.match(/(?:task[-_])?0*(\d+)$/i);
  return match?.[1] ?? taskId;
}

export function genericTaskDisplayName(taskId?: string): string {
  const trimmedId = trimmed(taskId);
  if (!trimmedId) {
    return "任务";
  }
  return `任务_${taskSequence(trimmedId)}`;
}

export function taskDisplayName(task: DisplayableTask): string {
  const explicit = trimmed(task.display_name);
  if (explicit) {
    return explicit;
  }

  const prefix = BUSINESS_TASK_PREFIX[String(task.task_type || "").toUpperCase()];
  if (prefix) {
    const sequence = task.task_id ? taskSequence(task.task_id) : "";
    return sequence ? `${prefix}_${sequence}` : prefix;
  }

  return trimmed(task.label) || genericTaskDisplayName(task.task_id) || "未命名任务";
}
