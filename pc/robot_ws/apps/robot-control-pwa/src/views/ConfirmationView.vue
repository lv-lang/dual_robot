<script setup lang="ts">
import { ref } from "vue";
import { AlertTriangle, Check, RefreshCw, X } from "lucide-vue-next";
import StatusPill from "../components/StatusPill.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useLogsStore } from "../stores/logs";
import type { ConfirmationResult, ConfirmationStep } from "../types";
import { commandReasonLabel, confirmationResultLabel, taskTypeLabel } from "../i18n";
import { taskDisplayName } from "../taskDisplay";

const aggregate = useAggregateStore();
const connection = useConnectionStore();
const logs = useLogsStore();
const busyKey = ref<string>();
const error = ref<string>();

async function refresh(): Promise<void> {
  await aggregate.refreshState();
}

async function confirm(step: ConfirmationStep, result: ConfirmationResult): Promise<void> {
  busyKey.value = `${step.task_id}-${result}`;
  error.value = undefined;
  try {
    await aggregate.confirmStep(step, result);
    await logs.refreshLogs();
  } catch (caught) {
    error.value = caught instanceof Error ? caught.message : "确认失败";
  } finally {
    busyKey.value = undefined;
  }
}

function isDeliveryConfirmation(step: ConfirmationStep): boolean {
  return step.task_type === "DELIVERY";
}

function isDropoffStep(step: ConfirmationStep): boolean {
  const text = `${step.step_id} ${step.label} ${step.point_id} ${step.point_label}`.toLowerCase();
  return step.step_index === 1 || text.includes("dropoff") || text.includes("delivery") || text.includes("卸货");
}

function deliveryConfirmationLabel(step: ConfirmationStep): string {
  return isDropoffStep(step) ? "已卸货" : "已取货";
}
</script>

<template>
  <section class="stack">
    <div class="page-heading">
      <h1>待确认</h1>
      <button class="button-secondary" type="button" :disabled="aggregate.loading" @click="refresh">
        <RefreshCw :size="18" aria-hidden="true" />
        <span>刷新</span>
      </button>
    </div>

    <p v-if="connection.commandDisabled" class="notice">
      {{ connection.commandDisabledReasons.map(commandReasonLabel).join("；") }}
    </p>
    <p v-if="error || aggregate.error" class="notice error">{{ error || aggregate.error }}</p>

    <div v-if="aggregate.waitingConfirmations.length" class="confirmation-grid">
      <article v-for="step in aggregate.waitingConfirmations" :key="`${step.task_id}-${step.step_id}`" class="card confirmation-card">
        <div class="row between wrap">
          <div>
            <h2>{{ taskDisplayName(step) }}</h2>
            <p>{{ step.label }} / {{ step.point_label }}</p>
          </div>
          <div class="row wrap">
            <StatusPill :label="taskTypeLabel(step.task_type)" tone="info" />
            <StatusPill v-if="step.robot_id" :label="step.robot_id" tone="neutral" />
          </div>
        </div>

        <div class="confirm-actions" :class="{ 'single-action': isDeliveryConfirmation(step) }">
          <button
            v-if="isDeliveryConfirmation(step)"
            type="button"
            :disabled="connection.commandDisabled || Boolean(busyKey)"
            :title="deliveryConfirmationLabel(step)"
            @click="confirm(step, 'OK')"
          >
            <Check :size="20" aria-hidden="true" />
            <span>{{ deliveryConfirmationLabel(step) }}</span>
          </button>
          <button
            v-else
            type="button"
            :disabled="connection.commandDisabled || Boolean(busyKey)"
            title="确认正常"
            @click="confirm(step, 'OK')"
          >
            <Check :size="20" aria-hidden="true" />
            <span>{{ confirmationResultLabel('OK') }}</span>
          </button>
          <button
            v-if="!isDeliveryConfirmation(step)"
            class="button-warning"
            type="button"
            :disabled="connection.commandDisabled || Boolean(busyKey)"
            title="确认异常"
            @click="confirm(step, 'ABNORMAL')"
          >
            <AlertTriangle :size="20" aria-hidden="true" />
            <span>{{ confirmationResultLabel('ABNORMAL') }}</span>
          </button>
          <button
            v-if="!isDeliveryConfirmation(step)"
            class="button-danger"
            type="button"
            :disabled="connection.commandDisabled || Boolean(busyKey)"
            title="拒绝确认"
            @click="confirm(step, 'REJECT')"
          >
            <X :size="20" aria-hidden="true" />
            <span>{{ confirmationResultLabel('REJECT') }}</span>
          </button>
        </div>
      </article>
    </div>
    <p v-else class="empty">暂无待确认事项</p>
  </section>
</template>

<style scoped>
.page-heading button,
.confirm-actions button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

.confirmation-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.confirmation-card {
  display: grid;
  gap: 16px;
}

h2 {
  margin: 0;
  font-size: 22px;
  letter-spacing: 0;
}

p {
  margin: 4px 0 0;
  color: var(--muted);
}

.confirm-actions {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 8px;
}

.confirm-actions.single-action {
  grid-template-columns: 1fr;
}

.confirm-actions button {
  justify-content: center;
  min-height: 54px;
  font-weight: 900;
}

@media (max-width: 900px) {
  .confirmation-grid,
  .confirm-actions {
    grid-template-columns: 1fr;
  }
}
</style>
