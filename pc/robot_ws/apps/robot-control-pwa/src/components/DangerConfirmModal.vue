<script setup lang="ts">
import { AlertTriangle } from "lucide-vue-next";

defineProps<{
  open: boolean;
  title: string;
  body: string;
  confirmLabel: string;
  busy?: boolean;
  tone?: "danger" | "warning";
}>();

const emit = defineEmits<{
  cancel: [];
  confirm: [];
}>();
</script>

<template>
  <Teleport to="body">
    <div v-if="open" class="modal-backdrop" role="presentation">
      <section class="modal" :class="tone === 'warning' ? 'warning' : 'danger'" role="dialog" aria-modal="true" :aria-label="title">
        <div class="modal-title">
          <AlertTriangle :size="24" aria-hidden="true" />
          <h2>{{ title }}</h2>
        </div>
        <p>{{ body }}</p>
        <div class="modal-actions">
          <button class="button-secondary" type="button" title="返回" :disabled="busy" @click="emit('cancel')">
            返回
          </button>
          <button
            type="button"
            :class="tone === 'warning' ? 'button-warning' : 'button-danger'"
            :title="confirmLabel"
            :disabled="busy"
            @click="emit('confirm')"
          >
            {{ confirmLabel }}
          </button>
        </div>
      </section>
    </div>
  </Teleport>
</template>

<style scoped>
.modal-backdrop {
  position: fixed;
  inset: 0;
  z-index: 50;
  display: grid;
  place-items: center;
  padding: 20px;
  background: rgb(7 13 11 / 78%);
}

.modal {
  width: min(460px, 100%);
  border: 3px solid var(--red);
  border-radius: var(--radius-md);
  padding: 20px;
  background: #ffffff;
  box-shadow: 0 0 0 4px rgb(0 0 0 / 28%);
}

.modal.warning {
  border-color: var(--amber);
}

.modal-title {
  display: flex;
  align-items: center;
  gap: 10px;
  color: var(--red);
}

.modal.warning .modal-title {
  color: var(--amber-dark);
}

h2 {
  margin: 0;
  font-size: 20px;
  letter-spacing: 0;
}

p {
  margin: 14px 0 20px;
  color: var(--muted);
  line-height: 1.45;
}

.modal-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}

.modal-actions button {
  min-width: 112px;
}

@media (max-width: 520px) {
  .modal-actions {
    flex-direction: column-reverse;
  }

  .modal-actions button {
    width: 100%;
  }
}
</style>
