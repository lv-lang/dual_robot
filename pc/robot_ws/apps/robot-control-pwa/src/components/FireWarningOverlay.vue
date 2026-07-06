<script setup lang="ts">
import { AlertTriangle, Flame, PackageOpen } from "lucide-vue-next";
import type { ActiveDemoWarning } from "../stores/logs";

defineProps<{
  warning?: ActiveDemoWarning;
}>();
</script>

<template>
  <Teleport to="body">
    <div v-if="warning" class="demo-warning-backdrop" role="presentation">
      <section class="demo-warning" :class="warning.severity" role="alertdialog" aria-modal="true" :aria-label="warning.title">
        <div class="warning-topline">
          <AlertTriangle :size="34" aria-hidden="true" />
          <strong>{{ warning.title }}</strong>
          <Flame v-if="warning.severity === 'danger'" :size="34" aria-hidden="true" />
          <PackageOpen v-else :size="34" aria-hidden="true" />
        </div>
        <p>{{ warning.message }}</p>
      </section>
    </div>
  </Teleport>
</template>

<style scoped>
.demo-warning-backdrop {
  position: fixed;
  inset: 0;
  z-index: 80;
  display: grid;
  place-items: center;
  padding: 22px;
  background: rgb(6 8 7 / 72%);
}

.demo-warning {
  width: min(620px, 100%);
  border: 6px solid #ff2c24;
  border-radius: var(--radius-md);
  padding: 28px 30px;
  color: #ffffff;
  background: #620c0a;
  box-shadow:
    0 0 0 6px rgb(255 44 36 / 28%),
    0 22px 70px rgb(0 0 0 / 55%);
  text-align: center;
}

.demo-warning.danger {
  border-color: #ff2c24;
  background: #620c0a;
  box-shadow:
    0 0 0 6px rgb(255 44 36 / 28%),
    0 22px 70px rgb(0 0 0 / 55%);
  animation: dangerPulse 0.72s steps(2, end) infinite;
}

.demo-warning.warning {
  border-color: #f1c84b;
  background: #604600;
  box-shadow:
    0 0 0 6px rgb(241 200 75 / 34%),
    0 22px 70px rgb(0 0 0 / 48%);
  animation: warningPulse 0.72s steps(2, end) infinite;
}

.warning-topline {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 16px;
  color: #ffe45c;
}

.warning-topline strong {
  font-size: clamp(34px, 6vw, 58px);
  letter-spacing: 0;
  line-height: 1;
}

p {
  margin: 22px 0 12px;
  font-size: clamp(24px, 4vw, 38px);
  font-weight: 900;
  letter-spacing: 0;
}

@keyframes dangerPulse {
  0% {
    border-color: #ff2c24;
    background: #620c0a;
    transform: scale(1);
  }
  50% {
    border-color: #ffe45c;
    background: #a0110c;
    transform: scale(1.018);
  }
  100% {
    border-color: #ff2c24;
    background: #620c0a;
    transform: scale(1);
  }
}

@keyframes warningPulse {
  0% {
    border-color: #f1c84b;
    background: #604600;
    transform: scale(1);
  }
  50% {
    border-color: #ffffff;
    background: #937000;
    transform: scale(1.018);
  }
  100% {
    border-color: #f1c84b;
    background: #604600;
    transform: scale(1);
  }
}
</style>
