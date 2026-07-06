<script setup lang="ts">
defineProps<{
  label: string;
  tone?: "ok" | "warn" | "bad" | "neutral" | "info";
}>();
</script>

<template>
  <span class="status-pill" :class="tone || 'neutral'" :title="label">
    {{ label }}
  </span>
</template>

<style scoped>
.status-pill {
  display: inline-flex;
  align-items: center;
  min-height: 30px;
  border: 2px solid transparent;
  border-radius: var(--radius-sm);
  padding: 0 9px;
  font-size: 12px;
  font-weight: 800;
  line-height: 1;
  white-space: nowrap;
}

.ok {
  color: #ffffff;
  border-color: #0b5f30;
  background: var(--green);
}

.warn {
  color: #1e1604;
  border-color: var(--amber-dark);
  background: var(--amber);
}

.bad {
  color: #ffffff;
  border-color: var(--red-dark);
  background: var(--red);
}

.neutral {
  color: var(--primary);
  border-color: var(--line);
  background: var(--surface-muted);
}

.info {
  color: #ffffff;
  border-color: #0b4f8d;
  background: var(--blue);
}
</style>
