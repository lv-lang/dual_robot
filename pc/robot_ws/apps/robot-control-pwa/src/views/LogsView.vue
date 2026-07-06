<script setup lang="ts">
import { computed, ref } from "vue";
import { RefreshCw } from "lucide-vue-next";
import LogList from "../components/LogList.vue";
import { useLogsStore } from "../stores/logs";
import { logLevelLabel } from "../i18n";

const logs = useLogsStore();
const level = ref("all");

const filteredLogs = computed(() =>
  level.value === "all" ? logs.latest : logs.latest.filter((log) => log.level === level.value)
);
</script>

<template>
  <section class="stack">
    <div class="page-heading">
      <h1>事件日志</h1>
      <div class="button-row">
        <select v-model="level" aria-label="日志等级">
          <option value="all">全部</option>
          <option value="info">{{ logLevelLabel("info") }}</option>
          <option value="warning">{{ logLevelLabel("warning") }}</option>
          <option value="error">{{ logLevelLabel("error") }}</option>
        </select>
        <button class="button-secondary" type="button" :disabled="logs.loading" @click="logs.refreshLogs()">
          <RefreshCw :size="18" aria-hidden="true" />
          <span>刷新</span>
        </button>
      </div>
    </div>

    <p v-if="logs.error" class="notice error">{{ logs.error }}</p>

    <section class="panel">
      <LogList :logs="filteredLogs" />
    </section>
  </section>
</template>

<style scoped>
.button-row button {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

select {
  min-height: 42px;
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 0 10px;
  color: var(--primary);
  background: #ffffff;
  font-weight: 800;
}
</style>
