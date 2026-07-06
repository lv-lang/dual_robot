<script setup lang="ts">
import { onMounted, reactive, ref } from "vue";
import type { CameraFeed } from "../types";

const feeds = ref<CameraFeed[]>([]);
// 记录加载失败的流, 退回占位
const failed = reactive<Record<string, boolean>>({});

async function loadCameras() {
  try {
    const res = await fetch("/api/cameras", { headers: { Accept: "application/json" } });
    if (!res.ok) return;
    const data = (await res.json()) as { cameras?: CameraFeed[] };
    feeds.value = data.cameras ?? [];
  } catch {
    /* 保留占位 */
  }
}

function onError(robotId: string) {
  failed[robotId] = true;
}

onMounted(loadCameras);
</script>

<template>
  <div class="camera-panel">
    <div class="camera-grid">
      <figure v-for="feed in feeds" :key="feed.robot_id" class="camera-cell" :class="`cam-${feed.robot_id}`">
        <figcaption class="camera-cap">
          <span class="dot" />
          {{ feed.label }}
        </figcaption>
        <img
          v-if="feed.stream_url && !failed[feed.robot_id]"
          class="camera-stream"
          :src="feed.stream_url"
          :alt="feed.label"
          @error="onError(feed.robot_id)"
        />
        <div v-else class="camera-placeholder">
          <span class="ph-icon">▦</span>
          <span class="ph-text">摄像头未接入</span>
          <span class="ph-sub">待视觉集成</span>
        </div>
      </figure>
    </div>
  </div>
</template>

<style scoped>
.camera-panel {
  height: 100%;
  display: flex;
  flex-direction: column;
  min-height: 0;
}
.camera-grid {
  flex: 1;
  min-height: 0;
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}
.camera-cell {
  position: relative;
  margin: 0;
  min-height: 0;
  border: 1px solid #243042;
  border-radius: 6px;
  background: #05070d;
  overflow: hidden;
  display: flex;
}
.cam-mecanum {
  border-color: rgba(45, 212, 191, 0.5);
}
.cam-ackermann {
  border-color: rgba(245, 158, 11, 0.5);
}
.camera-cap {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  z-index: 2;
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 5px 9px;
  font-size: 13px;
  font-weight: 600;
  color: #e2e8f0;
  background: linear-gradient(180deg, rgba(2, 6, 16, 0.85), transparent);
}
.camera-cap .dot {
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: #ef4444;
  box-shadow: 0 0 6px #ef4444;
}
.camera-stream {
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}
.camera-placeholder {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 4px;
  color: #475569;
  background:
    repeating-linear-gradient(45deg, #0a0f1a, #0a0f1a 10px, #0c1220 10px, #0c1220 20px);
}
.ph-icon {
  font-size: 30px;
  opacity: 0.5;
}
.ph-text {
  font-size: 14px;
  font-weight: 600;
  color: #64748b;
}
.ph-sub {
  font-size: 12px;
  color: #475569;
}
</style>
