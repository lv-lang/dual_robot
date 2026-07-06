<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from "vue";
import type { MapData, RobotSummary, TaskPoint, TaskSummary } from "../types";

const props = defineProps<{
  robots: RobotSummary[];
  tasks: TaskSummary[];
}>();

const mapData = ref<MapData | null>(null);
const points = ref<TaskPoint[]>([]);
const loadError = ref<string>("");
// 每车保留近 N 个真实位姿用于轨迹拖尾
const trails = reactive<Record<string, { x: number; y: number }[]>>({});
const TRAIL_MAX = 150;
const TRAIL_MIN_STEP = 0.04; // m, 移动超过才记一点

let pointsTimer: number | undefined;

async function fetchJson<T>(path: string): Promise<T> {
  const res = await fetch(path, { headers: { Accept: "application/json" } });
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return (await res.json()) as T;
}

async function loadMap() {
  try {
    mapData.value = await fetchJson<MapData>("/api/map");
  } catch (err) {
    loadError.value = `地图加载失败: ${String(err)}`;
  }
}

async function loadPoints() {
  try {
    const data = await fetchJson<{ points: TaskPoint[] }>("/api/task-points");
    points.value = (data.points ?? []).filter((p) => p.has_pose);
  } catch {
    /* 保留旧点 */
  }
}

onMounted(() => {
  loadMap();
  loadPoints();
  pointsTimer = window.setInterval(loadPoints, 10000);
});
onBeforeUnmount(() => {
  if (pointsTimer) window.clearInterval(pointsTimer);
});

// 位姿更新时追加轨迹
watch(
  () => props.robots,
  (robots) => {
    for (const robot of robots) {
      if (!robot.pose) continue;
      const trail = (trails[robot.robot_id] ||= []);
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(robot.pose.x - last.x, robot.pose.y - last.y) >= TRAIL_MIN_STEP) {
        trail.push({ x: robot.pose.x, y: robot.pose.y });
        if (trail.length > TRAIL_MAX) trail.shift();
      }
    }
  },
  { deep: true, immediate: true }
);

// 世界坐标 -> 底图像素 (origin 为左下角像素的世界坐标; 图像 y 轴向下)
const viewW = computed(() => mapData.value?.width ?? 640);
const viewH = computed(() => mapData.value?.height ?? 640);

function toPx(x: number, y: number): { x: number; y: number } {
  const map = mapData.value;
  if (!map || !map.resolution) return { x: 0, y: 0 };
  const [ox, oy] = map.origin;
  return {
    x: (x - ox) / map.resolution,
    y: (map.height ?? viewH.value) - (y - oy) / map.resolution
  };
}

const pointIndex = computed(() => {
  const idx: Record<string, TaskPoint> = {};
  for (const p of points.value) idx[p.point_id] = p;
  return idx;
});

function pointClass(kind: string): string {
  const k = (kind || "").toLowerCase();
  if (k.includes("waiting")) return "pt-waiting";
  if (k.includes("pickup")) return "pt-pickup";
  if (k.includes("delivery")) return "pt-delivery";
  if (k.includes("inspection")) return "pt-inspection";
  return "pt-other";
}

const screenPoints = computed(() =>
  points.value.map((p) => ({ ...p, screen: toPx(p.x, p.y), cls: pointClass(p.kind) }))
);

function activeTaskFor(robot: RobotSummary): TaskSummary | undefined {
  return props.tasks.find(
    (t) =>
      (t.assigned_robot_id === robot.robot_id || t.robot_id === robot.robot_id) &&
      !["COMPLETED", "SUCCEEDED", "CANCELED", "CANCELLED", "FAILED"].includes(
        (t.status || "").toUpperCase()
      )
  );
}

interface RobotView {
  robot: RobotSummary;
  screen: { x: number; y: number };
  rotationDeg: number;
  routePixels: string;
  target?: { screen: { x: number; y: number }; label: string; distance: number };
  trailPixels: string;
  stepLabel: string;
}

const robotViews = computed<RobotView[]>(() => {
  return props.robots
    .filter((robot) => robot.pose)
    .map((robot) => {
      const pose = robot.pose!;
      const screen = toPx(pose.x, pose.y);
      // 世界 yaw(逆时针,+x为0) -> 屏幕(图像y向下)旋转: 取负
      const rotationDeg = (-pose.yaw * 180) / Math.PI;

      const task = activeTaskFor(robot);
      const remaining = (task?.target_points ?? [])
        .map((id) => pointIndex.value[id])
        .filter((p): p is TaskPoint => Boolean(p));

      const routeCoords = [{ x: pose.x, y: pose.y }, ...remaining.map((p) => ({ x: p.x, y: p.y }))];
      const routePixels = routeCoords
        .map((c) => {
          const s = toPx(c.x, c.y);
          return `${s.x.toFixed(1)},${s.y.toFixed(1)}`;
        })
        .join(" ");

      let target: RobotView["target"];
      const next = remaining[0];
      if (next) {
        target = {
          screen: toPx(next.x, next.y),
          label: next.label || next.point_id,
          distance: Math.hypot(next.x - pose.x, next.y - pose.y)
        };
      }

      const trail = trails[robot.robot_id] ?? [];
      const trailPixels = trail
        .map((c) => {
          const s = toPx(c.x, c.y);
          return `${s.x.toFixed(1)},${s.y.toFixed(1)}`;
        })
        .join(" ");

      return {
        robot,
        screen,
        rotationDeg,
        routePixels,
        target,
        trailPixels,
        stepLabel: task?.current_step_label || ""
      };
    });
});

// 活动任务点位状态: 顺序编号 + 进度阶段(已过/当前/待去) + 当前到目标距离
type PointPhase = "done" | "current" | "pending";
interface PointActive {
  order: number;
  phase: PointPhase;
  robotId: string;
  distance?: number;
}

const activePointStates = computed<Record<string, PointActive>>(() => {
  const out: Record<string, PointActive> = {};
  for (const robot of props.robots) {
    const task = activeTaskFor(robot);
    if (!task) continue;
    const steps = task.steps ?? [];
    // 业务导航点(排除返回等待区), 按顺序去重
    const ordered: string[] = [];
    const seen = new Set<string>();
    for (const s of steps) {
      if (!s.point_id || s.step_type === "RETURN_HOME") continue;
      if (seen.has(s.point_id)) continue;
      seen.add(s.point_id);
      ordered.push(s.point_id);
    }
    if (!ordered.length) continue;
    const curStep = steps[task.current_step_index ?? 0];
    const currentPointId = curStep?.point_id ?? "";
    const curIdx = ordered.indexOf(currentPointId);
    ordered.forEach((pid, i) => {
      let phase: PointPhase = "pending";
      if (curIdx >= 0) phase = i < curIdx ? "done" : i === curIdx ? "current" : "pending";
      const entry: PointActive = { order: i + 1, phase, robotId: robot.robot_id };
      if (phase === "current" && robot.pose) {
        const pt = pointIndex.value[pid];
        if (pt) entry.distance = Math.hypot(pt.x - robot.pose.x, pt.y - robot.pose.y);
      }
      out[pid] = entry;
    });
  }
  return out;
});

const robotColor = (id: string) => (id === "mecanum" ? "#2dd4bf" : "#f59e0b");
</script>

<template>
  <div class="sitmap">
    <svg
      v-if="mapData"
      class="sitmap-svg"
      :viewBox="`0 0 ${viewW} ${viewH}`"
      preserveAspectRatio="xMidYMid slice"
    >
      <!-- 真实 SLAM 底图 -->
      <image
        v-if="mapData.available && mapData.image_base64"
        :href="`data:${mapData.image_mime};base64,${mapData.image_base64}`"
        x="0"
        y="0"
        :width="viewW"
        :height="viewH"
        style="image-rendering: pixelated"
      />
      <rect v-else x="0" y="0" :width="viewW" :height="viewH" fill="#0b1220" />

      <!-- 任务点: 活动任务的点位高亮+编号+进度, 其余淡化 -->
      <g
        v-for="p in screenPoints"
        :key="p.point_id"
        class="point"
        :class="[p.cls, activePointStates[p.point_id] ? `phase-${activePointStates[p.point_id].phase}` : 'idle']"
      >
        <circle
          v-if="activePointStates[p.point_id] && activePointStates[p.point_id].phase === 'current'"
          class="current-ring"
          :cx="p.screen.x"
          :cy="p.screen.y"
          r="15"
          fill="none"
          :stroke="robotColor(activePointStates[p.point_id].robotId)"
        />
        <circle class="dot" :cx="p.screen.x" :cy="p.screen.y" :r="activePointStates[p.point_id] ? 9 : 6" />
        <text class="point-label" :x="p.screen.x + 10" :y="p.screen.y - 10">
          {{ p.point_id }}
        </text>
        <template v-if="activePointStates[p.point_id]">
          <circle
            class="order-bg"
            :cx="p.screen.x"
            :cy="p.screen.y - 18"
            r="8.5"
            :fill="robotColor(activePointStates[p.point_id].robotId)"
          />
          <text class="order-num" :x="p.screen.x" :y="p.screen.y - 14" text-anchor="middle">
            {{ activePointStates[p.point_id].order }}
          </text>
          <text
            v-if="activePointStates[p.point_id].distance !== undefined"
            class="dist-num"
            :x="p.screen.x + 17"
            :y="p.screen.y + 4"
          >
            {{ activePointStates[p.point_id].distance!.toFixed(2) }}m
          </text>
        </template>
      </g>

      <!-- 每车: 轨迹 / 剩余路径 / 目标 / 车体 -->
      <g v-for="view in robotViews" :key="view.robot.robot_id">
        <polyline
          v-if="view.trailPixels"
          :points="view.trailPixels"
          fill="none"
          :stroke="robotColor(view.robot.robot_id)"
          stroke-width="2"
          stroke-opacity="0.45"
          stroke-dasharray="1 4"
        />
        <polyline
          v-if="view.routePixels"
          :points="view.routePixels"
          fill="none"
          :stroke="robotColor(view.robot.robot_id)"
          stroke-width="2.5"
          stroke-opacity="0.85"
          stroke-dasharray="7 5"
        />
        <g :transform="`translate(${view.screen.x} ${view.screen.y}) rotate(${view.rotationDeg})`">
          <!-- 朝向箭头 (+x 方向) -->
          <polygon points="16,0 4,-6 4,6" :fill="robotColor(view.robot.robot_id)" />
          <rect
            x="-10"
            y="-8"
            width="20"
            height="16"
            rx="3"
            :fill="robotColor(view.robot.robot_id)"
            fill-opacity="0.85"
            stroke="#0b1220"
            stroke-width="1.5"
          />
        </g>
      </g>
    </svg>

    <div v-else class="sitmap-empty">{{ loadError || "加载地图中…" }}</div>
    <div v-if="mapData && !mapData.available" class="sitmap-warn">
      底图不可用（{{ mapData.reason || "无图像" }}）— 显示任务点与车位
    </div>
  </div>
</template>

<style scoped>
.sitmap {
  position: relative;
  width: 100%;
  height: 100%;
  min-height: 360px;
  background: #0b1220;
  border: 1px solid #1e293b;
  border-radius: 6px;
  overflow: hidden;
}
.sitmap-svg {
  width: 100%;
  height: 100%;
  display: block;
}
.sitmap-empty {
  display: flex;
  align-items: center;
  justify-content: center;
  height: 100%;
  color: #64748b;
  font-size: 14px;
}
.sitmap-warn {
  position: absolute;
  left: 8px;
  bottom: 8px;
  padding: 2px 8px;
  font-size: 12px;
  color: #fbbf24;
  background: rgba(11, 18, 32, 0.8);
  border-radius: 4px;
}
.point .dot {
  stroke: #0b1220;
  stroke-width: 1.5;
}
.point-label {
  fill: #dce7e2;
  font-size: 12px;
  font-weight: 800;
  paint-order: stroke;
  stroke: #05070d;
  stroke-width: 3px;
}
.pt-waiting .dot {
  fill: #64748b;
}
.pt-pickup .dot {
  fill: #38bdf8;
}
.pt-delivery .dot {
  fill: #a78bfa;
}
.pt-inspection .dot {
  fill: #f472b6;
}
.pt-other .dot {
  fill: #94a3b8;
}

/* 空闲点位淡化, 活动任务点位高亮 */
.point.idle {
  opacity: 0.4;
}
.phase-done .dot {
  fill-opacity: 0.45;
}
.phase-current .dot {
  stroke: #fff;
  stroke-width: 2;
}

.current-ring {
  stroke-width: 2.5;
  animation: ring-pulse 1.4s ease-in-out infinite;
}
@keyframes ring-pulse {
  0%, 100% { opacity: 0.5; }
  50% { opacity: 1; }
}

.order-bg {
  stroke: #0b1220;
  stroke-width: 1;
}
.order-num {
  font-size: 11px;
  font-weight: 800;
  fill: #06121f;
}
.dist-num {
  font-size: 11.5px;
  font-weight: 700;
  fill: #e2e8f0;
  paint-order: stroke;
  stroke: #05070d;
  stroke-width: 3px;
}
</style>
