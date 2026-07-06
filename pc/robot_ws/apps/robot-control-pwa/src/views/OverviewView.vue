<script setup lang="ts">
import { computed, ref } from "vue";
import {
  Activity,
  CheckCircle2,
  Cpu,
  MapPinned,
  Play,
  Radio,
  RefreshCw,
  Server
} from "lucide-vue-next";
import StatusPill from "../components/StatusPill.vue";
import SituationMap from "../components/SituationMap.vue";
import CameraPanel from "../components/CameraPanel.vue";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useLogsStore } from "../stores/logs";
import { useTemplatesStore } from "../stores/templates";
import {
  chassisLabel,
  commandReasonLabel,
  statusLabel,
  taskTypeLabel
} from "../i18n";
import { genericTaskDisplayName, taskDisplayName } from "../taskDisplay";
import type { ResourceLock, RobotId, RobotSummary, TaskSummary, TaskTemplate } from "../types";

type FieldPointId = "W1" | "W2" | "A" | "C" | "P1" | "P2" | "P3";
type FieldPointType = "waiting" | "pickup" | "delivery" | "inspection";
type StatusTone = "ok" | "warn" | "bad" | "neutral" | "info";

interface FieldPoint {
  id: FieldPointId;
  label: string;
  x: number;
  y: number;
  yaw: number;
  type: FieldPointType;
}

interface SvgPoint {
  x: number;
  y: number;
}

interface RobotSituation {
  robot: RobotSummary;
  robotId: RobotId;
  reported: boolean;
  task?: TaskSummary;
  poseId: FieldPointId;
  screen: SvgPoint;
  headingDeg: number;
  tone: StatusTone;
  statusText: string;
  taskText: string;
  markerClass: string;
}

const aggregate = useAggregateStore();
const connection = useConnectionStore();
const logs = useLogsStore();
const templates = useTemplatesStore();
const operationError = ref<string>();

const FIELD_WIDTH = 1000;
const FIELD_HEIGHT = 560;
const FIELD_PAD = 60;
const FIELD_BOUNDS = {
  minX: -10,
  maxX: -6.5,
  minY: -4.8,
  maxY: 3.4
};

const FIELD_POINTS: Record<FieldPointId, FieldPoint> = {
  W1: { id: "W1", label: "W1 等待区", x: -8.5, y: -3.8, yaw: 0, type: "waiting" },
  A: { id: "A", label: "A 取货点", x: -7.5, y: -3.8, yaw: 0, type: "pickup" },
  C: { id: "C", label: "C 配送点", x: -9.0, y: -3.8, yaw: 0, type: "delivery" },
  W2: { id: "W2", label: "W2 等待区", x: -8.5, y: 2.5, yaw: 0, type: "waiting" },
  P1: { id: "P1", label: "P1 巡检点", x: -7.5, y: 2.5, yaw: 0, type: "inspection" },
  P2: { id: "P2", label: "P2 巡检点", x: -8.0, y: 2.5, yaw: 0, type: "inspection" },
  P3: { id: "P3", label: "P3 巡检点", x: -9.0, y: 2.5, yaw: 0, type: "inspection" }
};

const G1_ROUTES: Record<RobotId, FieldPointId[]> = {
  mecanum: ["W1", "A", "C", "W1"],
  ackermann: ["W2", "P1", "P2", "P3", "W2"]
};

const DEFAULT_ROBOTS: RobotSummary[] = [
  {
    robot_id: "mecanum",
    display_name: "mecanum",
    chassis_type: "mecanum",
    status: "OFFLINE"
  },
  {
    robot_id: "ackermann",
    display_name: "ackermann",
    chassis_type: "ackermann",
    status: "OFFLINE"
  }
];


function isFieldPointId(value?: string): value is FieldPointId {
  return Boolean(value && Object.prototype.hasOwnProperty.call(FIELD_POINTS, value));
}

function svgPointFor(point: Pick<FieldPoint, "x" | "y">): SvgPoint {
  const usableWidth = FIELD_WIDTH - FIELD_PAD * 2;
  const usableHeight = FIELD_HEIGHT - FIELD_PAD * 2;
  return {
    x: FIELD_PAD + ((point.x - FIELD_BOUNDS.minX) / (FIELD_BOUNDS.maxX - FIELD_BOUNDS.minX)) * usableWidth,
    y: FIELD_HEIGHT - FIELD_PAD - ((point.y - FIELD_BOUNDS.minY) / (FIELD_BOUNDS.maxY - FIELD_BOUNDS.minY)) * usableHeight
  };
}

function taskRobotId(task?: TaskSummary): RobotId | undefined {
  return task?.assigned_robot_id ?? task?.robot_id;
}

function taskMatchesRobot(task: TaskSummary, robot: RobotSummary): boolean {
  return (
    task.task_id === robot.current_task_id ||
    task.robot_id === robot.robot_id ||
    task.assigned_robot_id === robot.robot_id
  );
}

function activeTaskForRobot(robot: RobotSummary): TaskSummary | undefined {
  if (robot.current_task_id) {
    const current = aggregate.tasks.find((task) => task.task_id === robot.current_task_id);
    if (current) {
      return current;
    }
  }
  return aggregate.activeTasks.find((task) => taskMatchesRobot(task, robot));
}

function confirmationForRobot(robot: RobotSummary, task?: TaskSummary) {
  return aggregate.waitingConfirmations.find(
    (step) =>
      step.robot_id === robot.robot_id ||
      step.assigned_robot_id === robot.robot_id ||
      Boolean(task && step.task_id === task.task_id)
  );
}

function lockForRobot(robot: RobotSummary, task?: TaskSummary): ResourceLock | undefined {
  return aggregate.resourceLocks.find(
    (lock) =>
      lock.robot_id === robot.robot_id ||
      Boolean(task && lock.holder_task_id === task.task_id)
  );
}

function firstFieldPoint(points: string[] = []): FieldPointId | undefined {
  return points.find(isFieldPointId);
}

function poseForRobot(robot: RobotSummary, task?: TaskSummary): FieldPointId {
  const confirmation = confirmationForRobot(robot, task);
  if (isFieldPointId(confirmation?.point_id)) {
    return confirmation.point_id;
  }

  const lock = lockForRobot(robot, task);
  if (isFieldPointId(lock?.point_id)) {
    return lock.point_id;
  }

  const taskPoint = firstFieldPoint(task?.target_points);
  if (taskPoint) {
    return taskPoint;
  }

  return robot.robot_id === "mecanum" ? "W1" : "W2";
}

function headingBetween(fromId: FieldPointId, toId?: FieldPointId): number {
  if (!toId || fromId === toId) {
    return FIELD_POINTS[fromId].yaw;
  }
  const from = svgPointFor(FIELD_POINTS[fromId]);
  const to = svgPointFor(FIELD_POINTS[toId]);
  return Math.atan2(to.y - from.y, to.x - from.x) * (180 / Math.PI);
}

function headingForRobot(robot: RobotSummary, poseId: FieldPointId, task?: TaskSummary): number {
  const taskTarget = firstFieldPoint(task?.target_points.filter((pointId) => pointId !== poseId));
  if (taskTarget) {
    return headingBetween(poseId, taskTarget);
  }

  const route = G1_ROUTES[robot.robot_id];
  const routeIndex = route.indexOf(poseId);
  const nextRoutePoint = routeIndex >= 0 ? route[routeIndex + 1] ?? route[routeIndex - 1] : undefined;
  return headingBetween(poseId, nextRoutePoint);
}

function isFaultStatus(status: string): boolean {
  const normalized = status.toUpperCase();
  return (
    normalized.includes("ESTOP") ||
    normalized.includes("ERROR") ||
    normalized.includes("FAULT") ||
    normalized.includes("FAILED") ||
    normalized.includes("EMERGENCY")
  );
}

function isWaitingStatus(status: string): boolean {
  const normalized = status.toUpperCase();
  return normalized.includes("WAITING") || normalized.includes("PAUSED") || normalized.includes("DEGRADED");
}

function robotTone(robot: RobotSummary, reported: boolean): StatusTone {
  if (!reported) {
    return "neutral";
  }
  if (isFaultStatus(robot.status)) {
    return "bad";
  }
  if (isWaitingStatus(robot.status)) {
    return "warn";
  }
  return "ok";
}

function robotStatusText(robot: RobotSummary, reported: boolean): string {
  return reported ? statusLabel(robot.status) : "未上报";
}

function taskIsAbnormal(task: TaskSummary): boolean {
  const normalized = task.status.toUpperCase();
  return normalized.includes("FAILED") || normalized.includes("ERROR") || normalized.includes("ABNORMAL");
}

function resourceLockTaskName(lock: ResourceLock): string {
  const task = aggregate.tasks.find((candidate) => candidate.task_id === lock.holder_task_id);
  return task ? taskDisplayName(task) : genericTaskDisplayName(lock.holder_task_id);
}

function quickTemplateDetail(template: TaskTemplate): string {
  const robot = template.robot_preference === "auto" ? "自动分配" : template.robot_preference;
  return `${taskTypeLabel(template.task_type)} / ${robot} / ${template.target_points.join(" -> ")}`;
}

async function refresh(): Promise<void> {
  operationError.value = undefined;
  await Promise.all([
    connection.refreshHealth(),
    aggregate.refreshState(),
    templates.loadTemplates(),
    logs.refreshLogs()
  ]);
}

async function triggerQuickTemplate(template: TaskTemplate): Promise<void> {
  operationError.value = undefined;
  try {
    await templates.triggerTemplate(template.template_id);
  } catch (caught) {
    operationError.value = caught instanceof Error ? caught.message : "快捷任务触发失败";
  }
}

const reportedRobotIds = computed(() => new Set(aggregate.robots.map((robot) => robot.robot_id)));

const displayedRobots = computed(() =>
  DEFAULT_ROBOTS.map((defaultRobot) => {
    const reported = aggregate.robots.find((robot) => robot.robot_id === defaultRobot.robot_id);
    return reported ?? defaultRobot;
  })
);

const robotSituations = computed<RobotSituation[]>(() =>
  displayedRobots.value.map((robot) => {
    const reported = reportedRobotIds.value.has(robot.robot_id);
    const task = activeTaskForRobot(robot);
    const poseId = poseForRobot(robot, task);
    const tone = robotTone(robot, reported);
    return {
      robot,
      robotId: robot.robot_id,
      reported,
      task,
      poseId,
      screen: svgPointFor(FIELD_POINTS[poseId]),
      headingDeg: headingForRobot(robot, poseId, task),
      tone,
      statusText: robotStatusText(robot, reported),
      taskText: task ? taskDisplayName(task) : "无当前任务",
      markerClass: `vehicle-${robot.robot_id} vehicle-${robot.chassis_type} tone-${tone}`
    };
  })
);

const quickTemplates = computed(() =>
  templates.orderedTemplates.filter((template) => template.available !== false).slice(0, 4)
);

const faultCount = computed(
  () =>
    robotSituations.value.filter((robot) => robot.tone === "bad").length +
    aggregate.tasks.filter(taskIsAbnormal).length +
    logs.latest.filter((log) => log.level === "error").length
);

const globalTone = computed<StatusTone>(() => {
  if (faultCount.value > 0 || connection.commandDisabledReasons.includes("global_estop_active")) {
    return "bad";
  }
  if (aggregate.waitingConfirmations.length > 0 || connection.dispatchDegraded) {
    return "warn";
  }
  if (connection.backendOnline && connection.dispatchOnline) {
    return "ok";
  }
  return "bad";
});

const globalStatusLabel = computed(() => {
  if (globalTone.value === "bad") {
    return "急停/故障";
  }
  if (aggregate.waitingConfirmations.length > 0) {
    return "等待确认";
  }
  if (connection.dispatchDegraded) {
    return "调度降级";
  }
  return "运行正常";
});
</script>

<template>
  <section class="overview-hmi" aria-labelledby="overview-title">
    <div class="hmi-topbar">
      <div class="hmi-title-block">
        <span class="hmi-eyebrow">现场态势面板</span>
        <h1 id="overview-title">态势</h1>
        <p>二维场地态势图 / 双车实车部署 / 共享 /map</p>
      </div>

      <div class="global-status-strip" aria-label="全局状态">
        <StatusPill :label="globalStatusLabel" :tone="globalTone" />
        <span class="status-chip">
          <Server :size="15" aria-hidden="true" />
          后端 {{ connection.backendOnline ? "在线" : "离线" }}
        </span>
        <span class="status-chip">
          <Activity :size="15" aria-hidden="true" />
          调度 {{ connection.dispatchOnline ? (connection.dispatchDegraded ? "降级" : "在线") : "离线" }}
        </span>
        <span class="status-chip">
          <Radio :size="15" aria-hidden="true" />
          WS {{ connection.websocketOnline ? "在线" : "断开" }}
        </span>
      </div>

      <button class="hmi-refresh" type="button" :disabled="aggregate.loading || templates.loading || logs.loading" @click="refresh">
        <RefreshCw :size="18" aria-hidden="true" />
        <span>刷新</span>
      </button>
    </div>

    <p v-if="aggregate.error || templates.error || logs.error || operationError" class="hmi-notice bad">
      {{ operationError || aggregate.error || templates.error || logs.error }}
    </p>
    <p v-else-if="connection.commandDisabled" class="hmi-notice warn">
      {{ connection.commandDisabledReasons.map(commandReasonLabel).join("；") }}
    </p>
    <p v-else-if="templates.feedback" class="hmi-notice ok">{{ templates.feedback }}</p>

    <div class="hmi-layout">
      <aside class="hmi-panel left-rail" aria-label="机器人和设备状态">
        <div class="panel-heading">
          <div>
            <span class="panel-kicker">left rail</span>
            <h2>机器人 / 设备</h2>
          </div>
          <Cpu :size="21" aria-hidden="true" />
        </div>

        <div class="robot-stack">
          <article
            v-for="robot in robotSituations"
            :key="robot.robotId"
            class="robot-status-card"
            :class="robot.robotId"
          >
            <div class="robot-status-head">
              <div>
                <strong>{{ robot.robot.display_name }}</strong>
                <span>{{ chassisLabel(robot.robot.chassis_type) }}</span>
              </div>
              <StatusPill :label="robot.statusText" :tone="robot.tone" />
            </div>

            <div class="robot-line">
              <span class="robot-color-swatch" aria-hidden="true"></span>
              <span>{{ robot.taskText }}</span>
            </div>

            <dl class="robot-facts">
              <div>
                <dt>位置源</dt>
                <dd>{{ robot.reported ? "聚合状态推导" : "默认等待区" }}</dd>
              </div>
              <div>
                <dt>态势点</dt>
                <dd>{{ poseForRobot(robot.robot, robot.task) }}</dd>
              </div>
              <div>
                <dt>话题</dt>
                <dd>/{{ robot.robotId }}/scan /{{ robot.robotId }}/odom</dd>
              </div>
            </dl>
          </article>
        </div>

        <div class="device-contract">
          <div class="contract-row">
            <CheckCircle2 :size="17" aria-hidden="true" />
            <span>共享场地地图，双车统一坐标</span>
          </div>
          <div class="contract-row">
            <CheckCircle2 :size="17" aria-hidden="true" />
            <span>双车独立定位，各自里程计与底盘</span>
          </div>
          <div class="contract-row">
            <CheckCircle2 :size="17" aria-hidden="true" />
            <span>mecanum 与 ackermann 命名空间隔离</span>
          </div>
        </div>

        <div class="quick-task-panel">
          <div class="panel-subhead">
            <span>首屏快捷任务</span>
            <strong>{{ quickTemplates.length }}/4</strong>
          </div>
          <div v-if="quickTemplates.length" class="quick-task-list">
            <button
              v-for="template in quickTemplates"
              :key="template.template_id"
              class="quick-task-button"
              type="button"
              :disabled="connection.commandDisabled || templates.saving || template.available === false"
              :title="quickTemplateDetail(template)"
              @click="triggerQuickTemplate(template)"
            >
              <Play :size="16" aria-hidden="true" />
              <span>
                <strong>{{ template.name }}</strong>
                <small>{{ quickTemplateDetail(template) }}</small>
              </span>
            </button>
          </div>
          <p v-else class="hmi-empty">暂无可用快捷模板</p>
        </div>
      </aside>

      <section class="hmi-panel map-panel" aria-label="二维场地态势图">
        <div class="panel-heading">
          <div>
            <span class="panel-kicker">center situation map</span>
            <h2>二维场地态势图</h2>
          </div>
          <MapPinned :size="22" aria-hidden="true" />
        </div>

        <div class="map-frame">
          <SituationMap :robots="aggregate.robots" :tasks="aggregate.tasks" />
        </div>

        <div class="map-legend" aria-label="标志含义">
          <span><i class="legend-swatch robot-mecanum"></i>mecanum 车</span>
          <span><i class="legend-swatch robot-ackermann"></i>ackermann 车</span>
          <span><i class="legend-swatch pt-waiting"></i>等待区</span>
          <span><i class="legend-swatch pt-pickup"></i>取货点</span>
          <span><i class="legend-swatch pt-delivery"></i>配送点</span>
          <span><i class="legend-swatch pt-inspection"></i>巡检点</span>
          <span><i class="legend-line current"></i>剩余路径</span>
          <span><i class="legend-ring"></i>当前目标</span>
          <span><i class="legend-badge">1</i>任务执行顺序</span>
        </div>
      </section>

      <aside class="hmi-panel right-rail" aria-label="视觉画面 待确认与任务">
        <section class="camera-section">
          <div class="panel-subhead">
            <span>实时视觉画面</span>
            <strong>双车检测画面 · MJPEG 流</strong>
          </div>
          <div class="camera-wrap">
            <CameraPanel />
          </div>
        </section>

        <section class="confirmation-priority">
          <div class="panel-subhead urgent">
            <h2>待确认</h2>
            <strong>{{ aggregate.waitingConfirmations.length }}</strong>
          </div>
          <div v-if="aggregate.waitingConfirmations.length" class="confirmation-list">
            <article
              v-for="step in aggregate.waitingConfirmations"
              :key="`${step.task_id}-${step.step_id}`"
              class="confirmation-row"
            >
              <div>
                <strong>{{ taskDisplayName(step) }}</strong>
                <span>{{ step.label }} / {{ step.point_label }}</span>
              </div>
              <div class="row-meta">
                <StatusPill :label="taskTypeLabel(step.task_type)" tone="info" />
                <StatusPill :label="step.robot_id || step.assigned_robot_id || '未分配'" tone="warn" />
              </div>
            </article>
          </div>
          <p v-else class="hmi-empty">当前无等待确认</p>
        </section>

        <section class="active-tasks">
          <div class="panel-subhead">
            <h2>当前任务</h2>
            <strong>{{ aggregate.activeTasks.length }}</strong>
          </div>
          <div v-if="aggregate.activeTasks.length" class="active-task-list">
            <article v-for="task in aggregate.activeTasks" :key="task.task_id" class="active-task-row">
              <strong>{{ taskDisplayName(task) }}</strong>
              <span>{{ taskTypeLabel(task.task_type) }} / {{ taskRobotId(task) || "未分配" }} / {{ task.current_step_label || task.target_points.join(" → ") }}</span>
            </article>
          </div>
          <p v-else class="hmi-empty">暂无当前任务</p>
        </section>

        <section class="resource-locks">
          <div class="panel-subhead">
            <span>资源锁</span>
            <strong>{{ aggregate.resourceLocks.length }}</strong>
          </div>
          <div v-if="aggregate.resourceLocks.length" class="lock-list">
            <article
              v-for="lock in aggregate.resourceLocks"
              :key="`${lock.holder_task_id}-${lock.point_id}`"
              class="lock-row"
            >
              <strong>{{ lock.point_label }}</strong>
              <span>{{ lock.lock_type }} / {{ lock.robot_id || "未分配" }}</span>
              <small>{{ resourceLockTaskName(lock) }}</small>
            </article>
          </div>
          <p v-else class="hmi-empty">暂无资源锁</p>
        </section>
      </aside>
    </div>
  </section>
</template>

<style scoped>
.overview-hmi {
  --hmi-bg: #101513;
  --hmi-panel: #151d1b;
  --hmi-panel-2: #1b2422;
  --hmi-line: #33413d;
  --hmi-line-strong: #60736c;
  --hmi-text: #edf5f1;
  --hmi-muted: #9eaea8;
  --hmi-green: #31c76a;
  --hmi-blue: #2f85ff;
  --hmi-gray: #7f8a92;
  --hmi-yellow: #f1c84b;
  --hmi-orange: #ff9f2f;
  --hmi-red: #e23d36;
  --hmi-cyan: #44d7c1;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 12px;
  border: 0;
  border-radius: 0;
  padding: 14px 16px 16px;
  overflow: hidden;
  font-size: 15px;
  background:
    linear-gradient(135deg, rgb(47 133 255 / 8%), transparent 28%),
    linear-gradient(180deg, #121917 0%, #0e1412 100%);
  color: var(--hmi-text);
}

.hmi-topbar,
.hmi-layout,
.hmi-panel,
.robot-status-card,
.diagnosis-card,
.event-row,
.confirmation-row,
.lock-row,
.quick-task-button,
.vision-box,
.route-context article {
  min-width: 0;
}

.hmi-topbar {
  display: grid;
  grid-template-columns: minmax(240px, 1fr) auto auto;
  align-items: center;
  gap: 12px;
  border: 1px solid var(--hmi-line);
  border-radius: 8px;
  padding: 12px;
  background: linear-gradient(90deg, #18221f 0%, #111715 100%);
}

.hmi-title-block {
  display: grid;
  gap: 4px;
}

.hmi-eyebrow,
.panel-kicker {
  color: var(--hmi-cyan);
  font-size: 11px;
  font-weight: 900;
  letter-spacing: 0;
  text-transform: uppercase;
}

.hmi-title-block h1,
.panel-heading h2 {
  margin: 0;
  letter-spacing: 0;
}

.hmi-title-block h1 {
  font-size: 28px;
  line-height: 1;
}

.hmi-title-block p {
  margin: 0;
  color: var(--hmi-muted);
  font-size: 13px;
  line-height: 1.35;
}

.global-status-strip {
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 8px;
  flex-wrap: wrap;
}

.status-chip {
  min-height: 30px;
  display: inline-flex;
  align-items: center;
  gap: 6px;
  border: 1px solid var(--hmi-line);
  border-radius: 6px;
  padding: 0 9px;
  color: var(--hmi-text);
  background: #101816;
  font-size: 12px;
  font-weight: 800;
  white-space: nowrap;
}

.hmi-refresh {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 7px;
  border-color: #3d5950;
  color: #07110d;
  background: var(--hmi-green);
  font-weight: 900;
}

.hmi-refresh:hover:not(:disabled) {
  color: #ffffff;
  background: #21864b;
}

.hmi-notice {
  margin: 0;
  border: 1px solid var(--hmi-line);
  border-left-width: 5px;
  border-radius: 8px;
  padding: 10px 12px;
  font-weight: 800;
}

.hmi-notice.bad {
  border-left-color: var(--hmi-red);
  background: rgb(226 61 54 / 14%);
}

.hmi-notice.warn {
  border-left-color: var(--hmi-orange);
  background: rgb(255 159 47 / 14%);
}

.hmi-notice.ok {
  border-left-color: var(--hmi-green);
  background: rgb(49 199 106 / 12%);
}

.hmi-layout {
  flex: 1;
  min-height: 0;
  display: grid;
  grid-template-columns: minmax(250px, 0.72fr) minmax(520px, 2fr) minmax(380px, 1.2fr);
  gap: 12px;
  align-items: stretch;
  overflow: hidden;
}

.hmi-panel {
  position: relative;
  overflow: hidden;
  border: 1px solid var(--hmi-line);
  border-radius: 8px;
  padding: 12px;
  background:
    linear-gradient(90deg, rgb(47 133 255 / 6%), transparent 22%),
    linear-gradient(180deg, var(--hmi-panel), var(--hmi-panel-2));
}

.hmi-panel::before {
  content: "";
  position: absolute;
  inset: 0 auto 0 0;
  width: 4px;
  background: linear-gradient(180deg, var(--hmi-blue), #3745a8, var(--hmi-red));
}

.panel-heading {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 10px;
  margin-bottom: 12px;
  padding-left: 4px;
}

.panel-heading.compact {
  margin-bottom: 10px;
}

.panel-heading h2 {
  margin-top: 3px;
  font-size: 18px;
  line-height: 1.15;
}

.left-rail,
.right-rail {
  display: flex;
  flex-direction: column;
  gap: 12px;
  min-height: 0;
  overflow-y: auto;
}

/* 右栏: 摄像头固定高度在上, 其余可滚动 */
.camera-section {
  display: flex;
  flex-direction: column;
  gap: 8px;
  flex: 0 0 auto;
}

.camera-wrap {
  height: clamp(190px, 26vh, 320px);
}

.active-task-list {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.active-task-row {
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 7px 9px;
  border: 1px solid var(--hmi-line);
  border-radius: 6px;
  background: var(--hmi-panel-2);
}

.active-task-row strong {
  font-size: 14px;
}

.active-task-row span {
  font-size: 12.5px;
  color: var(--hmi-muted);
}

.robot-stack,
.quick-task-list,
.confirmation-list,
.lock-list,
.event-strip {
  display: grid;
  gap: 9px;
}

.robot-status-card {
  display: grid;
  gap: 10px;
  border: 1px solid var(--hmi-line);
  border-radius: 8px;
  padding: 11px;
  background: #101816;
}

.robot-status-card.mecanum {
  border-color: rgb(47 133 255 / 62%);
}

.robot-status-card.ackermann {
  border-color: rgb(49 199 106 / 62%);
}

.robot-status-head {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  gap: 10px;
}

.robot-status-head strong {
  display: block;
  font-size: 19px;
  line-height: 1.1;
}

.robot-status-head span {
  display: block;
  margin-top: 3px;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 800;
}

.robot-line {
  display: grid;
  grid-template-columns: auto 1fr;
  align-items: center;
  gap: 8px;
  color: var(--hmi-text);
  font-size: 13px;
  font-weight: 800;
  line-height: 1.3;
}

.robot-color-swatch {
  width: 12px;
  height: 12px;
  border-radius: 3px;
  background: var(--hmi-blue);
}

.ackermann .robot-color-swatch {
  background: var(--hmi-green);
}

.robot-facts {
  display: grid;
  grid-template-columns: 1fr;
  gap: 7px;
  margin: 0;
}

.robot-facts div {
  display: grid;
  grid-template-columns: 54px minmax(0, 1fr);
  gap: 6px;
}

.robot-facts dt,
.robot-facts dd {
  margin: 0;
  min-width: 0;
  font-size: 12px;
  line-height: 1.3;
}

.robot-facts dt {
  color: var(--hmi-muted);
  font-weight: 900;
}

.robot-facts dd {
  overflow-wrap: anywhere;
  color: var(--hmi-text);
  font-weight: 800;
}

.device-contract,
.quick-task-panel,
.confirmation-priority,
.resource-locks,
.vision-state {
  display: grid;
  gap: 9px;
}

.device-contract {
  border: 1px solid var(--hmi-line);
  border-radius: 8px;
  padding: 10px;
  background: rgb(255 255 255 / 3%);
}

.contract-row {
  display: grid;
  grid-template-columns: auto 1fr;
  align-items: center;
  gap: 8px;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 800;
  line-height: 1.35;
}

.contract-row svg {
  color: var(--hmi-green);
}

.contract-row:last-child svg {
  color: var(--hmi-orange);
}

.panel-subhead {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 10px;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 900;
  text-transform: uppercase;
}

.panel-subhead h2 {
  margin: 0;
  color: inherit;
  font-size: inherit;
  font-weight: inherit;
  line-height: 1.2;
}

.panel-subhead strong {
  color: var(--hmi-text);
}

.panel-subhead.urgent strong {
  color: var(--hmi-orange);
}

.quick-task-button {
  width: 100%;
  min-height: 58px;
  display: grid;
  grid-template-columns: auto 1fr;
  align-items: center;
  gap: 9px;
  border-color: #2f4d45;
  padding: 9px 10px;
  color: var(--hmi-text);
  background: #0f1715;
  text-align: left;
}

.quick-task-button:hover:not(:disabled) {
  background: #1d332c;
}

.quick-task-button svg {
  color: var(--hmi-green);
}

.quick-task-button span {
  min-width: 0;
  display: grid;
  gap: 3px;
}

.quick-task-button strong,
.quick-task-button small {
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.quick-task-button strong {
  font-size: 13px;
}

.quick-task-button small {
  color: var(--hmi-muted);
  font-size: 11px;
  font-weight: 800;
}

.map-panel {
  display: grid;
  grid-template-rows: auto minmax(0, 1fr) auto;
  gap: 10px;
}

.map-frame {
  position: relative;
  overflow: hidden;
  border: 1px solid #263a35;
  border-radius: 8px;
  background: #0a100e;
}

.field-map {
  display: block;
  width: 100%;
  height: auto;
  aspect-ratio: 1000 / 560;
}

.grid-pattern {
  fill: none;
  stroke: rgb(103 128 119 / 24%);
  stroke-width: 1;
}

.field-bg {
  fill: #07100d;
}

.field-grid-bg {
  fill: #0d1714;
  stroke: #31443e;
  stroke-width: 2;
}

.field-grid-fill {
  fill: url(#field-grid);
}

.axis-lines line {
  stroke: rgb(158 174 168 / 24%);
  stroke-width: 1;
  vector-effect: non-scaling-stroke;
}

.planned-route {
  fill: none;
  stroke: var(--hmi-gray);
  stroke-dasharray: 12 10;
  stroke-linecap: square;
  stroke-linejoin: round;
  stroke-width: 5;
  opacity: 0.78;
  vector-effect: non-scaling-stroke;
}

.current-route {
  fill: none;
  stroke: var(--hmi-blue);
  stroke-linecap: square;
  stroke-linejoin: round;
  stroke-width: 7;
  opacity: 0.95;
  vector-effect: non-scaling-stroke;
}

.current-route.ackermann {
  stroke: #4ba3ff;
}

.map-point circle {
  fill: #14211e;
  stroke: #83938d;
  stroke-width: 2;
  vector-effect: non-scaling-stroke;
}

.map-point .point-core {
  fill: #c6d0cc;
  stroke: none;
}

.map-point.waiting circle {
  stroke: var(--hmi-orange);
}

.map-point.locked circle {
  stroke: var(--hmi-yellow);
}

.map-point.abnormal circle {
  stroke: var(--hmi-red);
}

.map-point.waiting .point-core {
  fill: var(--hmi-orange);
}

.map-point.locked .point-core {
  fill: var(--hmi-yellow);
}

.map-point.abnormal .point-core {
  fill: var(--hmi-red);
}

.map-point.waiting text,
.map-point.locked text,
.map-point.abnormal text {
  fill: #ffffff;
}

.map-point text,
.vehicle-label,
.lock-marker text,
.waiting-marker text,
.abnormal-marker text {
  fill: #dfe9e5;
  font-size: 20px;
  font-weight: 900;
  letter-spacing: 0;
  text-anchor: middle;
  paint-order: stroke;
  stroke: #07100d;
  stroke-width: 4px;
}

.map-point .point-label {
  fill: var(--hmi-muted);
  font-size: 14px;
}

.lock-marker rect {
  fill: rgb(241 200 75 / 12%);
  stroke: var(--hmi-yellow);
  stroke-width: 4;
  vector-effect: non-scaling-stroke;
}

.lock-marker text {
  fill: var(--hmi-yellow);
  font-size: 14px;
}

.waiting-marker circle {
  fill: none;
  stroke: var(--hmi-orange);
  stroke-width: 4;
  vector-effect: non-scaling-stroke;
}

.waiting-marker text {
  fill: var(--hmi-orange);
  font-size: 14px;
}

.abnormal-marker path {
  fill: rgb(226 61 54 / 20%);
  stroke: var(--hmi-red);
  stroke-width: 4;
  vector-effect: non-scaling-stroke;
}

.abnormal-marker text {
  fill: var(--hmi-red);
  font-size: 14px;
}

.vehicle {
  color: var(--hmi-blue);
}

.vehicle-ackermann {
  color: var(--hmi-green);
}

.vehicle.tone-bad {
  color: var(--hmi-red);
}

.vehicle.tone-warn {
  color: var(--hmi-orange);
}

.heading-arrow {
  fill: none;
  stroke: currentColor;
  stroke-width: 5;
  stroke-linecap: square;
  opacity: 0.82;
  vector-effect: non-scaling-stroke;
}

.heading-nose {
  fill: currentColor;
  opacity: 0.92;
}

.vehicle-body rect,
.vehicle-body path {
  fill: #101816;
  stroke: currentColor;
  stroke-width: 4;
  vector-effect: non-scaling-stroke;
}

.vehicle-body .vehicle-cabin {
  fill: currentColor;
  opacity: 0.3;
}

.mecanum-wheels rect,
.ackermann-wheels rect {
  fill: currentColor;
  stroke: #06100d;
  stroke-width: 2;
  vector-effect: non-scaling-stroke;
}

.vehicle-label {
  fill: #ffffff;
  font-size: 18px;
}

.map-callouts {
  position: absolute;
  right: 10px;
  bottom: 10px;
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 6px;
  flex-wrap: wrap;
  pointer-events: none;
}

.map-callouts span {
  border: 1px solid rgb(158 174 168 / 32%);
  border-radius: 6px;
  padding: 5px 7px;
  color: #dce7e2;
  background: rgb(7 16 13 / 82%);
  font-size: 11px;
  font-weight: 900;
}

.map-legend {
  display: flex;
  align-items: center;
  gap: 8px 12px;
  flex-wrap: wrap;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 900;
}

.map-legend span {
  display: inline-flex;
  align-items: center;
  gap: 6px;
}

.legend-swatch {
  width: 12px;
  height: 12px;
  border-radius: 3px;
  background: var(--hmi-gray);
}

/* 颜色与 SituationMap 中标志严格一致 */
.legend-swatch.robot-mecanum {
  background: #2dd4bf;
  border-radius: 2px;
}

.legend-swatch.robot-ackermann {
  background: #f59e0b;
  border-radius: 2px;
}

.legend-swatch.pt-waiting {
  background: #64748b;
  border-radius: 50%;
}

.legend-swatch.pt-pickup {
  background: #38bdf8;
  border-radius: 50%;
}

.legend-swatch.pt-delivery {
  background: #a78bfa;
  border-radius: 50%;
}

.legend-swatch.pt-inspection {
  background: #f472b6;
  border-radius: 50%;
}

.legend-ring {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  border: 2px solid var(--hmi-cyan);
}

.legend-badge {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 15px;
  height: 15px;
  border-radius: 50%;
  background: var(--hmi-cyan);
  color: #06121f;
  font-size: 10px;
  font-weight: 800;
  font-style: normal;
}

.legend-line {
  width: 28px;
  height: 0;
  border-top: 3px dashed var(--hmi-text);
}

.confirmation-row,
.lock-row,
.vision-box,
.event-row,
.diagnosis-card,
.route-context article {
  border: 1px solid var(--hmi-line);
  border-radius: 8px;
  background: #101816;
}

.confirmation-row {
  display: grid;
  gap: 9px;
  padding: 10px;
  border-color: rgb(255 159 47 / 54%);
}

.confirmation-row strong,
.confirmation-row span,
.lock-row strong,
.lock-row span,
.lock-row small,
.vision-box strong,
.vision-box span {
  min-width: 0;
}

.confirmation-row strong {
  display: block;
  font-size: 14px;
  line-height: 1.25;
}

.confirmation-row span {
  display: block;
  margin-top: 3px;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 800;
  line-height: 1.3;
}

.row-meta {
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}

.lock-row {
  display: grid;
  gap: 3px;
  padding: 9px 10px;
  border-color: rgb(241 200 75 / 45%);
}

.lock-row strong {
  color: var(--hmi-yellow);
  font-size: 14px;
}

.lock-row span,
.lock-row small {
  overflow-wrap: anywhere;
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 800;
}

.vision-box {
  display: grid;
  grid-template-columns: auto 1fr;
  align-items: start;
  gap: 9px;
  padding: 10px;
  border-color: rgb(158 174 168 / 42%);
}

.vision-box svg {
  color: var(--hmi-orange);
}

.vision-box div {
  min-width: 0;
  display: grid;
  gap: 3px;
}

.vision-box span {
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 800;
  line-height: 1.35;
}

.bottom-diagnosis {
  grid-column: 1 / -1;
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(230px, 0.55fr) minmax(280px, 0.8fr);
  gap: 10px;
  align-items: start;
}

.bottom-diagnosis .panel-heading {
  grid-column: 1 / -1;
}

.diagnosis-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 8px;
}

.diagnosis-card {
  display: grid;
  gap: 4px;
  min-height: 82px;
  padding: 10px;
}

.diagnosis-card span,
.diagnosis-card small,
.event-row span,
.event-row small {
  color: var(--hmi-muted);
  font-size: 11px;
  font-weight: 900;
  line-height: 1.3;
}

.diagnosis-card strong,
.event-row strong,
.route-context strong {
  min-width: 0;
  overflow-wrap: anywhere;
  color: var(--hmi-text);
  font-size: 14px;
  line-height: 1.25;
}

.diagnosis-card.tone-ok {
  border-color: rgb(49 199 106 / 50%);
}

.diagnosis-card.tone-warn {
  border-color: rgb(255 159 47 / 58%);
}

.diagnosis-card.tone-bad {
  border-color: rgb(226 61 54 / 62%);
}

.diagnosis-card.tone-info {
  border-color: rgb(47 133 255 / 58%);
}

.route-context {
  display: grid;
  gap: 8px;
}

.context-title {
  margin: 0;
  color: var(--hmi-text);
  font-size: 16px;
  line-height: 1.2;
  letter-spacing: 0;
}

.route-context article {
  display: grid;
  gap: 5px;
  padding: 10px;
}

.route-context article.active-task-row {
  border-color: rgb(47 133 255 / 46%);
  background: rgb(47 133 255 / 8%);
}

.route-context article.mecanum {
  border-color: rgb(47 133 255 / 56%);
}

.route-context article.ackermann {
  border-color: rgb(49 199 106 / 56%);
}

.route-context span {
  color: var(--hmi-muted);
  font-size: 12px;
  font-weight: 900;
  line-height: 1.35;
}

.event-row {
  display: grid;
  grid-template-columns: 56px minmax(90px, 0.55fr) minmax(0, 1fr);
  align-items: center;
  gap: 8px;
  padding: 8px 10px;
}

.event-row.tone-ok {
  border-color: rgb(49 199 106 / 38%);
}

.event-row.tone-warn {
  border-color: rgb(255 159 47 / 48%);
}

.event-row.tone-bad {
  border-color: rgb(226 61 54 / 58%);
}

.hmi-empty {
  margin: 0;
  border: 1px dashed var(--hmi-line-strong);
  border-radius: 8px;
  padding: 12px;
  color: var(--hmi-muted);
  background: rgb(255 255 255 / 3%);
  font-size: 13px;
  font-weight: 800;
}

@media (max-width: 1180px) {
  .hmi-topbar,
  .hmi-layout,
  .bottom-diagnosis {
    grid-template-columns: 1fr;
  }

  .global-status-strip {
    justify-content: flex-start;
  }

  .bottom-diagnosis .panel-heading {
    grid-column: auto;
  }
}

@media (max-width: 760px) {
  .overview-hmi {
    padding: 10px;
  }

  .hmi-topbar {
    align-items: stretch;
  }

  .hmi-refresh {
    width: 100%;
  }

  .map-callouts {
    position: static;
    justify-content: flex-start;
    padding: 8px;
    background: #07100d;
  }

  .diagnosis-grid {
    grid-template-columns: 1fr;
  }

  .event-row {
    grid-template-columns: 1fr;
  }

  .quick-task-button strong,
  .quick-task-button small {
    white-space: normal;
  }
}
</style>
