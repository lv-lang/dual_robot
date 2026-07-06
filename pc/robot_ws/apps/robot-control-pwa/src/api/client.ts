import type {
  AggregateState,
  AggregateStatus,
  ApiClient,
  BusinessPoint,
  CommandResult,
  ConfirmStepPayload,
  HealthResponse,
  LogEntry,
  LogListResponse,
  ResourceLock,
  RobotSummary,
  StatusSocketHandlers,
  StatusSocketHandle,
  StatusSocketMessage,
  SystemActionResponse,
  SystemHealthStatus,
  SystemLaunchLogLine,
  SystemLogsResponse,
  SystemStatusResponse,
  SystemStatusValue,
  TaskSummary,
  TaskTemplate,
  TemplateListResponse,
  TemplatePayload,
  TriggerTaskResult
} from "../types";

type NormalizedRobotPreference = "auto" | "mecanum" | "ackermann";

export class ApiError extends Error {
  readonly status: number;
  readonly detail?: unknown;

  constructor(message: string, status: number, detail?: unknown) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.detail = detail;
  }
}

type Fetcher = typeof fetch;

function apiBase(): string {
  return import.meta.env.VITE_API_BASE || "/api";
}

function pathWithBase(path: string): string {
  const base = apiBase().replace(/\/$/, "");
  return `${base}${path.startsWith("/") ? path : `/${path}`}`;
}

async function parseResponse<T>(response: Response): Promise<T> {
  const text = await response.text();
  const data = text ? (JSON.parse(text) as unknown) : undefined;

  if (!response.ok) {
    const message =
      typeof data === "object" && data && "message" in data
        ? String((data as { message: unknown }).message)
        : typeof data === "object" && data && "detail" in data
          ? String((data as { detail: unknown }).detail)
          : `HTTP ${response.status}`;
    throw new ApiError(message, response.status, data);
  }

  return data as T;
}

function stringValue(value: unknown, fallback = ""): string {
  return typeof value === "string" ? value : value == null ? fallback : String(value);
}

function boolValue(value: unknown, fallback = false): boolean {
  return typeof value === "boolean" ? value : fallback;
}

function arrayValue<T = unknown>(value: unknown): T[] {
  return Array.isArray(value) ? (value as T[]) : [];
}

function numberOrNull(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) {
    return value;
  }
  if (typeof value === "string" && value.trim()) {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  }
  return null;
}

function robotPreferenceValue(value: unknown): NormalizedRobotPreference {
  const normalized = stringValue(value, "auto");
  return normalized === "mecanum" || normalized === "ackermann" ? normalized : "auto";
}

function normalizeHealth(raw: unknown): HealthResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  const dispatchOnline = boolValue(data.dispatch_online);
  return {
    backend_online: boolValue(data.backend_online, true),
    dispatch_online: dispatchOnline,
    dispatch_degraded: boolValue(data.dispatch_degraded, !dispatchOnline),
    reason: stringValue(data.reason),
    updated_at: stringValue(data.updated_at, new Date().toISOString()),
    disabled_reasons: arrayValue<string>(data.disabled_reasons)
  };
}

function normalizeAggregateStatus(raw: Record<string, unknown>): AggregateStatus {
  const status = (raw.status ?? {}) as Record<string, unknown>;
  const dispatchStatus = (raw.dispatch_status ?? {}) as Record<string, unknown>;
  const backendStatus = (raw.backend_status ?? {}) as Record<string, unknown>;
  const dispatchOnline = boolValue(status.dispatch_online, boolValue(dispatchStatus.online));
  return {
    backend_online: boolValue(status.backend_online, boolValue(backendStatus.online, true)),
    dispatch_online: dispatchOnline,
    dispatch_degraded: boolValue(status.dispatch_degraded, !dispatchOnline),
    websocket_online:
      typeof status.websocket_online === "boolean" ? Boolean(status.websocket_online) : undefined,
    disabled_reasons: arrayValue<string>(status.disabled_reasons ?? raw.disabled_reasons),
    updated_at: stringValue(status.updated_at, new Date().toISOString())
  };
}

function normalizePose(raw: unknown): RobotSummary["pose"] {
  if (!raw || typeof raw !== "object") return undefined;
  const pose = raw as Record<string, unknown>;
  const position = (pose.position ?? {}) as Record<string, unknown>;
  const orientation = (pose.orientation ?? {}) as Record<string, unknown>;
  const x = Number(position.x);
  const y = Number(position.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return undefined;
  const qz = Number(orientation.z ?? 0);
  const qw = Number(orientation.w ?? 1);
  const yaw = Math.atan2(2 * qw * qz, 1 - 2 * qz * qz);
  return { x, y, yaw: Number.isFinite(yaw) ? yaw : 0 };
}

function normalizeRobot(raw: unknown): RobotSummary {
  const data = (raw ?? {}) as Record<string, unknown>;
  const robotId = stringValue(data.robot_id) as RobotSummary["robot_id"];
  return {
    robot_id: robotId,
    display_name: stringValue(data.display_name, robotId),
    chassis_type: stringValue(data.chassis_type, robotId === "mecanum" ? "mecanum" : "ackermann") as RobotSummary["chassis_type"],
    status: stringValue(data.status ?? data.state, "UNKNOWN"),
    current_task_id: stringValue(data.current_task_id) || undefined,
    current_task_label: stringValue(data.current_task_label) || undefined,
    last_update: stringValue(data.last_update) || undefined,
    pose: normalizePose(data.pose)
  };
}

function normalizeTask(raw: unknown): TaskSummary {
  const data = (raw ?? {}) as Record<string, unknown>;
  const steps = arrayValue<Record<string, unknown>>(data.steps);
  const targetPoints = arrayValue<string>(data.target_points ?? data.target_point_ids);
  const fallbackPoints = steps.map((step) => stringValue(step.point_id)).filter(Boolean);
  const assignedRobotId = stringValue(data.assigned_robot_id ?? data.robot_id) || undefined;
  return {
    task_id: stringValue(data.task_id),
    task_type: stringValue(data.task_type, "UNKNOWN"),
    display_name: stringValue(data.display_name ?? data.task_display_name) || undefined,
    label: stringValue(data.label ?? data.display_name, stringValue(data.task_id)),
    status: stringValue(data.status ?? data.state, "UNKNOWN"),
    robot_id: assignedRobotId as TaskSummary["robot_id"],
    preferred_robot_id: data.preferred_robot_id == null ? undefined : robotPreferenceValue(data.preferred_robot_id),
    assigned_robot_id: assignedRobotId as TaskSummary["assigned_robot_id"],
    current_step_label: stringValue(data.current_step_label) || undefined,
    current_step_index:
      typeof data.current_step_index === "number" ? data.current_step_index : undefined,
    steps: steps.map((step) => ({
      sequence: Number(step.sequence ?? 0),
      step_type: stringValue(step.step_type),
      point_id: stringValue(step.point_id),
      label: stringValue(step.label) || undefined
    })),
    target_points: targetPoints.length ? targetPoints : fallbackPoints,
    created_at: stringValue(data.created_at) || undefined,
    updated_at: stringValue(data.updated_at) || undefined
  };
}

function normalizeResourceLock(raw: unknown): ResourceLock {
  const data = (raw ?? {}) as Record<string, unknown>;
  const pointId = stringValue(data.point_id ?? data.resource_id);
  return {
    point_id: pointId,
    point_label: stringValue(data.point_label, pointId),
    holder_task_id: stringValue(data.holder_task_id ?? data.locked_by_task_id),
    robot_id: (stringValue(data.robot_id ?? data.locked_by_robot_id) || undefined) as ResourceLock["robot_id"],
    lock_type: stringValue(data.lock_type ?? data.resource_type ?? data.status, "LOCKED")
  };
}

function normalizeConfirmation(raw: unknown) {
  const data = (raw ?? {}) as Record<string, unknown>;
  const pointId = stringValue(data.point_id);
  const assignedRobotId = stringValue(data.assigned_robot_id ?? data.robot_id) || undefined;
  return {
    task_id: stringValue(data.task_id),
    step_index: typeof data.step_index === "number" ? data.step_index : undefined,
    step_id: stringValue(data.step_id),
    task_type: stringValue(data.task_type, "UNKNOWN"),
    display_name: stringValue(data.display_name ?? data.task_display_name) || undefined,
    point_id: pointId,
    point_label: stringValue(data.point_label, pointId),
    label: stringValue(data.label, pointId || stringValue(data.step_id)),
    robot_id: assignedRobotId as "mecanum" | "ackermann" | undefined,
    preferred_robot_id: data.preferred_robot_id == null ? undefined : robotPreferenceValue(data.preferred_robot_id),
    assigned_robot_id: assignedRobotId as "mecanum" | "ackermann" | undefined
  };
}

function normalizeAggregateState(raw: unknown): AggregateState {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    status: normalizeAggregateStatus(data),
    robots: arrayValue(data.robots).map(normalizeRobot),
    tasks: arrayValue(data.tasks).map(normalizeTask),
    resource_locks: arrayValue(data.resource_locks).map(normalizeResourceLock),
    waiting_confirmations: arrayValue(data.waiting_confirmations).map(normalizeConfirmation)
  };
}

function normalizeBusinessPoint(raw: unknown): BusinessPoint {
  const data = (raw ?? {}) as Record<string, unknown>;
  const kind = stringValue(data.point_type ?? data.kind).toLowerCase();
  return {
    point_id: stringValue(data.point_id),
    label: stringValue(data.label ?? data.point_id),
    point_type: (kind === "waiting_area" ? "waiting" : kind) as BusinessPoint["point_type"]
  };
}

function normalizeTemplate(raw: unknown): TaskTemplate {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    template_id: stringValue(data.template_id),
    name: stringValue(data.name ?? data.display_name),
    task_type: stringValue(data.task_type, "DELIVERY") as TaskTemplate["task_type"],
    robot_preference: robotPreferenceValue(data.robot_preference ?? data.preferred_robot_id),
    target_points: arrayValue<string>(data.target_points ?? data.target_point_ids),
    readonly: boolValue(data.readonly, boolValue(data.builtin)),
    sort_order: Number(data.sort_order ?? 100),
    available: boolValue(data.available, true),
    unavailable_reason: stringValue(data.unavailable_reason),
    missing_point_ids: arrayValue<string>(data.missing_point_ids)
  };
}

function normalizeTemplatePayload(payload: TemplatePayload): Record<string, unknown> {
  return {
    display_name: payload.name,
    name: payload.name,
    task_type: payload.task_type,
    preferred_robot_id: payload.robot_preference,
    robot_preference: payload.robot_preference,
    target_point_ids: payload.target_points,
    target_points: payload.target_points,
    sort_order: payload.sort_order
  };
}

function normalizeTemplateList(raw: unknown): TemplateListResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    templates: arrayValue(data.templates).map(normalizeTemplate),
    business_points: arrayValue(data.business_points ?? data.points).map(normalizeBusinessPoint)
  };
}

function normalizeCommand(raw: unknown): CommandResult {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    ok: boolValue(data.ok, boolValue(data.accepted, true)),
    message: stringValue(data.message)
  };
}

function normalizeTriggerTaskResult(raw: unknown): TriggerTaskResult {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    task_id: stringValue(data.task_id),
    message: stringValue(data.message),
    display_name: stringValue(data.display_name ?? data.task_display_name) || undefined,
    preferred_robot_id: data.preferred_robot_id == null ? undefined : robotPreferenceValue(data.preferred_robot_id),
    assigned_robot_id: (stringValue(data.assigned_robot_id ?? data.robot_id) || undefined) as TriggerTaskResult["assigned_robot_id"]
  };
}

function normalizeLog(raw: unknown): LogEntry {
  const data = (raw ?? {}) as Record<string, unknown>;
  const level = stringValue(data.level, "info").toLowerCase();
  const detail = data.detail && typeof data.detail === "object" ? data.detail as Record<string, unknown> : {};
  return {
    log_id: stringValue(data.log_id ?? data.id),
    timestamp: stringValue(data.timestamp ?? data.created_at, new Date().toISOString()),
    level: (level === "error" || level === "warning" ? level : "info") as LogEntry["level"],
    event: stringValue(data.event ?? data.event_type),
    message: stringValue(data.message),
    task_id: stringValue(data.task_id) || undefined,
    task_display_name: stringValue(data.task_display_name ?? detail.display_name) || undefined,
    robot_id: (stringValue(data.robot_id ?? detail.robot_id) || undefined) as LogEntry["robot_id"],
    detail
  };
}

function normalizeLogList(raw: unknown): LogListResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  return { logs: arrayValue(data.logs).map(normalizeLog) };
}

const SYSTEM_STATUS_VALUES = new Set<string>([
  "stopped",
  "starting",
  "running",
  "degraded",
  "stopping",
  "failed",
  "external"
]);

const SYSTEM_HEALTH_VALUES = new Set<string>(["ok", "missing", "failed", "not_checked"]);

function normalizeSystemStatusValue(value: unknown, fallback: SystemStatusValue): SystemStatusValue {
  const normalized = stringValue(value, fallback).toLowerCase();
  return SYSTEM_STATUS_VALUES.has(normalized) ? (normalized as SystemStatusValue) : fallback;
}

function normalizeSystemHealthStatus(value: unknown): SystemHealthStatus {
  const normalized = stringValue(value, "not_checked").toLowerCase().replace("-", "_");
  return SYSTEM_HEALTH_VALUES.has(normalized) ? (normalized as SystemHealthStatus) : "not_checked";
}

function normalizeSystemProfile(raw: unknown) {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    id: stringValue(data.id, "real_robot_control_plane"),
    name: stringValue(data.name, "Real Robot Control Plane"),
    command: stringValue(
      data.command,
      "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
    )
  };
}

function normalizeSystemHealthRow(raw: unknown): SystemStatusResponse["health"][number] {
  const data = (raw ?? {}) as Record<string, unknown>;
  const id = stringValue(data.id);
  return {
    id,
    label: stringValue(data.label, id || "未命名检查项"),
    category: stringValue(data.category, "system"),
    status: normalizeSystemHealthStatus(data.status),
    required: boolValue(data.required, true),
    detail: stringValue(data.detail)
  };
}

function normalizeSystemStatus(raw: unknown): SystemStatusResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  const status = normalizeSystemStatusValue(
    data.status,
    boolValue(data.external_running) ? "external" : "stopped"
  );
  const externalRunning = boolValue(data.external_running, status === "external");
  const managed = boolValue(data.managed);
  const defaultCanStart = status === "stopped" || status === "failed";
  const defaultCanStop = managed && (status === "starting" || status === "running" || status === "degraded");
  const defaultCanRestart = managed && (status === "running" || status === "degraded" || status === "failed");

  return {
    status,
    summary: stringValue(data.summary, status === "stopped" ? "调度系统未启动" : "调度系统状态未知"),
    managed,
    external_running: externalRunning,
    can_start: externalRunning ? false : boolValue(data.can_start, defaultCanStart),
    can_stop: externalRunning ? false : boolValue(data.can_stop, defaultCanStop),
    can_restart: externalRunning ? false : boolValue(data.can_restart, defaultCanRestart),
    profile: normalizeSystemProfile(data.profile),
    pid: numberOrNull(data.pid),
    pgid: numberOrNull(data.pgid),
    started_at: stringValue(data.started_at),
    updated_at: stringValue(data.updated_at, new Date().toISOString()),
    health: arrayValue(data.health).map(normalizeSystemHealthRow)
  };
}

function normalizeSystemAction(raw: unknown): SystemActionResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    accepted: boolValue(data.accepted, boolValue(data.ok, true)),
    message: stringValue(data.message),
    status: normalizeSystemStatus(data.status ?? data.system_status ?? raw),
    log: data.log ? normalizeLog(data.log) : undefined
  };
}

function normalizeSystemLaunchLog(raw: unknown, index: number): SystemLaunchLogLine {
  const data = (raw ?? {}) as Record<string, unknown>;
  const lineNo = numberOrNull(data.line_no ?? data.line);
  return {
    line_no: lineNo ?? index + 1,
    stream: stringValue(data.stream, "launch"),
    message: stringValue(data.message ?? data.text),
    timestamp: stringValue(data.timestamp)
  };
}

function normalizeSystemLogs(raw: unknown): SystemLogsResponse {
  const data = (raw ?? {}) as Record<string, unknown>;
  return {
    launch_logs: arrayValue(data.launch_logs ?? data.output_logs).map(normalizeSystemLaunchLog),
    operation_logs: arrayValue(data.operation_logs).map(normalizeLog)
  };
}

export class RobotControlApiClient implements ApiClient {
  private readonly fetcher: Fetcher;

  constructor(fetcher: Fetcher = fetch.bind(globalThis)) {
    this.fetcher = fetcher;
  }

  private async request<T>(path: string, init: RequestInit = {}): Promise<T> {
    const headers = new Headers(init.headers);
    if (!headers.has("Content-Type") && init.body) {
      headers.set("Content-Type", "application/json");
    }
    const response = await this.fetcher(pathWithBase(path), {
      ...init,
      headers
    });
    return parseResponse<T>(response);
  }

  getHealth(): Promise<HealthResponse> {
    return this.request<unknown>("/health").then(normalizeHealth);
  }

  getState(): Promise<AggregateState> {
    return this.request<unknown>("/state").then(normalizeAggregateState);
  }

  getTemplates(): Promise<TemplateListResponse> {
    return this.request<unknown>("/templates").then(normalizeTemplateList);
  }

  createTemplate(payload: TemplatePayload): Promise<TaskTemplate> {
    return this.request<unknown>("/templates", {
      method: "POST",
      body: JSON.stringify(normalizeTemplatePayload(payload))
    }).then(normalizeTemplate);
  }

  updateTemplate(templateId: string, payload: TemplatePayload): Promise<TaskTemplate> {
    return this.request<unknown>(`/templates/${encodeURIComponent(templateId)}`, {
      method: "PUT",
      body: JSON.stringify(normalizeTemplatePayload(payload))
    }).then(normalizeTemplate);
  }

  deleteTemplate(templateId: string): Promise<CommandResult> {
    return this.request<unknown>(`/templates/${encodeURIComponent(templateId)}`, {
      method: "DELETE"
    }).then(normalizeCommand);
  }

  reorderTemplates(templateIds: string[]): Promise<TemplateListResponse> {
    return this.request<unknown>("/templates/reorder", {
      method: "POST",
      body: JSON.stringify({ template_ids: templateIds })
    }).then(normalizeTemplateList);
  }

  triggerTemplate(templateId: string): Promise<TriggerTaskResult> {
    return this.request<unknown>(`/templates/${encodeURIComponent(templateId)}/trigger`, {
      method: "POST"
    }).then(normalizeTriggerTaskResult);
  }

  confirmStep(payload: ConfirmStepPayload): Promise<CommandResult> {
    return this.request<unknown>("/confirmations", {
      method: "POST",
      body: JSON.stringify(payload)
    }).then(normalizeCommand);
  }

  pauseTask(taskId: string): Promise<CommandResult> {
    return this.request<unknown>(`/tasks/${encodeURIComponent(taskId)}/pause`, {
      method: "POST"
    }).then(normalizeCommand);
  }

  resumeTask(taskId: string): Promise<CommandResult> {
    return this.request<unknown>(`/tasks/${encodeURIComponent(taskId)}/resume`, {
      method: "POST"
    }).then(normalizeCommand);
  }

  cancelTask(taskId: string): Promise<CommandResult> {
    return this.request<unknown>(`/tasks/${encodeURIComponent(taskId)}/cancel`, {
      method: "POST"
    }).then(normalizeCommand);
  }

  emergencyStop(): Promise<CommandResult> {
    return this.request<unknown>("/emergency-stop", {
      method: "POST"
    }).then(normalizeCommand);
  }

  getLogs(limit = 100): Promise<LogListResponse> {
    return this.request<unknown>(`/logs?limit=${encodeURIComponent(String(limit))}`).then(normalizeLogList);
  }

  getSystemStatus(): Promise<SystemStatusResponse> {
    return this.request<unknown>("/system/status").then(normalizeSystemStatus);
  }

  startSystem(): Promise<SystemActionResponse> {
    return this.request<unknown>("/system/start", {
      method: "POST"
    }).then(normalizeSystemAction);
  }

  stopSystem(): Promise<SystemActionResponse> {
    return this.request<unknown>("/system/stop", {
      method: "POST"
    }).then(normalizeSystemAction);
  }

  restartSystem(): Promise<SystemActionResponse> {
    return this.request<unknown>("/system/restart", {
      method: "POST"
    }).then(normalizeSystemAction);
  }

  getSystemLogs(limit = 120): Promise<SystemLogsResponse> {
    return this.request<unknown>(`/system/logs?limit=${encodeURIComponent(String(limit))}`).then(normalizeSystemLogs);
  }
}

export function statusSocketUrl(): string {
  if (import.meta.env.VITE_WS_URL) {
    return import.meta.env.VITE_WS_URL;
  }

  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/ws/status`;
}

export function connectStatusSocket(
  handlers: StatusSocketHandlers,
  WebSocketCtor: typeof WebSocket = WebSocket
): StatusSocketHandle {
  const socket = new WebSocketCtor(statusSocketUrl());

  socket.addEventListener("open", () => handlers.onOpen?.());
  socket.addEventListener("close", () => handlers.onClose?.());
  socket.addEventListener("error", (event) => handlers.onError?.(event));
  socket.addEventListener("message", (event) => {
    const message = JSON.parse(String(event.data)) as StatusSocketMessage;
    if (message.type === "state_update") {
      handlers.onStateUpdate?.(normalizeAggregateState(message.state));
      return;
    }
    if (message.type === "log_update") {
      const logs = Array.isArray(message.logs) ? message.logs : message.log ? [message.log] : [];
      handlers.onLogUpdate?.(logs.map(normalizeLog));
      return;
    }
    if (message.type === "demo_warning_clear") {
      handlers.onDemoWarningClear?.(message.warning);
    }
  });

  return {
    close: () => socket.close()
  };
}

export const realApiClient = new RobotControlApiClient();
