import type {
  AggregateState,
  ApiClient,
  BusinessPoint,
  CommandResult,
  ConfirmStepPayload,
  HealthResponse,
  LogEntry,
  LogListResponse,
  StatusSocketHandlers,
  StatusSocketHandle,
  SystemActionResponse,
  SystemHealthRow,
  SystemLaunchLogLine,
  SystemLogsResponse,
  SystemProfile,
  SystemStatusResponse,
  SystemStatusValue,
  TaskSummary,
  TaskTemplate,
  TemplateListResponse,
  TemplatePayload,
  TriggerTaskResult
} from "../types";

const now = () => new Date().toISOString();

const businessPoints: BusinessPoint[] = [
  { point_id: "W1", label: "W1 等待区", point_type: "waiting" },
  { point_id: "W2", label: "W2 等待区", point_type: "waiting" },
  { point_id: "A", label: "A 取货点", point_type: "pickup" },
  { point_id: "C", label: "C 配送点", point_type: "delivery" },
  { point_id: "P1", label: "P1 巡检点", point_type: "inspection" },
  { point_id: "P2", label: "P2 巡检点", point_type: "inspection" },
  { point_id: "P3", label: "P3 巡检点", point_type: "inspection" }
];

const initialTemplates: TaskTemplate[] = [
  {
    template_id: "builtin-delivery-a-c",
    name: "配送 A 到 C",
    task_type: "DELIVERY",
    robot_preference: "mecanum",
    target_points: ["A", "C"],
    readonly: true,
    sort_order: 10
  },
  {
    template_id: "builtin-inspection-p-route",
    name: "巡检 P1 P2 P3",
    task_type: "INSPECTION",
    robot_preference: "ackermann",
    target_points: ["P1", "P2", "P3"],
    readonly: true,
    sort_order: 20
  }
];

let templates: TaskTemplate[] = clone(initialTemplates);

let taskCounter = 1;
let taskDisplayCounters: Record<string, number> = {};

let tasks: TaskSummary[] = [];

const initialLogs: LogEntry[] = [
  {
    log_id: "log-000",
    timestamp: now(),
    level: "info",
    event: "mock_started",
    message: "模拟网关已就绪"
  }
];

let logs: LogEntry[] = clone(initialLogs);

const systemProfile: SystemProfile = {
  id: "real_robot_control_plane",
  name: "Real Robot Control Plane",
  command: "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
};

let systemStatusValue: SystemStatusValue = "stopped";
let systemManaged = false;
let systemPid: number | null = null;
let systemPgid: number | null = null;
let systemStartedAt = "";

let launchLogs: SystemLaunchLogLine[] = [];
const initialOperationLogs: LogEntry[] = [
  {
    log_id: "system-log-000",
    timestamp: now(),
    level: "info",
    event: "system_external_running",
    message: "模拟系统控制接口已就绪"
  }
];

let operationLogs: LogEntry[] = clone(initialOperationLogs);

function clone<T>(value: T): T {
  return JSON.parse(JSON.stringify(value)) as T;
}

function pushLog(entry: Omit<LogEntry, "log_id" | "timestamp">): void {
  logs = [
    {
      log_id: `log-${String(logs.length + 1).padStart(3, "0")}`,
      timestamp: now(),
      ...entry
    },
    ...logs
  ];
}

function pushSystemOperation(entry: Omit<LogEntry, "log_id" | "timestamp">): LogEntry {
  const log = {
    log_id: `system-log-${String(operationLogs.length + 1).padStart(3, "0")}`,
    timestamp: now(),
    ...entry
  };
  operationLogs = [log, ...operationLogs];
  return log;
}

function pushLaunchLog(stream: string, message: string): void {
  launchLogs = [
    ...launchLogs,
    {
      line_no: launchLogs.length + 1,
      stream,
      message,
      timestamp: now()
    }
  ];
}

function ok(message: string): CommandResult {
  return { ok: true, message };
}

function resetMockState(): void {
  templates = clone(initialTemplates);
  taskCounter = 1;
  taskDisplayCounters = {};
  tasks = [];
  logs = clone(initialLogs);
  systemStatusValue = "stopped";
  systemManaged = false;
  systemPid = null;
  systemPgid = null;
  systemStartedAt = "";
  launchLogs = [];
  operationLogs = clone(initialOperationLogs);
}

function nextTaskDisplayName(taskType: TaskTemplate["task_type"]): string {
  const normalizedType = String(taskType).toUpperCase();
  taskDisplayCounters[normalizedType] = (taskDisplayCounters[normalizedType] ?? 0) + 1;
  const prefix: Record<string, string> = {
    DELIVERY: "配送任务",
    INSPECTION: "巡检任务",
    RECHECK: "复查任务"
  };
  return `${prefix[normalizedType] ?? "任务"}_${taskDisplayCounters[normalizedType]}`;
}

function pointType(pointId: string): BusinessPoint["point_type"] | undefined {
  return businessPoints.find((point) => point.point_id === pointId)?.point_type;
}

function validateTemplatePayload(payload: TemplatePayload): void {
  if (!payload.name.trim()) {
    throw new Error("请输入模板名称");
  }
  if (payload.robot_preference !== "auto" && payload.robot_preference !== "mecanum" && payload.robot_preference !== "ackermann") {
    throw new Error("机器人偏好必须是自动、mecanum 或 ackermann");
  }
  if (payload.task_type === "DELIVERY") {
    if (
      payload.target_points.length !== 2 ||
      pointType(payload.target_points[0]) !== "pickup" ||
      pointType(payload.target_points[1]) !== "delivery"
    ) {
      throw new Error("配送模板需要先选择取货点，再选择配送点");
    }
    return;
  }
  if (payload.task_type === "INSPECTION") {
    if (payload.target_points.length === 0 || payload.target_points.some((pointId) => pointType(pointId) !== "inspection")) {
      throw new Error("巡检模板只能选择巡检点");
    }
    return;
  }
  throw new Error("任务类型必须是配送或巡检");
}

function systemHealth(): SystemHealthRow[] {
  const running = systemStatusValue === "running" || systemStatusValue === "degraded";
  return [
    {
      id: "process.managed_launch",
      label: "受控 launch 进程",
      category: "process",
      status: running ? "ok" : "missing",
      required: true,
      detail: running ? `pid ${systemPid ?? "-"}` : "未检测到 App 管理进程"
    },
    {
      id: "process.map_server",
      label: "PC map_server",
      category: "process",
      status: running ? "ok" : "missing",
      required: true,
      detail: running ? "map_server online" : "no matching process"
    },
    {
      id: "node.robot_dispatch",
      label: "/robot_dispatch",
      category: "node",
      status: running ? "ok" : "not_checked",
      required: true,
      detail: running ? "调度节点在线" : "系统未运行"
    },
    {
      id: "topic.system_state",
      label: "/robot_dispatch/system_state",
      category: "topic",
      status: running ? "ok" : "not_checked",
      required: true,
      detail: running ? "系统状态在线" : "系统未运行"
    },
    {
      id: "topic.mecanum.scan",
      label: "/mecanum/scan",
      category: "topic",
      status: running ? "ok" : "not_checked",
      required: true,
      detail: running ? "最近检测到消息" : "系统未运行"
    },
    {
      id: "topic.ackermann.odom",
      label: "/ackermann/odom",
      category: "topic",
      status: running ? "ok" : "not_checked",
      required: true,
      detail: running ? "最近检测到消息" : "系统未运行"
    },
    {
      id: "forbidden.global_cmd_vel",
      label: "禁止全局 /cmd_vel",
      category: "forbidden_topic",
      status: running ? "ok" : "not_checked",
      required: true,
      detail: running ? "未发现全局 /cmd_vel" : "系统未运行"
    }
  ];
}

function systemSummary(): string {
  if (systemStatusValue === "running") {
    return "调度系统由 App 管理运行中";
  }
  if (systemStatusValue === "starting") {
    return "调度系统启动中";
  }
  if (systemStatusValue === "stopping") {
    return "调度系统停止中";
  }
  if (systemStatusValue === "failed") {
    return "调度系统启动或运行失败";
  }
  return "调度系统未启动";
}

function systemStatus(): SystemStatusResponse {
  const running = systemStatusValue === "running" || systemStatusValue === "degraded";
  const starting = systemStatusValue === "starting";
  return {
    status: systemStatusValue,
    summary: systemSummary(),
    managed: systemManaged,
    external_running: systemStatusValue === "external",
    can_start: systemStatusValue === "stopped" || systemStatusValue === "failed",
    can_stop: systemManaged && (running || starting),
    can_restart: systemManaged && running,
    profile: systemProfile,
    pid: systemPid,
    pgid: systemPgid,
    started_at: systemStartedAt,
    updated_at: now(),
    health: systemHealth()
  };
}

function aggregateState(): AggregateState {
  const waitingTasks = tasks.filter((task) => task.status === "WAITING_CONFIRMATION");
  const mecanumTask = tasks.find((task) => task.robot_id === "mecanum" && task.status !== "COMPLETED");
  const ackermannTask = tasks.find((task) => task.robot_id === "ackermann" && task.status !== "COMPLETED");
  return {
    status: {
      backend_online: true,
      dispatch_online: true,
      dispatch_degraded: false,
      websocket_online: true,
      disabled_reasons: [],
      updated_at: now()
    },
    robots: [
      {
        robot_id: "mecanum",
        display_name: "mecanum",
        chassis_type: "mecanum",
        status: mecanumTask ? mecanumTask.status : "IDLE",
        current_task_id: mecanumTask?.task_id,
        current_task_label: mecanumTask?.label,
        last_update: now()
      },
      {
        robot_id: "ackermann",
        display_name: "ackermann",
        chassis_type: "ackermann",
        status: ackermannTask ? ackermannTask.status : "IDLE",
        current_task_id: ackermannTask?.task_id,
        current_task_label: ackermannTask?.label,
        last_update: now()
      }
    ],
    tasks: clone(tasks),
    resource_locks: tasks
      .filter((task) => task.status !== "COMPLETED" && task.target_points[0])
      .map((task) => ({
        point_id: task.target_points[0],
        point_label: task.target_points[0],
        holder_task_id: task.task_id,
        robot_id: task.robot_id,
        lock_type: task.task_type
      })),
    waiting_confirmations: waitingTasks.map((task) => ({
      task_id: task.task_id,
      step_index: 0,
      step_id: task.task_type === "DELIVERY" ? "step-pickup" : "step-p1",
      task_type: task.task_type,
      display_name: task.display_name,
      point_id: task.target_points[0] || "P1",
      point_label: task.target_points[0] || "P1",
      label: task.current_step_label || "等待确认",
      robot_id: task.robot_id,
      assigned_robot_id: task.assigned_robot_id,
      preferred_robot_id: task.preferred_robot_id
    }))
  };
}

function assignedRobotFor(template: TaskTemplate): "mecanum" | "ackermann" | undefined {
  const defaultRobot = template.task_type === "INSPECTION" ? "ackermann" : "mecanum";
  if (template.robot_preference === "auto") {
    return defaultRobot;
  }
  const preferredTask = tasks.find(
    (task) =>
      task.robot_id === template.robot_preference &&
      ["DELIVERY", "INSPECTION", "RECHECK"].includes(String(task.task_type)) &&
      !["COMPLETED", "SUCCEEDED", "CANCELED", "FAILED"].includes(task.status)
  );
  if (preferredTask) {
    return template.robot_preference === "mecanum" ? "ackermann" : "mecanum";
  }
  return template.robot_preference;
}

function listTemplates(): TemplateListResponse {
  return {
    templates: clone([...templates].sort((left, right) => left.sort_order - right.sort_order)),
    business_points: clone(businessPoints)
  };
}

export class MockRobotControlApiClient implements ApiClient {
  constructor() {
    resetMockState();
  }

  async getHealth(): Promise<HealthResponse> {
    return {
      backend_online: true,
      dispatch_online: true,
      dispatch_degraded: false,
      updated_at: now(),
      disabled_reasons: []
    };
  }

  async getState(): Promise<AggregateState> {
    return aggregateState();
  }

  async getTemplates(): Promise<TemplateListResponse> {
    return listTemplates();
  }

  async createTemplate(payload: TemplatePayload): Promise<TaskTemplate> {
    validateTemplatePayload(payload);
    const template: TaskTemplate = {
      template_id: `user-${Date.now()}-${templates.length}`,
      readonly: false,
      ...payload
    };
    templates = [...templates, template];
    pushLog({ level: "info", event: "template_created", message: `已创建模板 ${template.name}` });
    return clone(template);
  }

  async updateTemplate(templateId: string, payload: TemplatePayload): Promise<TaskTemplate> {
    const existing = templates.find((template) => template.template_id === templateId);
    if (!existing || existing.readonly) {
      throw new Error("Template is not editable");
    }
    validateTemplatePayload(payload);
    const updated = { ...existing, ...payload };
    templates = templates.map((template) => (template.template_id === templateId ? updated : template));
    pushLog({ level: "info", event: "template_updated", message: `已更新模板 ${updated.name}` });
    return clone(updated);
  }

  async deleteTemplate(templateId: string): Promise<CommandResult> {
    const existing = templates.find((template) => template.template_id === templateId);
    if (!existing || existing.readonly) {
      throw new Error("Template is not deletable");
    }
    templates = templates.filter((template) => template.template_id !== templateId);
    pushLog({ level: "warning", event: "template_deleted", message: `已删除模板 ${existing.name}` });
    return ok("模板已删除");
  }

  async reorderTemplates(templateIds: string[]): Promise<TemplateListResponse> {
    const order = new Map(templateIds.map((templateId, index) => [templateId, (index + 1) * 10]));
    templates = templates.map((template) => ({
      ...template,
      sort_order: order.get(template.template_id) ?? template.sort_order
    }));
    return listTemplates();
  }

  async triggerTemplate(templateId: string): Promise<TriggerTaskResult> {
    const template = templates.find((candidate) => candidate.template_id === templateId);
    if (!template) {
      throw new Error("Template not found");
    }
    const taskId = `task-${String(taskCounter++).padStart(3, "0")}`;
    const assignedRobotId = assignedRobotFor(template);
    const displayName = nextTaskDisplayName(template.task_type);
    tasks = [
      {
        task_id: taskId,
        task_type: template.task_type,
        display_name: displayName,
        label: template.name,
        status: "ASSIGNED",
        robot_id: assignedRobotId,
        assigned_robot_id: assignedRobotId,
        preferred_robot_id: template.robot_preference,
        target_points: template.target_points,
        created_at: now(),
        updated_at: now()
      },
      ...tasks
    ];
    pushLog({
      level: "info",
      event: "template_triggered",
      message: `已从 ${template.name} 创建 ${displayName}`,
      task_id: taskId
    });
    return {
      task_id: taskId,
      display_name: displayName,
      preferred_robot_id: template.robot_preference,
      assigned_robot_id: assignedRobotId,
      message: "任务已创建"
    };
  }

  async confirmStep(payload: ConfirmStepPayload): Promise<CommandResult> {
    tasks = tasks.map((task) =>
      task.task_id === payload.task_id ? { ...task, status: "COMPLETED", updated_at: now() } : task
    );
    pushLog({
      level: payload.result === "OK" ? "info" : "warning",
      event: "confirmation",
      message: `已提交 ${payload.result}`,
      task_id: payload.task_id
    });
    return ok("确认已提交");
  }

  async pauseTask(taskId: string): Promise<CommandResult> {
    tasks = tasks.map((task) => (task.task_id === taskId ? { ...task, status: "PAUSED" } : task));
    pushLog({ level: "warning", event: "task_paused", message: `已暂停 ${taskId}`, task_id: taskId });
    return ok("任务已暂停");
  }

  async resumeTask(taskId: string): Promise<CommandResult> {
    tasks = tasks.map((task) => (task.task_id === taskId ? { ...task, status: "EXECUTING" } : task));
    pushLog({ level: "info", event: "task_resumed", message: `已恢复 ${taskId}`, task_id: taskId });
    return ok("任务已恢复");
  }

  async cancelTask(taskId: string): Promise<CommandResult> {
    tasks = tasks.map((task) => (task.task_id === taskId ? { ...task, status: "CANCELED" } : task));
    pushLog({ level: "warning", event: "task_canceled", message: `已取消 ${taskId}`, task_id: taskId });
    return ok("任务已取消");
  }

  async emergencyStop(): Promise<CommandResult> {
    pushLog({ level: "error", event: "global_emergency_stop", message: "已请求全局急停" });
    return ok("已请求急停");
  }

  async getLogs(limit = 100): Promise<LogListResponse> {
    return { logs: clone(logs.slice(0, limit)) };
  }

  async getSystemStatus(): Promise<SystemStatusResponse> {
    return clone(systemStatus());
  }

  async startSystem(): Promise<SystemActionResponse> {
    systemStatusValue = "running";
    systemManaged = true;
    systemPid = 4242;
    systemPgid = 4242;
    systemStartedAt = now();
    pushLaunchLog("launch", "[INFO] real_robot_control_plane launch started");
    pushLaunchLog("stdout", "[INFO] robot_dispatch ready");
    const log = pushSystemOperation({
      level: "info",
      event: "system_start",
      message: "调度系统启动中"
    });
    return {
      accepted: true,
      message: "调度系统启动中",
      status: clone(systemStatus()),
      log: clone(log)
    };
  }

  async stopSystem(): Promise<SystemActionResponse> {
    const log = pushSystemOperation({
      level: "warning",
      event: "system_stop",
      message: "调度系统停止中"
    });
    pushLaunchLog("launch", "[INFO] shutdown requested");
    systemStatusValue = "stopped";
    systemManaged = false;
    systemPid = null;
    systemPgid = null;
    systemStartedAt = "";
    return {
      accepted: true,
      message: "调度系统停止中",
      status: clone(systemStatus()),
      log: clone(log)
    };
  }

  async restartSystem(): Promise<SystemActionResponse> {
    const log = pushSystemOperation({
      level: "warning",
      event: "system_restart",
      message: "调度系统重启中"
    });
    pushLaunchLog("launch", "[INFO] restart requested");
    systemStatusValue = "running";
    systemManaged = true;
    systemPid = 4343;
    systemPgid = 4343;
    systemStartedAt = now();
    pushLaunchLog("stdout", "[INFO] robot_dispatch ready after restart");
    return {
      accepted: true,
      message: "调度系统重启中",
      status: clone(systemStatus()),
      log: clone(log)
    };
  }

  async getSystemLogs(limit = 120): Promise<SystemLogsResponse> {
    return {
      launch_logs: clone(launchLogs.slice(-limit)),
      operation_logs: clone(operationLogs.slice(0, limit))
    };
  }
}

export function connectMockStatusSocket(handlers: StatusSocketHandlers): StatusSocketHandle {
  let closed = false;
  const emit = () => {
    if (closed) {
      return;
    }
    handlers.onStateUpdate?.(aggregateState());
    handlers.onLogUpdate?.(clone(logs.slice(0, 20)));
  };

  window.setTimeout(() => {
    if (!closed) {
      handlers.onOpen?.();
      emit();
    }
  }, 0);

  const interval = window.setInterval(emit, 15000);
  return {
    close: () => {
      closed = true;
      window.clearInterval(interval);
      handlers.onClose?.();
    }
  };
}

export const mockApiClient = new MockRobotControlApiClient();
