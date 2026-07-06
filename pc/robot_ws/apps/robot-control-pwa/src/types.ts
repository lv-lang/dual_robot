export type RobotId = "mecanum" | "ackermann";
export type ChassisType = "mecanum" | "ackermann";
export type RobotPreference = "auto" | RobotId;
export type TaskType = "DELIVERY" | "INSPECTION" | "RECHECK";
export type ConfirmationResult = "OK" | "ABNORMAL" | "REJECT";
export type LogLevel = "info" | "warning" | "error";
export type BusinessPointType = "waiting" | "pickup" | "delivery" | "inspection";
export type SystemStatusValue = "stopped" | "starting" | "running" | "degraded" | "stopping" | "failed" | "external";
export type SystemHealthStatus = "ok" | "missing" | "failed" | "not_checked";

export interface HealthResponse {
  backend_online: boolean;
  dispatch_online: boolean;
  dispatch_degraded: boolean;
  reason?: string;
  updated_at: string;
  disabled_reasons: string[];
}

export interface AggregateStatus {
  backend_online: boolean;
  dispatch_online: boolean;
  dispatch_degraded: boolean;
  websocket_online?: boolean;
  disabled_reasons: string[];
  updated_at: string;
}

export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

export interface RobotSummary {
  robot_id: RobotId;
  display_name: string;
  chassis_type: ChassisType;
  status: string;
  current_task_id?: string;
  current_task_label?: string;
  last_update?: string;
  pose?: RobotPose;
}

export interface TaskPoint {
  point_id: string;
  kind: string;
  label: string;
  x: number;
  y: number;
  yaw: number;
  has_pose: boolean;
}

export interface CameraFeed {
  robot_id: string;
  label: string;
  stream_url: string;
  kind: string;
}

export interface MapData {
  available: boolean;
  resolution: number;
  origin: [number, number, number];
  width?: number;
  height?: number;
  source_width?: number;
  source_height?: number;
  crop?: {
    enabled: boolean;
    x: number;
    y: number;
    width: number;
    height: number;
  };
  image_base64?: string;
  image_mime?: string;
  map_version?: string;
  reason?: string;
}

export interface MissionStepSummary {
  sequence: number;
  step_type: string;
  point_id: string;
  label?: string;
}

export interface TaskSummary {
  task_id: string;
  task_type: TaskType | string;
  display_name?: string;
  label: string;
  status: string;
  robot_id?: RobotId;
  preferred_robot_id?: RobotPreference;
  assigned_robot_id?: RobotId;
  current_step_label?: string;
  current_step_index?: number;
  steps?: MissionStepSummary[];
  target_points: string[];
  created_at?: string;
  updated_at?: string;
}

export interface ResourceLock {
  point_id: string;
  point_label: string;
  holder_task_id: string;
  robot_id?: RobotId;
  lock_type: string;
}

export interface ConfirmationStep {
  task_id: string;
  step_index?: number;
  step_id: string;
  task_type: TaskType | string;
  display_name?: string;
  point_id: string;
  point_label: string;
  label: string;
  robot_id?: RobotId;
  preferred_robot_id?: RobotPreference;
  assigned_robot_id?: RobotId;
}

export interface AggregateState {
  status: AggregateStatus;
  robots: RobotSummary[];
  tasks: TaskSummary[];
  resource_locks: ResourceLock[];
  waiting_confirmations: ConfirmationStep[];
}

export interface BusinessPoint {
  point_id: string;
  label: string;
  point_type: BusinessPointType;
}

export interface TaskTemplate {
  template_id: string;
  name: string;
  task_type: TaskType;
  robot_preference: RobotPreference;
  target_points: string[];
  readonly: boolean;
  sort_order: number;
  available?: boolean;
  unavailable_reason?: string;
  missing_point_ids?: string[];
}

export interface TemplatePayload {
  name: string;
  task_type: TaskType;
  robot_preference: RobotPreference;
  target_points: string[];
  sort_order: number;
}

export interface TemplateListResponse {
  templates: TaskTemplate[];
  business_points: BusinessPoint[];
}

export interface TriggerTaskResult {
  task_id: string;
  message: string;
  display_name?: string;
  preferred_robot_id?: RobotPreference;
  assigned_robot_id?: RobotId;
}

export interface CommandResult {
  ok: boolean;
  message: string;
}

export interface LogEntry {
  log_id: string;
  timestamp: string;
  level: LogLevel;
  event: string;
  message: string;
  task_id?: string;
  task_display_name?: string;
  robot_id?: RobotId;
  detail?: Record<string, unknown>;
}

export interface LogListResponse {
  logs: LogEntry[];
}

export interface SystemProfile {
  id: string;
  name: string;
  command: string;
}

export interface SystemHealthRow {
  id: string;
  label: string;
  category: string;
  status: SystemHealthStatus;
  required: boolean;
  detail: string;
}

export interface SystemStatusResponse {
  status: SystemStatusValue;
  summary: string;
  managed: boolean;
  external_running: boolean;
  can_start: boolean;
  can_stop: boolean;
  can_restart: boolean;
  profile: SystemProfile;
  pid: number | null;
  pgid: number | null;
  started_at: string;
  updated_at: string;
  health: SystemHealthRow[];
}

export interface SystemActionResponse {
  accepted: boolean;
  message: string;
  status: SystemStatusResponse;
  log?: LogEntry;
}

export interface SystemLaunchLogLine {
  line_no: number;
  stream: string;
  message: string;
  timestamp: string;
}

export interface SystemLogsResponse {
  launch_logs: SystemLaunchLogLine[];
  operation_logs: LogEntry[];
}

export type StatusSocketMessage =
  | { type: "state_update"; state: AggregateState }
  | { type: "log_update"; logs?: LogEntry[]; log?: LogEntry }
  | { type: "demo_warning_clear"; warning?: Record<string, unknown> };

export interface ConfirmStepPayload {
  task_id: string;
  step_index?: number;
  step_id: string;
  point_id?: string;
  result: ConfirmationResult;
}

export interface ApiClient {
  getHealth(): Promise<HealthResponse>;
  getState(): Promise<AggregateState>;
  getTemplates(): Promise<TemplateListResponse>;
  createTemplate(payload: TemplatePayload): Promise<TaskTemplate>;
  updateTemplate(templateId: string, payload: TemplatePayload): Promise<TaskTemplate>;
  deleteTemplate(templateId: string): Promise<CommandResult>;
  reorderTemplates(templateIds: string[]): Promise<TemplateListResponse>;
  triggerTemplate(templateId: string): Promise<TriggerTaskResult>;
  confirmStep(payload: ConfirmStepPayload): Promise<CommandResult>;
  pauseTask(taskId: string): Promise<CommandResult>;
  resumeTask(taskId: string): Promise<CommandResult>;
  cancelTask(taskId: string): Promise<CommandResult>;
  emergencyStop(): Promise<CommandResult>;
  getLogs(limit?: number): Promise<LogListResponse>;
  getSystemStatus(): Promise<SystemStatusResponse>;
  startSystem(): Promise<SystemActionResponse>;
  stopSystem(): Promise<SystemActionResponse>;
  restartSystem(): Promise<SystemActionResponse>;
  getSystemLogs(limit?: number): Promise<SystemLogsResponse>;
}

export interface StatusSocketHandle {
  close(): void;
}

export interface StatusSocketHandlers {
  onOpen?: () => void;
  onClose?: () => void;
  onError?: (error: Event | Error) => void;
  onStateUpdate?: (state: AggregateState) => void;
  onLogUpdate?: (logs: LogEntry[]) => void;
  onDemoWarningClear?: (warning?: Record<string, unknown>) => void;
}
