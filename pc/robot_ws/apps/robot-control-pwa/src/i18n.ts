const TASK_TYPE_LABELS: Record<string, string> = {
  DELIVERY: "配送",
  INSPECTION: "巡检",
  RECHECK: "复查"
};

const STATUS_LABELS: Record<string, string> = {
  CREATED: "已创建",
  PENDING: "待调度",
  ASSIGNED: "已分配",
  RUNNING: "执行中",
  EXECUTING: "执行中",
  WAITING_CONFIRMATION: "等待确认",
  PAUSED: "已暂停",
  RESUMING: "恢复中",
  SUCCEEDED: "已完成",
  COMPLETED: "已完成",
  FAILED: "失败",
  CANCELED: "已取消",
  WAITING_RESOURCE: "等待资源",
  IDLE: "空闲",
  RETURNING_HOME: "返回等待区",
  ESTOP: "急停",
  ERROR: "故障"
};

const CHASSIS_LABELS: Record<string, string> = {
  mecanum: "麦克纳木",
  ackermann: "阿克曼"
};

const LOG_LEVEL_LABELS: Record<string, string> = {
  info: "信息",
  warning: "警告",
  error: "错误"
};

const LOG_EVENT_LABELS: Record<string, string> = {
  mock_started: "模拟网关启动",
  template_created: "模板已创建",
  template_updated: "模板已更新",
  template_deleted: "模板已删除",
  template_triggered: "模板已触发",
  template_trigger_failed: "模板触发失败",
  confirmation: "确认已提交",
  confirmation_submitted: "确认已提交",
  confirmation_failed: "确认失败",
  task_pause: "任务已暂停",
  task_pause_failed: "任务暂停失败",
  task_resume: "任务已恢复",
  task_resume_failed: "任务恢复失败",
  task_cancel: "任务已取消",
  task_cancel_failed: "任务取消失败",
  task_paused: "任务已暂停",
  task_resumed: "任务已恢复",
  task_canceled: "任务已取消",
  global_estop: "全局急停",
  global_estop_failed: "全局急停失败",
  global_emergency_stop: "全局急停",
  demo_pickup_arrived: "到达取货点",
  demo_delivery_arrived: "到达配送点",
  demo_ackermann_pickup_arrived: "到达取货点",
  demo_ackermann_delivery_arrived: "到达配送点",
  demo_mecanum_heading_pickup: "前往取货点",
  demo_ackermann_heading_pickup: "前往取货点",
  demo_mecanum_heading_delivery_d: "前往配送点",
  demo_ackermann_heading_delivery_c: "前往配送点",
  demo_ackermann_heading_inspection: "前往巡检点",
  demo_mecanum_heading_inspection_p3: "前往巡检点",
  demo_inspection_started: "开始巡检",
  demo_inspection_normal: "巡检正常",
  demo_recheck_smoke: "烟雾异常",
  demo_recheck_stack_risk: "货物堆叠异常",
  demo_recheck_confirmed: "复检确认",
  demo_fire_alert: "火灾警告",
  demo_fire_alert_cleared: "火灾告警关闭",
  demo_ackermann_returning_home: "返回等待区",
  demo_mecanum_returning_home: "返回等待区",
  demo_demo_started: "演示启动",
  demo_demo_state: "演示正常运行",
  demo_logs_cleared: "事件日志已清空",
  system_start: "系统启动",
  system_start_failed: "系统启动失败",
  system_stop: "系统停止",
  system_stop_failed: "系统停止失败",
  system_restart: "系统重启",
  system_restart_failed: "系统重启失败",
  system_external_running: "外部运行",
  system_recovered: "系统管理权恢复"
};

const COMMAND_REASON_LABELS: Record<string, string> = {
  "Backend offline": "后端离线",
  "Dispatch degraded": "调度服务降级",
  "Dispatch offline": "调度服务离线",
  "WebSocket disconnected": "实时连接断开",
  dispatch_offline: "调度服务离线",
  global_estop_active: "全局急停中"
};

export function taskTypeLabel(value?: string): string {
  return value ? TASK_TYPE_LABELS[value] ?? value : "未知类型";
}

export function statusLabel(value?: string): string {
  return value ? STATUS_LABELS[value] ?? value : "未知状态";
}

export function chassisLabel(value?: string): string {
  return value ? CHASSIS_LABELS[value] ?? value : "未知底盘";
}

export function robotPreferenceLabel(value?: string): string {
  if (value === "auto") {
    return "自动";
  }
  return value || "自动";
}

export function confirmationResultLabel(value: string): string {
  if (value === "OK") {
    return "正常";
  }
  if (value === "ABNORMAL") {
    return "异常";
  }
  if (value === "REJECT") {
    return "拒绝";
  }
  return value;
}

export function logLevelLabel(value: string): string {
  return LOG_LEVEL_LABELS[value] ?? value;
}

export function logEventLabel(value: string): string {
  return LOG_EVENT_LABELS[value] ?? value;
}

export function commandReasonLabel(value: string): string {
  return COMMAND_REASON_LABELS[value] ?? value;
}

const SYSTEM_STATUS_LABELS: Record<string, string> = {
  stopped: "已停止",
  starting: "启动中",
  running: "运行中",
  degraded: "降级",
  stopping: "停止中",
  failed: "失败",
  external: "外部运行"
};

const SYSTEM_HEALTH_LABELS: Record<string, string> = {
  ok: "正常",
  missing: "缺失",
  failed: "失败",
  not_checked: "未检查"
};

const SYSTEM_CATEGORY_LABELS: Record<string, string> = {
  process: "进程",
  node: "节点",
  service: "服务",
  topic: "话题",
  forbidden_topic: "禁止话题",
  system: "系统"
};

const SYSTEM_HEALTH_ITEM_LABELS: Record<string, string> = {
  "process.managed_launch": "App 管理启动进程",
  "process.map_server": "PC 地图服务",
  "process.rviz2": "RViz 可视化",
  "node.robot_dispatch": "调度节点",
  "node.map_server": "地图节点",
  "node.mecanum_mission_executor": "mecanum 任务执行节点",
  "node.ackermann_mission_executor": "ackermann 任务执行节点",
  "service.robot_dispatch_create_task": "创建任务服务",
  "service.robot_dispatch_get_state": "状态查询服务",
  "service.robot_dispatch_enable_system": "启用系统服务",
  "service.robot_dispatch_recover_system": "恢复系统服务",
  "service.robot_dispatch_pause_task": "暂停任务服务",
  "service.robot_dispatch_resume_task": "恢复任务服务",
  "service.robot_dispatch_cancel_task": "取消任务服务",
  "service.robot_dispatch_emergency_stop": "急停服务",
  "topic.map": "共享地图话题",
  "topic.system_state": "系统状态话题",
  "topic.mecanum_heartbeat": "mecanum 心跳话题",
  "topic.mecanum_scan": "mecanum 雷达话题",
  "topic.mecanum_odom": "mecanum 里程计话题",
  "topic.mecanum_cmd_vel": "mecanum 速度指令话题",
  "topic.ackermann_heartbeat": "ackermann 心跳话题",
  "topic.ackermann_scan": "ackermann 雷达话题",
  "topic.ackermann_odom": "ackermann 里程计话题",
  "topic.ackermann_cmd_vel": "ackermann 速度指令话题",
  "topic.robot_dispatch_markers": "调度可视化标记话题",
  "forbidden.cmd_vel": "禁止全局 /cmd_vel",
  "forbidden.odom": "禁止全局 /odom",
  "forbidden.scan": "禁止全局 /scan"
};

export function systemStatusLabel(value: string): string {
  return SYSTEM_STATUS_LABELS[value] ?? value;
}

export function systemHealthLabel(value: string): string {
  return SYSTEM_HEALTH_LABELS[value] ?? value;
}

export function systemCategoryLabel(value: string): string {
  return SYSTEM_CATEGORY_LABELS[value] ?? value;
}

export function systemHealthItemLabel(id: string, fallback: string): string {
  return SYSTEM_HEALTH_ITEM_LABELS[id] ?? fallback;
}

export function systemHealthDetailLabel(value?: string): string {
  if (!value) {
    return "-";
  }
  if (value === "ROS graph not checked") {
    return "ROS 图未检查";
  }
  if (value === "no matching process") {
    return "未找到匹配进程";
  }
  if (value === "no App-managed launch process") {
    return "没有 App 管理的启动进程";
  }
  const missingMatch = value.match(/^(\/\S+) missing$/);
  if (missingMatch) {
    return `缺少 ${missingMatch[1]}`;
  }
  const absentMatch = value.match(/^(\/\S+) absent$/);
  if (absentMatch) {
    return `${absentMatch[1]} 不存在`;
  }
  const forbiddenMatch = value.match(/^(\/\S+) must not exist$/);
  if (forbiddenMatch) {
    return `${forbiddenMatch[1]} 不允许存在`;
  }
  return value;
}
