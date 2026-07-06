import { describe, expect, it, vi } from "vitest";
import { RobotControlApiClient, connectStatusSocket } from "./client";
import type { AggregateState } from "../types";

function jsonResponse(body: unknown, init: ResponseInit = {}): Response {
  return new Response(JSON.stringify(body), {
    status: 200,
    headers: { "Content-Type": "application/json" },
    ...init
  });
}

describe("RobotControlApiClient", () => {
  it("uses same-origin /api paths for health and task commands", async () => {
    const fetcher = vi.fn((input: RequestInfo | URL) => {
      if (String(input) === "/api/health") {
        return Promise.resolve(
          jsonResponse({
            backend_online: true,
            dispatch_online: true,
            dispatch_degraded: false,
            updated_at: "2026-05-23T00:00:00Z",
            disabled_reasons: []
          })
        );
      }
      return Promise.resolve(jsonResponse({ ok: true, message: "paused" }));
    });
    const client = new RobotControlApiClient(fetcher as unknown as typeof fetch);

    await client.getHealth();
    await client.pauseTask("task-1");

    expect(fetcher).toHaveBeenNthCalledWith(
      1,
      "/api/health",
      expect.objectContaining({ headers: expect.any(Headers) })
    );
    expect(fetcher).toHaveBeenNthCalledWith(
      2,
      "/api/tasks/task-1/pause",
      expect.objectContaining({ method: "POST" })
    );
  });

  it("surfaces backend failure reasons", async () => {
    const fetcher = vi.fn(() =>
      Promise.resolve(jsonResponse({ detail: "dispatch offline" }, { status: 503 }))
    );
    const client = new RobotControlApiClient(fetcher as unknown as typeof fetch);

    await expect(client.triggerTemplate("t1")).rejects.toMatchObject({
      status: 503,
      message: "dispatch offline"
    });
  });

  it("normalizes robot_web gateway fields for the PWA stores", async () => {
    const fetcher = vi.fn((input: RequestInfo | URL) => {
      if (String(input) === "/api/state") {
        return Promise.resolve(
          jsonResponse({
            status: {
              backend_online: true,
              dispatch_online: true,
              dispatch_degraded: false,
              disabled_reasons: [],
              updated_at: "2026-05-23T00:00:00Z"
            },
            robots: [{ robot_id: "mecanum", state: "EXECUTING", chassis_type: "mecanum", current_task_id: "task-1" }],
            tasks: [
              {
                task_id: "task-1",
                display_name: "配送任务_1",
                task_type: "DELIVERY",
                state: "RUNNING",
                preferred_robot_id: "ackermann",
                assigned_robot_id: "mecanum",
                target_points: ["PICKUP_A", "DELIVERY_C"]
              }
            ],
            resource_locks: [{ point_id: "PICKUP_A", locked_by_task_id: "task-1", resource_type: "PICKUP" }],
            waiting_confirmations: [
              {
                task_id: "task-1",
                display_name: "配送任务_1",
                step_index: 0,
                step_id: "step-1",
                point_id: "PICKUP_A",
                task_type: "DELIVERY",
                preferred_robot_id: "ackermann",
                assigned_robot_id: "mecanum"
              }
            ]
          })
        );
      }
      if (String(input) === "/api/templates/builtin_delivery_demo/trigger") {
        return Promise.resolve(
          jsonResponse({
            task_id: "task-2",
            display_name: "配送任务_2",
            preferred_robot_id: "ackermann",
            assigned_robot_id: "mecanum",
            message: "任务已创建"
          })
        );
      }
      return Promise.resolve(
        jsonResponse({
          templates: [
            {
              template_id: "builtin_delivery_demo",
              display_name: "配送演示",
              task_type: "DELIVERY",
              preferred_robot_id: "ackermann",
              target_point_ids: ["PICKUP_A", "DELIVERY_C"],
              builtin: true,
              available: false,
              unavailable_reason: "missing_task_points",
              missing_point_ids: ["DELIVERY_C"],
              sort_order: 10
            }
          ],
          business_points: [{ point_id: "PICKUP_A", label: "A 取货点", point_type: "pickup" }]
        })
      );
    });
    const client = new RobotControlApiClient(fetcher as unknown as typeof fetch);

    const state = await client.getState();
    const templates = await client.getTemplates();
    const triggered = await client.triggerTemplate("builtin_delivery_demo");

    expect(state.robots[0]).toMatchObject({ robot_id: "mecanum", display_name: "mecanum", status: "EXECUTING" });
    expect(state.tasks[0]).toMatchObject({
      task_id: "task-1",
      display_name: "配送任务_1",
      status: "RUNNING",
      robot_id: "mecanum",
      preferred_robot_id: "ackermann",
      assigned_robot_id: "mecanum",
      target_points: ["PICKUP_A", "DELIVERY_C"]
    });
    expect(state.waiting_confirmations[0]).toMatchObject({
      step_index: 0,
      point_label: "PICKUP_A",
      display_name: "配送任务_1",
      preferred_robot_id: "ackermann",
      assigned_robot_id: "mecanum"
    });
    expect(templates.templates[0]).toMatchObject({
      name: "配送演示",
      robot_preference: "ackermann",
      target_points: ["PICKUP_A", "DELIVERY_C"],
      readonly: true,
      available: false,
      unavailable_reason: "missing_task_points",
      missing_point_ids: ["DELIVERY_C"]
    });
    expect(templates.business_points[0]).toMatchObject({ point_type: "pickup" });
    expect(triggered).toMatchObject({
      task_id: "task-2",
      display_name: "配送任务_2",
      preferred_robot_id: "ackermann",
      assigned_robot_id: "mecanum"
    });
  });

  it("uses fixed system-control endpoints and normalizes status plus logs", async () => {
    const fetcher = vi.fn((input: RequestInfo | URL) => {
      if (String(input) === "/api/system/status") {
        return Promise.resolve(
          jsonResponse({
            status: "external",
            summary: "检测到外部运行中的调度系统",
            managed: false,
            external_running: true,
            can_start: true,
            can_stop: true,
            can_restart: true,
            profile: {
              id: "real_robot_control_plane",
              name: "Real Robot Control Plane",
              command: "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
            },
            pid: null,
            pgid: null,
            started_at: "",
            updated_at: "2026-05-23T00:00:00Z",
            health: [{ id: "process.map_server", label: "PC map_server", category: "process", status: "missing", required: true, detail: "missing" }]
          })
        );
      }
      if (String(input) === "/api/system/logs?limit=5") {
        return Promise.resolve(
          jsonResponse({
            launch_logs: [{ line_no: 1, stream: "launch", message: "ready", timestamp: "" }],
            operation_logs: [{ log_id: "1", timestamp: "2026-05-23T00:00:00Z", level: "info", event: "system_start", message: "调度系统启动中" }]
          })
        );
      }
      return Promise.resolve(jsonResponse({ accepted: true, message: "调度系统启动中", status: { status: "starting" } }));
    });
    const client = new RobotControlApiClient(fetcher as unknown as typeof fetch);

    const status = await client.getSystemStatus();
    const logs = await client.getSystemLogs(5);
    await client.startSystem();

    expect(status.external_running).toBe(true);
    expect(status.can_stop).toBe(false);
    expect(status.can_restart).toBe(false);
    expect(status.health[0]).toMatchObject({ status: "missing", label: "PC map_server" });
    expect(logs.launch_logs[0]).toMatchObject({ line_no: 1, message: "ready" });
    expect(logs.operation_logs[0]).toMatchObject({ event: "system_start" });
    expect(fetcher).toHaveBeenCalledWith("/api/system/start", expect.objectContaining({ method: "POST" }));
  });
});

describe("connectStatusSocket", () => {
  it("routes state_update and log_update messages", () => {
    const state: AggregateState = {
      status: {
        backend_online: true,
        dispatch_online: true,
        dispatch_degraded: false,
        websocket_online: true,
        disabled_reasons: [],
        updated_at: "2026-05-23T00:00:00Z"
      },
      robots: [],
      tasks: [],
      resource_locks: [],
      waiting_confirmations: []
    };

    class FakeSocket extends EventTarget {
      static instances: FakeSocket[] = [];
      readonly url: string;

      constructor(url: string) {
        super();
        this.url = url;
        FakeSocket.instances.push(this);
      }

      close(): void {
        this.dispatchEvent(new Event("close"));
      }

      emit(data: unknown): void {
        this.dispatchEvent(new MessageEvent("message", { data: JSON.stringify(data) }));
      }
    }

    const onStateUpdate = vi.fn();
    const onLogUpdate = vi.fn();
    const handle = connectStatusSocket(
      {
        onStateUpdate,
        onLogUpdate
      },
      FakeSocket as unknown as typeof WebSocket
    );
    const socket = FakeSocket.instances[0];

    socket.emit({ type: "state_update", state });
    socket.emit({
      type: "log_update",
      log: {
        id: 8,
        created_at: "2026-06-18T04:20:00Z",
        level: "ERROR",
        event_type: "demo_fire_alert",
        message: "发现当前巡检点 P3 有火情",
        detail: {
          robot_id: "ackermann",
          point_id: "P3",
          warning_active: true,
          warning_message: "P3 火灾警告"
        }
      }
    });

    expect(handle).toBeDefined();
    expect(socket.url).toBe("ws://localhost:3000/ws/status");
    expect(onStateUpdate).toHaveBeenCalledWith(state);
    expect(onLogUpdate).toHaveBeenCalledWith([
      expect.objectContaining({
        log_id: "8",
        level: "error",
        event: "demo_fire_alert",
        robot_id: "ackermann",
        detail: expect.objectContaining({
          warning_active: true,
          warning_message: "P3 火灾警告"
        })
      })
    ]);
  });
});
