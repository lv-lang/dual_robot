import { beforeEach, describe, expect, it, vi } from "vitest";
import { flushPromises, mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import SystemView from "./SystemView.vue";
import { useSystemStore } from "../stores/system";

const api = vi.hoisted(() => ({
  getSystemStatus: vi.fn(),
  getSystemLogs: vi.fn(),
  startSystem: vi.fn(),
  stopSystem: vi.fn(),
  restartSystem: vi.fn(),
  getState: vi.fn()
}));

vi.mock("../api", () => ({ apiClient: api }));

function stoppedStatus() {
  return {
    status: "stopped" as const,
    summary: "调度系统未启动",
    managed: false,
    external_running: false,
    can_start: true,
    can_stop: false,
    can_restart: false,
    profile: {
      id: "real_robot_control_plane",
      name: "Real Robot Control Plane",
      command: "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
    },
    pid: null,
    pgid: null,
    started_at: "",
    updated_at: "2026-05-23T00:00:00Z",
    health: [
      {
        id: "process.map_server",
        label: "PC map_server",
        category: "process",
        status: "missing" as const,
        required: true,
        detail: "no matching process"
      }
    ]
  };
}

function logs() {
  return {
    launch_logs: [{ line_no: 1, stream: "launch", message: "fake launch ready", timestamp: "" }],
    operation_logs: [{
      log_id: "1",
      timestamp: "2026-05-23T00:00:00Z",
      level: "info" as const,
      event: "system_start",
      message: "调度系统启动中"
    }]
  };
}

function aggregateState() {
  return {
    status: {
      backend_online: true,
      dispatch_online: false,
      dispatch_degraded: true,
      disabled_reasons: [],
      updated_at: "2026-05-23T00:00:00Z"
    },
    robots: [],
    tasks: [],
    resource_locks: [],
    waiting_confirmations: []
  };
}

describe("SystemView", () => {
  beforeEach(() => {
    for (const mock of Object.values(api)) {
      mock.mockReset();
    }
    api.getSystemStatus.mockResolvedValue(stoppedStatus());
    api.getSystemLogs.mockResolvedValue(logs());
    api.getState.mockResolvedValue(aggregateState());
  });

  it("renders status, health, logs, and starts through the API", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    api.startSystem.mockResolvedValue({
      accepted: true,
      message: "调度系统启动中",
      status: { ...stoppedStatus(), status: "starting", managed: true, can_start: false, can_stop: true },
      log: logs().operation_logs[0]
    });

    const wrapper = mount(SystemView, { global: { plugins: [pinia] } });
    await flushPromises();

    expect(wrapper.text()).toContain("系统");
    expect(wrapper.text()).toContain("调度系统未启动");
    expect(wrapper.text()).toContain("PC 地图服务");
    expect(wrapper.text()).not.toContain("未找到匹配进程");
    expect(wrapper.text()).toContain("fake launch ready");
    expect(wrapper.text()).not.toContain("real_robot_control_plane.launch.py");

    await wrapper.findAll("button").find((button) => button.text().includes("启动调度系统"))?.trigger("click");

    expect(api.startSystem).toHaveBeenCalledTimes(1);
  });

  it("hides launch command and health detail notes from the operator summary", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    api.getSystemStatus.mockResolvedValue({
      ...stoppedStatus(),
      health: [
        {
          id: "process.rviz2",
          label: "RViz",
          category: "process",
          status: "ok",
          required: false,
          detail: "pid=620202"
        },
        {
          id: "process.map_server",
          label: "PC map_server",
          category: "process",
          status: "ok",
          required: true,
          detail: "pid=620201"
        }
      ]
    });

    const wrapper = mount(SystemView, { global: { plugins: [pinia] } });
    await flushPromises();

    expect(wrapper.text()).not.toContain("real_robot_control_plane.launch.py");
    expect(wrapper.text()).not.toContain("RViz");
    expect(wrapper.text()).not.toContain("pid=620201");
    expect(wrapper.text()).not.toContain("pid=620202");
  });

  it("auto-refreshes system status and logs only while the system page is mounted", async () => {
    vi.useFakeTimers();
    try {
      const pinia = createPinia();
      setActivePinia(pinia);

      const wrapper = mount(SystemView, { global: { plugins: [pinia] } });
      await flushPromises();

      expect(api.getSystemStatus).toHaveBeenCalledTimes(1);
      expect(api.getSystemLogs).toHaveBeenCalledTimes(1);

      await vi.advanceTimersByTimeAsync(1000);
      expect(api.getSystemStatus).toHaveBeenCalledTimes(2);
      expect(api.getSystemLogs).toHaveBeenCalledTimes(2);

      wrapper.unmount();
      await vi.advanceTimersByTimeAsync(3000);
      expect(api.getSystemStatus).toHaveBeenCalledTimes(2);
      expect(api.getSystemLogs).toHaveBeenCalledTimes(2);
    } finally {
      vi.useRealTimers();
    }
  });

  it("disables stop and restart for external-running systems", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    const system = useSystemStore();
    system.applyStatus({
      ...stoppedStatus(),
      status: "external",
      summary: "检测到外部运行中的调度系统",
      external_running: true,
      can_start: false,
      can_stop: false,
      can_restart: false
    });

    const wrapper = mount(SystemView, { global: { plugins: [pinia] } });

    expect(wrapper.text()).toContain("外部运行中的调度系统");
    expect(wrapper.findAll("button").find((button) => button.text().includes("停止调度系统"))?.attributes("disabled")).toBeDefined();
    expect(wrapper.findAll("button").find((button) => button.text().includes("重启调度系统"))?.attributes("disabled")).toBeDefined();
  });

  it("confirms dangerous stop actions", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    const system = useSystemStore();
    system.applyStatus({
      ...stoppedStatus(),
      status: "running",
      summary: "调度系统运行中",
      managed: true,
      can_start: false,
      can_stop: true,
      can_restart: true
    });

    const wrapper = mount(SystemView, { global: { plugins: [pinia] } });
    await wrapper.findAll("button").find((button) => button.text().includes("停止调度系统"))?.trigger("click");

    expect(document.body.textContent).toContain("当前任务会中断");
    expect(document.body.textContent).toContain("确认停止");
  });
});
