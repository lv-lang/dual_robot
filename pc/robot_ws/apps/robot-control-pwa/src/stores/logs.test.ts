import { beforeEach, describe, expect, it } from "vitest";
import { createPinia, setActivePinia } from "pinia";
import { useLogsStore } from "./logs";

describe("logs store demo warning state", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("restores active danger warning from logs until a clear message appears", () => {
    const logs = useLogsStore();

    logs.applyLogs([
      {
        log_id: "1",
        timestamp: "2026-06-18T04:20:00Z",
        level: "error",
        event: "demo_fire_alert",
        message: "发现当前巡检点 P3 有火情",
        detail: {
          point_id: "P3",
          warning_title: "WARNING",
          warning_message: "P3 火灾警告"
        }
      }
    ]);

    expect(logs.activeDemoWarning).toMatchObject({
      point_id: "P3",
      title: "WARNING",
      severity: "danger",
      message: "P3 火灾警告"
    });

    logs.clearDemoWarning({ point_id: "P3", warning_severity: "danger" });

    expect(logs.activeDemoWarning).toBeUndefined();
  });

  it("shows yellow recheck warning until confirmation clears it", () => {
    const logs = useLogsStore();

    logs.applyLogs([
      {
        log_id: "1",
        timestamp: "2026-06-18T04:20:00Z",
        level: "warning",
        event: "demo_recheck_smoke",
        message: "P3 发现烟雾异常，已分配 mecanum 前往复检",
        detail: {
          point_id: "P3",
          warning_active: true,
          warning_severity: "warning",
          warning_title: "WARNING",
          warning_message: "P3 烟雾异常，mecanum 正在复检"
        }
      }
    ]);

    expect(logs.activeDemoWarning).toMatchObject({
      point_id: "P3",
      severity: "warning",
      message: "P3 烟雾异常，mecanum 正在复检"
    });

    logs.applyLogs([
      {
        log_id: "2",
        timestamp: "2026-06-18T04:21:00Z",
        level: "warning",
        event: "demo_recheck_confirmed",
        message: "P3 复检确认异常，已通知对应区域安全负责人前去处理",
        detail: { point_id: "P3", warning_active: false, warning_severity: "warning" }
      }
    ]);

    expect(logs.activeDemoWarning).toBeUndefined();
  });

  it("clears log list when an empty websocket log update arrives", () => {
    const logs = useLogsStore();

    logs.applyLogs([
      {
        log_id: "1",
        timestamp: "2026-06-18T04:20:00Z",
        level: "info",
        event: "demo_pickup_arrived",
        message: "已确认到达取货点 A"
      }
    ]);

    logs.applyLogs([]);

    expect(logs.latest).toHaveLength(0);
  });
});
