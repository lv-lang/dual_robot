import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { flushPromises, mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import OverviewView from "./OverviewView.vue";
import { useAggregateStore } from "../stores/aggregate";
import type { AggregateState } from "../types";

const situationState: AggregateState = {
  status: {
    backend_online: true,
    dispatch_online: true,
    dispatch_degraded: false,
    websocket_online: true,
    disabled_reasons: [],
    updated_at: "2026-05-24T00:00:00Z"
  },
  robots: [
    {
      robot_id: "mecanum",
      display_name: "mecanum 麦克纳木车",
      chassis_type: "mecanum",
      status: "EXECUTING",
      current_task_id: "task_delivery_1",
      current_task_label: "前往 A"
    },
    {
      robot_id: "ackermann",
      display_name: "ackermann 阿克曼车",
      chassis_type: "ackermann",
      status: "WAITING_CONFIRMATION",
      current_task_id: "task_inspection_2",
      current_task_label: "等待 P2 确认"
    }
  ],
  tasks: [
    {
      task_id: "task_delivery_1",
      task_type: "DELIVERY",
      display_name: "配送任务_1",
      label: "task_delivery_1",
      status: "EXECUTING",
      robot_id: "mecanum",
      target_points: ["W1", "A", "C"]
    },
    {
      task_id: "task_inspection_2",
      task_type: "INSPECTION",
      display_name: "巡检任务_2",
      label: "task_inspection_2",
      status: "WAITING_CONFIRMATION",
      robot_id: "ackermann",
      target_points: ["W2", "P1", "P2", "P3"]
    }
  ],
  resource_locks: [
    {
      point_id: "A",
      point_label: "A 取货点",
      holder_task_id: "task_delivery_1",
      robot_id: "mecanum",
      lock_type: "pickup"
    }
  ],
  waiting_confirmations: [
    {
      task_id: "task_inspection_2",
      step_index: 2,
      step_id: "inspect-p2",
      task_type: "INSPECTION",
      display_name: "巡检任务_2",
      point_id: "P2",
      point_label: "P2 巡检点",
      label: "P2 异常标志确认",
      robot_id: "ackermann"
    }
  ]
};

function mountOverview(state: AggregateState = situationState) {
  const pinia = createPinia();
  setActivePinia(pinia);
  useAggregateStore().applyState(state);

  return mount(OverviewView, {
    global: {
      plugins: [pinia]
    }
  });
}

describe("OverviewView", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
    vi.stubGlobal(
      "fetch",
      vi.fn(async (input: RequestInfo | URL) => {
        const path = String(input);
        const body = path.includes("/api/task-points")
          ? {
              points: [
                { point_id: "W1", kind: "WAITING_AREA", label: "W1 等待区", x: -8.5, y: -3.8, yaw: 0, has_pose: true },
                { point_id: "A", kind: "PICKUP", label: "A 取货点", x: -7.5, y: -3.8, yaw: 0, has_pose: true },
                { point_id: "C", kind: "DELIVERY", label: "C 配送点", x: -9.0, y: -3.8, yaw: 0, has_pose: true },
                { point_id: "P2", kind: "INSPECTION", label: "P2 巡检点", x: -8.0, y: 2.5, yaw: 0, has_pose: true }
              ]
            }
          : {
              available: false,
              resolution: 0.03,
              origin: [-9.99, -9.99, 0],
              width: 992,
              height: 640
            };
        return {
          ok: true,
          json: async () => body
        };
      })
    );
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("uses the situation label and exposes the first-screen 2D field map", async () => {
    const wrapper = mountOverview();
    await flushPromises();

    expect(wrapper.find("h1").text()).toBe("态势");

    const fieldMap = wrapper.find("[aria-label='二维场地态势图']");
    expect(fieldMap.exists()).toBe(true);
    expect(fieldMap.text()).toContain("mecanum");
    expect(fieldMap.text()).toContain("ackermann");
    expect(fieldMap.text()).toContain("P2");
  });

  it("prioritizes waiting confirmations ahead of the running task list", async () => {
    const wrapper = mountOverview();
    await flushPromises();
    const text = wrapper.text();
    const sectionHeadings = wrapper.findAll("h2").map((heading) => heading.text().trim());
    const confirmationIndex = sectionHeadings.indexOf("待确认");
    const currentTaskIndex = sectionHeadings.indexOf("当前任务");

    expect(sectionHeadings).toContain("待确认");
    expect(text).toContain("P2 异常标志确认");
    expect(confirmationIndex).toBeGreaterThanOrEqual(0);
    expect(currentTaskIndex).toBeGreaterThanOrEqual(0);
    expect(confirmationIndex).toBeLessThan(currentTaskIndex);
  });
});
