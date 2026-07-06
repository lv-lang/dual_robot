import { beforeEach, describe, expect, it, vi } from "vitest";
import { flushPromises, mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import TasksView from "./TasksView.vue";
import { apiClient } from "../api";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import { useTemplatesStore } from "../stores/templates";
import type { AggregateState } from "../types";

const baseStatus = {
  backend_online: true,
  dispatch_online: true,
  dispatch_degraded: false,
  websocket_online: true,
  disabled_reasons: [],
  updated_at: "2026-05-23T00:00:00Z"
};

describe("TasksView", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("warns about preference fallback and then shows the actual assigned robot", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    const aggregate = useAggregateStore();
    const templates = useTemplatesStore();
    useConnectionStore().applyAggregateStatus(baseStatus);

    const initialState: AggregateState = {
      status: baseStatus,
      robots: [
        {
          robot_id: "ackermann",
          display_name: "ackermann",
          chassis_type: "ackermann",
          status: "WAITING_CONFIRMATION",
          current_task_id: "task_1"
        }
      ],
      tasks: [
        {
          task_id: "task_1",
          task_type: "INSPECTION",
          display_name: "巡检任务_1",
          label: "task_1",
          status: "WAITING_CONFIRMATION",
          robot_id: "ackermann",
          assigned_robot_id: "ackermann",
          target_points: ["P1"]
        }
      ],
      resource_locks: [],
      waiting_confirmations: []
    };
    const createdState: AggregateState = {
      ...initialState,
      tasks: [
        {
          task_id: "task_2",
          task_type: "DELIVERY",
          display_name: "配送任务_2",
          label: "task_2",
          status: "ASSIGNED",
          robot_id: "mecanum",
          assigned_robot_id: "mecanum",
          preferred_robot_id: "ackermann",
          target_points: ["A", "C"]
        },
        ...initialState.tasks
      ]
    };
    aggregate.applyState(initialState);
    templates.templates = [
      {
        template_id: "delivery-ackermann",
        name: "配送 A 到 C",
        task_type: "DELIVERY",
        robot_preference: "ackermann",
        target_points: ["A", "C"],
        readonly: true,
        sort_order: 10
      }
    ];
    templates.businessPoints = [];

    let resolveTrigger!: (value: Awaited<ReturnType<typeof apiClient.triggerTemplate>>) => void;
    vi.spyOn(apiClient, "triggerTemplate").mockReturnValue(
      new Promise((resolve) => {
        resolveTrigger = resolve;
      })
    );
    vi.spyOn(apiClient, "getState").mockResolvedValue(createdState);
    vi.spyOn(apiClient, "getLogs").mockResolvedValue({ logs: [] });

    const wrapper = mount(TasksView, { global: { plugins: [pinia] } });
    await wrapper.findAll("button").find((button) => button.text() === "触发")?.trigger("click");
    await flushPromises();

    expect(wrapper.text()).toContain("当前选择 ackermann 正在执行配送/巡检任务");

    resolveTrigger({
      task_id: "task_2",
      display_name: "配送任务_2",
      preferred_robot_id: "ackermann",
      assigned_robot_id: "mecanum",
      message: "任务已创建"
    });
    await flushPromises();

    expect(wrapper.text()).toContain("配送任务_2");
    expect(wrapper.text()).toContain("已由 mecanum 执行");
  });
});
