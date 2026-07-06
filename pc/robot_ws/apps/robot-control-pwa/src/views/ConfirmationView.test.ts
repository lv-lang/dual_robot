import { beforeEach, describe, expect, it, vi } from "vitest";
import { flushPromises, mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import ConfirmationView from "./ConfirmationView.vue";
import { apiClient } from "../api";
import { useAggregateStore } from "../stores/aggregate";
import { useConnectionStore } from "../stores/connection";
import type { AggregateState } from "../types";

describe("ConfirmationView", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("renders only waiting confirmation actions and disables them when disconnected", () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    const aggregate = useAggregateStore();
    aggregate.applyState({
      status: {
        backend_online: true,
        dispatch_online: true,
        dispatch_degraded: false,
        websocket_online: false,
        disabled_reasons: [],
        updated_at: "2026-05-23T00:00:00Z"
      },
      robots: [],
      tasks: [],
      resource_locks: [],
      waiting_confirmations: [
        {
          task_id: "task-1",
          step_id: "step-1",
          task_type: "INSPECTION",
          point_id: "P1",
          point_label: "P1",
          label: "确认 P1",
          robot_id: "ackermann"
        }
      ]
    });
    useConnectionStore().setWebSocketOnline(false);

    const wrapper = mount(ConfirmationView, { global: { plugins: [pinia] } });

    expect(wrapper.text()).toContain("确认 P1");
    expect(wrapper.text()).toContain("正常");
    expect(wrapper.text()).toContain("异常");
    expect(wrapper.text()).toContain("拒绝");
    expect(wrapper.findAll("button").filter((button) => button.text() !== "刷新").every((button) => button.attributes("disabled") !== undefined)).toBe(true);
  });

  it("renders delivery pickup as one business button and submits OK", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    const aggregate = useAggregateStore();
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
      waiting_confirmations: [
        {
          task_id: "task_3",
          step_index: 0,
          step_id: "pickup",
          task_type: "DELIVERY",
          point_id: "A",
          point_label: "A 取货点",
          label: "等待取货确认",
          robot_id: "mecanum"
        }
      ]
    };
    aggregate.applyState(state);
    vi.spyOn(apiClient, "confirmStep").mockResolvedValue({ ok: true, message: "确认已提交" });
    vi.spyOn(apiClient, "getState").mockResolvedValue({ ...state, waiting_confirmations: [] });
    vi.spyOn(apiClient, "getLogs").mockResolvedValue({ logs: [] });

    const wrapper = mount(ConfirmationView, { global: { plugins: [pinia] } });

    expect(wrapper.text()).toContain("配送任务_3");
    expect(wrapper.text()).toContain("已取货");
    expect(wrapper.text()).not.toContain("正常");
    expect(wrapper.text()).not.toContain("异常");
    expect(wrapper.text()).not.toContain("拒绝");

    await wrapper.find("button[title='已取货']").trigger("click");
    await flushPromises();

    expect(apiClient.confirmStep).toHaveBeenCalledWith(
      expect.objectContaining({ task_id: "task_3", result: "OK" })
    );
  });

  it("renders delivery dropoff as one unload button", () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    useAggregateStore().applyState({
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
      waiting_confirmations: [
        {
          task_id: "task_4",
          step_index: 1,
          step_id: "dropoff",
          task_type: "DELIVERY",
          point_id: "C",
          point_label: "C 配送点",
          label: "等待卸货确认",
          robot_id: "mecanum"
        }
      ]
    });

    const wrapper = mount(ConfirmationView, { global: { plugins: [pinia] } });

    expect(wrapper.text()).toContain("配送任务_4");
    expect(wrapper.text()).toContain("已卸货");
    expect(wrapper.findAll(".confirm-actions button")).toHaveLength(1);
  });
});
