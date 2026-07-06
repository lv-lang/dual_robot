import { beforeEach, describe, expect, it, vi } from "vitest";
import { mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import TaskControlGroup from "./TaskControlGroup.vue";
import { apiClient } from "../api";
import { useConnectionStore } from "../stores/connection";

describe("TaskControlGroup", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("keeps task actions disabled while global commands are disabled", async () => {
    const pinia = createPinia();
    setActivePinia(pinia);
    useConnectionStore().applyAggregateStatus({
      backend_online: false,
      dispatch_online: true,
      dispatch_degraded: false,
      websocket_online: true,
      disabled_reasons: [],
      updated_at: "2026-05-24T00:00:00Z"
    });
    const pauseTask = vi.spyOn(apiClient, "pauseTask").mockResolvedValue({ ok: true, message: "paused" });

    const wrapper = mount(TaskControlGroup, {
      props: {
        taskId: "task-1",
        taskName: "配送任务_1",
        taskStatus: "EXECUTING"
      },
      global: {
        plugins: [pinia]
      }
    });

    const pause = wrapper.find("button[title='暂停任务']");
    const cancel = wrapper.find("button[title='取消任务']");

    expect(pause.attributes("disabled")).toBeDefined();
    expect(cancel.attributes("disabled")).toBeDefined();

    await pause.trigger("click");

    expect(pauseTask).not.toHaveBeenCalled();
  });
});
