import { describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import LogList from "./LogList.vue";

describe("LogList", () => {
  it("uses the task display name captured on the log entry", () => {
    const wrapper = mount(LogList, {
      props: {
        logs: [
          {
            log_id: "log-1",
            timestamp: "2026-05-24T03:00:00Z",
            level: "info",
            event: "template_triggered",
            message: "任务已创建",
            task_id: "task_2",
            task_display_name: "配送任务_1"
          }
        ]
      }
    });

    expect(wrapper.text()).toContain("配送任务_1");
    expect(wrapper.text()).not.toContain("任务_2");
  });

  it("falls back to the raw task id display when an old log has no captured display name", () => {
    const wrapper = mount(LogList, {
      props: {
        logs: [
          {
            log_id: "log-1",
            timestamp: "2026-05-24T03:00:00Z",
            level: "info",
            event: "template_triggered",
            message: "历史任务",
            task_id: "task_2"
          }
        ]
      }
    });

    expect(wrapper.text()).toContain("任务_2");
    expect(wrapper.text()).not.toContain("配送任务_1");
  });
});
