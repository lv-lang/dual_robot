import { describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import TaskSummaryList from "./TaskSummaryList.vue";

describe("TaskSummaryList", () => {
  it("uses business display names instead of raw task ids as the primary task name", () => {
    const wrapper = mount(TaskSummaryList, {
      props: {
        tasks: [
          {
            task_id: "task_7",
            task_type: "DELIVERY",
            label: "task_7",
            status: "EXECUTING",
            target_points: ["A", "C"]
          },
          {
            task_id: "task-008",
            task_type: "INSPECTION",
            display_name: "巡检任务_8",
            label: "task-008",
            status: "WAITING_CONFIRMATION",
            target_points: ["P1"]
          },
          {
            task_id: "task_9",
            task_type: "RECHECK",
            label: "task_9",
            status: "ASSIGNED",
            target_points: ["P2"]
          }
        ]
      }
    });

    const primaryNames = wrapper.findAll("strong").map((node) => node.text());
    expect(primaryNames).toEqual(["配送任务_7", "巡检任务_8", "复查任务_9"]);
    expect(wrapper.text()).not.toContain("task_7");
  });
});
