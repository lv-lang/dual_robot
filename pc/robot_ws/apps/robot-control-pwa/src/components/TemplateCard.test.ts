import { describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import TemplateCard from "./TemplateCard.vue";
import type { TaskTemplate } from "../types";

const baseTemplate: TaskTemplate = {
  template_id: "template-1",
  name: "配送 A 到 C",
  task_type: "DELIVERY",
  robot_preference: "mecanum",
  target_points: ["A", "C"],
  readonly: true,
  sort_order: 10
};

describe("TemplateCard", () => {
  it("keeps built-in templates read-only in the UI", () => {
    const wrapper = mount(TemplateCard, {
      props: {
        template: baseTemplate
      }
    });

    expect(wrapper.text()).toContain("内置");
    expect(wrapper.text()).not.toContain("编辑");
    expect(wrapper.text()).not.toContain("删除");
  });

  it("shows edit and delete controls for user templates", () => {
    const wrapper = mount(TemplateCard, {
      props: {
        template: { ...baseTemplate, readonly: false }
      }
    });

    expect(wrapper.text()).toContain("编辑");
    expect(wrapper.text()).toContain("删除");
  });

  it("disables triggering unavailable templates and shows missing point ids", () => {
    const wrapper = mount(TemplateCard, {
      props: {
        template: {
          ...baseTemplate,
          readonly: false,
          available: false,
          unavailable_reason: "missing_task_points",
          missing_point_ids: ["RVIZ_PICKUP_1"]
        }
      }
    });

    expect(wrapper.text()).toContain("不可用");
    expect(wrapper.text()).toContain("RVIZ_PICKUP_1");
    expect(wrapper.find("button[title='触发模板']").attributes("disabled")).toBeDefined();
    expect(wrapper.text()).toContain("编辑");
    expect(wrapper.text()).toContain("删除");
  });
});
