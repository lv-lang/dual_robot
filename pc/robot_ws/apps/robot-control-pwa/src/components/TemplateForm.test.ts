import { describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import TemplateForm from "./TemplateForm.vue";
import type { BusinessPoint } from "../types";

const businessPoints: BusinessPoint[] = [
  { point_id: "A", label: "A 取货点", point_type: "pickup" },
  { point_id: "C", label: "C 配送点", point_type: "delivery" },
  { point_id: "P1", label: "P1 巡检点", point_type: "inspection" },
  { point_id: "P2", label: "P2 巡检点", point_type: "inspection" },
  { point_id: "P3", label: "P3 巡检点", point_type: "inspection" }
];

const rvizBusinessPoints: BusinessPoint[] = [
  { point_id: "RVIZ_PICKUP_1", label: "rviz pickup", point_type: "pickup" },
  { point_id: "RVIZ_PICKUP_2", label: "rviz pickup", point_type: "pickup" },
  { point_id: "RVIZ_DELIVERY_1", label: "rviz delivery", point_type: "delivery" },
  { point_id: "RVIZ_INSPECTION_1", label: "rviz inspection", point_type: "inspection" },
  { point_id: "RVIZ_INSPECTION_2", label: "rviz inspection", point_type: "inspection" }
];

describe("TemplateForm", () => {
  it("defaults a new delivery template to pickup then delivery points", async () => {
    const wrapper = mount(TemplateForm, {
      props: {
        businessPoints
      }
    });

    await wrapper.find<HTMLInputElement>("#template-name").setValue("Robot2 delivery");
    await wrapper.find<HTMLSelectElement>("#template-robot").setValue("ackermann");
    await wrapper.find("form").trigger("submit");

    expect(wrapper.emitted("save")?.[0]).toEqual([
      {
        name: "Robot2 delivery",
        task_type: "DELIVERY",
        robot_preference: "ackermann",
        target_points: ["A", "C"],
        sort_order: 100
      }
    ]);
  });

  it("submits inspection templates with inspection points only", async () => {
    const wrapper = mount(TemplateForm, {
      props: {
        businessPoints
      }
    });

    await wrapper.find<HTMLInputElement>("#template-name").setValue("Robot1 inspection");
    await wrapper.find<HTMLSelectElement>("#template-type").setValue("INSPECTION");
    await wrapper.find<HTMLSelectElement>("#template-robot").setValue("mecanum");
    await wrapper.find<HTMLInputElement>("input[value='P1']").setValue(true);
    await wrapper.find<HTMLInputElement>("input[value='P2']").setValue(true);
    await wrapper.find("form").trigger("submit");

    expect(wrapper.emitted("save")?.[0]).toEqual([
      {
        name: "Robot1 inspection",
        task_type: "INSPECTION",
        robot_preference: "mecanum",
        target_points: ["P1", "P2"],
        sort_order: 100
      }
    ]);
  });

  it("renders point ids with labels so temporary RViz points are distinguishable", async () => {
    const wrapper = mount(TemplateForm, {
      props: {
        businessPoints: rvizBusinessPoints
      }
    });

    const pickupOptions = wrapper.findAll("#pickup-point option").map((option) => option.text());
    expect(pickupOptions).toEqual([
      "RVIZ_PICKUP_1 · rviz pickup",
      "RVIZ_PICKUP_2 · rviz pickup"
    ]);

    await wrapper.find<HTMLSelectElement>("#template-type").setValue("INSPECTION");
    const inspectionLabels = wrapper.findAll(".checkbox-tile span").map((label) => label.text());
    expect(inspectionLabels).toEqual([
      "RVIZ_INSPECTION_1 · rviz inspection",
      "RVIZ_INSPECTION_2 · rviz inspection"
    ]);
  });
});
