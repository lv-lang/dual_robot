import { describe, expect, it } from "vitest";
import { MockRobotControlApiClient } from "./mock";

describe("MockRobotControlApiClient", () => {
  it("provides runnable templates and task state without robot_web", async () => {
    const client = new MockRobotControlApiClient();
    const templates = await client.getTemplates();
    const result = await client.triggerTemplate(templates.templates[0].template_id);
    const state = await client.getState();

    expect(result.task_id).toMatch(/^task-/);
    expect(state.status.backend_online).toBe(true);
    expect(state.tasks.some((task) => task.task_id === result.task_id)).toBe(true);
  });

  it("supports user-template CRUD in mock mode", async () => {
    const client = new MockRobotControlApiClient();
    const created = await client.createTemplate({
      name: "Custom inspection",
      task_type: "INSPECTION",
      robot_preference: "ackermann",
      target_points: ["P1"],
      sort_order: 90
    });
    const updated = await client.updateTemplate(created.template_id, {
      name: "Custom inspection updated",
      task_type: "INSPECTION",
      robot_preference: "ackermann",
      target_points: ["P1", "P2"],
      sort_order: 95
    });
    await client.deleteTemplate(created.template_id);

    expect(created.readonly).toBe(false);
    expect(updated.target_points).toEqual(["P1", "P2"]);
    expect((await client.getTemplates()).templates.some((template) => template.template_id === created.template_id)).toBe(false);
  });

  it("keeps delivery and inspection target points separate when triggering opposite robot preferences", async () => {
    const client = new MockRobotControlApiClient();
    const delivery = await client.createTemplate({
      name: "Robot2 delivery",
      task_type: "DELIVERY",
      robot_preference: "ackermann",
      target_points: ["A", "C"],
      sort_order: 90
    });
    const inspection = await client.createTemplate({
      name: "Robot1 inspection",
      task_type: "INSPECTION",
      robot_preference: "mecanum",
      target_points: ["P1", "P2", "P3"],
      sort_order: 100
    });

    const deliveryResult = await client.triggerTemplate(delivery.template_id);
    const inspectionResult = await client.triggerTemplate(inspection.template_id);
    const state = await client.getState();
    const deliveryTask = state.tasks.find((task) => task.task_id === deliveryResult.task_id);
    const inspectionTask = state.tasks.find((task) => task.task_id === inspectionResult.task_id);

    expect(deliveryResult.display_name).toBe("配送任务_1");
    expect(inspectionResult.display_name).toBe("巡检任务_1");
    expect(deliveryTask?.task_type).toBe("DELIVERY");
    expect(deliveryTask?.display_name).toBe("配送任务_1");
    expect(deliveryTask?.assigned_robot_id).toBe("ackermann");
    expect(deliveryTask?.target_points).toEqual(["A", "C"]);
    expect(inspectionTask?.task_type).toBe("INSPECTION");
    expect(inspectionTask?.display_name).toBe("巡检任务_1");
    expect(inspectionTask?.assigned_robot_id).toBe("mecanum");
    expect(inspectionTask?.target_points).toEqual(["P1", "P2", "P3"]);
  });

  it("rejects templates whose point kinds do not match the task type", async () => {
    const client = new MockRobotControlApiClient();

    await expect(
      client.createTemplate({
        name: "Invalid delivery",
        task_type: "DELIVERY",
        robot_preference: "ackermann",
        target_points: ["P1", "P2"],
        sort_order: 90
      })
    ).rejects.toThrow("配送模板需要先选择取货点，再选择配送点");

    await expect(
      client.createTemplate({
        name: "Invalid inspection",
        task_type: "INSPECTION",
        robot_preference: "mecanum",
        target_points: ["A", "C"],
        sort_order: 100
      })
    ).rejects.toThrow("巡检模板只能选择巡检点");
  });

  it("simulates system control lifecycle and logs", async () => {
    const client = new MockRobotControlApiClient();

    const started = await client.startSystem();
    const logs = await client.getSystemLogs();
    const stopped = await client.stopSystem();

    expect(started.status.managed).toBe(true);
    expect(started.status.profile.command).toContain(
      "real_robot_control_plane.launch.py launch_rviz:=true"
    );
    expect(logs.launch_logs.some((line) => line.message.includes("robot_dispatch"))).toBe(true);
    expect(logs.operation_logs[0].event).toBe("system_start");
    expect(stopped.status.status).toBe("stopped");
  });
});
