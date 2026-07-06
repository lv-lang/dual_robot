import { beforeEach, describe, expect, it } from "vitest";
import { createPinia, setActivePinia } from "pinia";
import { useAggregateStore } from "./aggregate";
import { useConnectionStore } from "./connection";
import type { AggregateState } from "../types";

describe("connection and aggregate stores", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("disables commands until backend, dispatch, and websocket are available", () => {
    const connection = useConnectionStore();

    expect(connection.commandDisabled).toBe(true);
    expect(connection.commandDisabledReasons).toContain("Backend offline");

    connection.applyHealth({
      backend_online: true,
      dispatch_online: true,
      dispatch_degraded: false,
      updated_at: "2026-05-23T00:00:00Z",
      disabled_reasons: []
    });
    connection.setWebSocketOnline(true);

    expect(connection.commandDisabled).toBe(false);
  });

  it("replaces aggregate state and mirrors status into the connection store", () => {
    const aggregate = useAggregateStore();
    const connection = useConnectionStore();
    const state: AggregateState = {
      status: {
        backend_online: true,
        dispatch_online: false,
        dispatch_degraded: true,
        websocket_online: true,
        disabled_reasons: ["dispatch service timeout"],
        updated_at: "2026-05-23T00:00:00Z"
      },
      robots: [],
      tasks: [
        {
          task_id: "task-1",
          task_type: "DELIVERY",
          label: "Delivery",
          status: "EXECUTING",
          target_points: ["A", "C"]
        },
        {
          task_id: "task-2",
          task_type: "INSPECTION",
          label: "Inspection",
          status: "COMPLETED",
          target_points: ["P1"]
        }
      ],
      resource_locks: [],
      waiting_confirmations: []
    };

    aggregate.applyState(state);

    expect(aggregate.activeTasks).toHaveLength(1);
    expect(connection.backendOnline).toBe(true);
    expect(connection.commandDisabledReasons).toContain("Dispatch offline");
    expect(connection.commandDisabledReasons).toContain("dispatch service timeout");
  });
});
