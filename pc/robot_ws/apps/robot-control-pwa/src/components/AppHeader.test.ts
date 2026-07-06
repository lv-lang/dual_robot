import { beforeEach, describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import AppHeader from "./AppHeader.vue";
import { useConnectionStore } from "../stores/connection";
import type { AggregateStatus } from "../types";

const routerLinkStub = {
  template: "<a><slot /></a>"
};

const readyStatus: AggregateStatus = {
  backend_online: true,
  dispatch_online: true,
  dispatch_degraded: false,
  websocket_online: true,
  disabled_reasons: [],
  updated_at: "2026-05-24T00:00:00Z"
};

function mountHeader(status: AggregateStatus = readyStatus) {
  const pinia = createPinia();
  setActivePinia(pinia);
  useConnectionStore().applyAggregateStatus(status);

  return mount(AppHeader, {
    global: {
      plugins: [pinia],
      stubs: {
        RouterLink: routerLinkStub
      }
    }
  });
}

describe("AppHeader", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("renders the Industrial HMI navigation labels", () => {
    const wrapper = mountHeader();

    const labels = wrapper.find("nav[aria-label='Primary']").findAll("a").map((link) => link.text().trim());

    expect(labels).toEqual(["态势", "任务", "待确认", "事件日志", "系统诊断"]);
  });

  it("keeps global emergency stop visible and disabled while commands are disabled", () => {
    const wrapper = mountHeader({
      ...readyStatus,
      backend_online: false,
      disabled_reasons: ["global_estop_active"]
    });

    const emergencyStop = wrapper.find("button[title='全局急停']");

    expect(emergencyStop.exists()).toBe(true);
    expect(emergencyStop.text()).toContain("全局急停");
    expect(emergencyStop.attributes("disabled")).toBeDefined();
  });
});
