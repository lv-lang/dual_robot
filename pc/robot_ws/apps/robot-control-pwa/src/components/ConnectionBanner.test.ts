import { beforeEach, describe, expect, it } from "vitest";
import { mount } from "@vue/test-utils";
import { createPinia, setActivePinia } from "pinia";
import ConnectionBanner from "./ConnectionBanner.vue";
import { useConnectionStore } from "../stores/connection";
import type { AggregateStatus } from "../types";

const readyStatus: AggregateStatus = {
  backend_online: true,
  dispatch_online: true,
  dispatch_degraded: false,
  websocket_online: true,
  disabled_reasons: [],
  updated_at: "2026-05-24T00:00:00Z"
};

function mountBanner(status: AggregateStatus) {
  const pinia = createPinia();
  setActivePinia(pinia);
  useConnectionStore().applyAggregateStatus(status);

  return mount(ConnectionBanner, {
    global: {
      plugins: [pinia]
    }
  });
}

describe("ConnectionBanner", () => {
  beforeEach(() => {
    setActivePinia(createPinia());
  });

  it("shows global command readiness when all live links are healthy", () => {
    const wrapper = mountBanner(readyStatus);

    expect(wrapper.text()).toContain("可以操作");
    expect(wrapper.text()).toContain("视觉画面接入");
    expect(wrapper.text()).toContain("后端在线");
    expect(wrapper.text()).toContain("调度在线");
    expect(wrapper.text()).toContain("实时连接在线");
  });

  it("shows command-disabled reasons from backend and live-link status", () => {
    const wrapper = mountBanner({
      ...readyStatus,
      dispatch_online: false,
      websocket_online: false,
      disabled_reasons: ["global_estop_active"]
    });

    expect(wrapper.text()).toContain("操作已禁用");
    expect(wrapper.text()).toContain("调度服务离线");
    expect(wrapper.text()).toContain("实时连接断开");
    expect(wrapper.text()).toContain("全局急停中");
  });
});
