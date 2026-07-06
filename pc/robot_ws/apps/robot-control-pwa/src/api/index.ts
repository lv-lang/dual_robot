import { connectStatusSocket, realApiClient } from "./client";
import { connectMockStatusSocket, mockApiClient } from "./mock";
import type { ApiClient, StatusSocketHandlers, StatusSocketHandle } from "../types";

const useMockApi = import.meta.env.VITE_USE_MOCK_API === "true";

export const apiClient: ApiClient = useMockApi ? mockApiClient : realApiClient;

export function connectStatus(handlers: StatusSocketHandlers): StatusSocketHandle {
  return useMockApi ? connectMockStatusSocket(handlers) : connectStatusSocket(handlers);
}

export { ApiError, RobotControlApiClient, connectStatusSocket } from "./client";
export { MockRobotControlApiClient } from "./mock";
