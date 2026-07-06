import { defineConfig, loadEnv } from "vite";
import vue from "@vitejs/plugin-vue";

function toWsTarget(target: string): string {
  if (target.startsWith("https://")) {
    return target.replace("https://", "wss://");
  }
  if (target.startsWith("http://")) {
    return target.replace("http://", "ws://");
  }
  return target;
}

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), "");
  const backendTarget = env.VITE_ROBOT_WEB_TARGET || "http://127.0.0.1:8000";

  return {
    plugins: [vue()],
    server: {
      host: "0.0.0.0",
      port: 5173,
      proxy: {
        "/api": {
          target: backendTarget,
          changeOrigin: true
        },
        "/ws": {
          target: toWsTarget(backendTarget),
          changeOrigin: true,
          ws: true
        }
      }
    },
    test: {
      environment: "jsdom",
      globals: true,
      setupFiles: ["src/test/setup.ts"]
    }
  };
});
