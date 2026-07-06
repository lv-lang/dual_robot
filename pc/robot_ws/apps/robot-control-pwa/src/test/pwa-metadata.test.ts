import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";

describe("PWA metadata", () => {
  it("declares standalone display and icon assets without offline behavior", () => {
    const manifest = JSON.parse(
      readFileSync(resolve(process.cwd(), "public/manifest.webmanifest"), "utf-8")
    ) as { display: string; icons: unknown[]; start_url: string; short_name: string };

    expect(manifest.display).toBe("standalone");
    expect(manifest.start_url).toBe("/");
    expect(manifest.short_name).toBe("任务台");
    expect(manifest.icons.length).toBeGreaterThanOrEqual(2);
  });
});
