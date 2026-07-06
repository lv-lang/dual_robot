# robot_web

`robot_web` is the App backend gateway for the Vue3 PWA. It exposes `/api/*`
HTTP routes, `/ws/status`, adapts task commands to `robot_dispatch` services,
stores only user task templates and gateway logs in SQLite, and serves the PWA
`dist` directory when one is present.

Runtime Python dependencies that are not currently installed in the active
environment:

```bash
python3 -m pip install fastapi uvicorn httpx
```

Same-origin layout:

- `/` serves `apps/robot-control-pwa/dist/index.html` when the dist exists.
- `/api/health`, `/api/state`, `/api/templates`, `/api/logs`, and task control
  endpoints provide JSON API access.
- `/api/map` serves the current App situation-map image from
  `real_competition_map.yaml` by default. If the field-exported source map is
  named `lv_home`, import its content into `real_competition_map.*` and keep the
  runtime map argument on the logical competition-map name.
- `/ws/status` streams `state_update` and `log_update` messages.

Example:

```bash
ros2 run robot_web app_gateway --host 0.0.0.0 --port 8000
```
