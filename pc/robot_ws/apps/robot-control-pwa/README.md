# Robot Control PWA

Vue 3 PWA for the robot task gateway.

## Development

Install dependencies:

```bash
npm install
```

Run with the Vite proxy pointed at `robot_web`:

```bash
VITE_ROBOT_WEB_TARGET=http://127.0.0.1:8000 npm run dev
```

The development server keeps frontend paths stable:

- `/` serves the PWA.
- `/api/...` proxies to `robot_web`.
- `/ws/status` proxies to `robot_web`.

When `robot_web` is not available, use the local mock client:

```bash
VITE_USE_MOCK_API=true npm run dev
```

## Validation

```bash
npm run test
npm run build
```

The production build emits `dist/` for same-origin serving by `robot_web`.
