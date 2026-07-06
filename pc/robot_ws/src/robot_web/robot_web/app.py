import asyncio
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

from robot_web.exceptions import GatewayError
from robot_web.mapdata import load_map_payload
from robot_web.cameras import load_cameras


class WebSocketManager:
    def __init__(self):
        self._connections = set()

    async def connect(self, websocket):
        await websocket.accept()
        self._connections.add(websocket)

    def disconnect(self, websocket):
        self._connections.discard(websocket)

    async def broadcast(self, message):
        stale = []
        for websocket in list(self._connections):
            try:
                await websocket.send_json(message)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.disconnect(websocket)


def find_frontend_dist(explicit_path=None):
    if explicit_path:
        path = Path(explicit_path).expanduser()
        if path.exists():
            return path

    env_path = os.environ.get("ROBOT_WEB_FRONTEND_DIST")
    if env_path:
        path = Path(env_path).expanduser()
        if path.exists():
            return path

    candidates = [
        Path.cwd() / "apps" / "robot-control-pwa" / "dist",
        Path("/home/robot/robot_ws/apps/robot-control-pwa/dist"),
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


async def _json_body(request):
    try:
        return await request.json()
    except Exception:
        return {}


def create_app(gateway, frontend_dist=None, poll_interval_sec=1.0, map_yaml=None, cameras_file=None):
    app = FastAPI(title="robot_web", version="0.0.0")
    manager = WebSocketManager()
    dist_path = Path(frontend_dist).expanduser() if frontend_dist else None
    if dist_path is not None and not dist_path.exists():
        dist_path = None

    @app.exception_handler(GatewayError)
    async def gateway_error_handler(_, exc):
        return JSONResponse(status_code=exc.status_code, content=exc.to_dict())

    async def run_gateway(method, *args, broadcast_state=False, **kwargs):
        result = await asyncio.to_thread(method, *args, **kwargs)
        log = result.get("log") if isinstance(result, dict) else None
        if log:
            await manager.broadcast({"type": "log_update", "log": log})
        if broadcast_state:
            state = await asyncio.to_thread(gateway.state)
            await manager.broadcast({"type": "state_update", "state": state})
        return result

    async def run_system_action(method, payload):
        result = method(payload)
        log = result.get("log") if isinstance(result, dict) else None
        if log:
            await manager.broadcast({"type": "log_update", "log": log})
        if isinstance(result, dict) and "state" in result:
            await manager.broadcast({"type": "state_update", "state": result["state"]})
        return result

    @app.get("/api/health")
    async def health():
        return await asyncio.to_thread(gateway.health)

    @app.get("/api/state")
    async def state():
        return await asyncio.to_thread(gateway.state)

    @app.get("/api/task-points")
    async def task_points():
        return await asyncio.to_thread(gateway.list_task_points)

    @app.get("/api/map")
    async def map_data():
        return await asyncio.to_thread(load_map_payload, map_yaml)

    @app.get("/api/cameras")
    async def cameras():
        return await asyncio.to_thread(load_cameras, cameras_file)

    @app.get("/api/templates")
    async def list_templates():
        return await asyncio.to_thread(gateway.list_template_catalog)

    @app.post("/api/templates")
    async def create_template(request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.create_template, payload)

    @app.put("/api/templates/{template_id}")
    async def update_template(template_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.update_template, template_id, payload)

    @app.delete("/api/templates/{template_id}")
    async def delete_template(template_id: str):
        return await run_gateway(gateway.delete_template, template_id)

    @app.post("/api/templates/reorder")
    async def reorder_templates(request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.reorder_templates, payload)

    @app.post("/api/templates/{template_id}/trigger")
    async def trigger_template(template_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(
            gateway.trigger_template,
            template_id,
            payload,
            broadcast_state=True,
        )

    @app.post("/api/tasks/{task_id}/confirm")
    async def confirm_task(task_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(
            gateway.confirm_task,
            task_id,
            payload,
            broadcast_state=True,
        )

    @app.post("/api/confirmations")
    async def confirm_task_from_payload(request: Request):
        payload = await _json_body(request)
        task_id = str(payload.get("task_id", ""))
        if not task_id:
            raise GatewayError(400, "invalid_confirmation", "task_id is required")
        return await run_gateway(
            gateway.confirm_task,
            task_id,
            payload,
            broadcast_state=True,
        )

    @app.post("/api/tasks/{task_id}/pause")
    async def pause_task(task_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.pause_task, task_id, payload, broadcast_state=True)

    @app.post("/api/tasks/{task_id}/resume")
    async def resume_task(task_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.resume_task, task_id, payload, broadcast_state=True)

    @app.post("/api/tasks/{task_id}/cancel")
    async def cancel_task(task_id: str, request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.cancel_task, task_id, payload, broadcast_state=True)

    @app.post("/api/emergency-stop")
    async def emergency_stop(request: Request):
        payload = await _json_body(request)
        return await run_gateway(gateway.emergency_stop, payload, broadcast_state=True)

    @app.get("/api/logs")
    async def logs(limit: int = 100):
        return {"logs": await asyncio.to_thread(gateway.list_logs, limit)}

    @app.post("/api/demo-events/{key}")
    async def demo_event(key: str, request: Request):
        payload = await _json_body(request)
        result = gateway.trigger_demo_event(key, payload)
        log = result.get("log") if isinstance(result, dict) else None
        if log:
            await manager.broadcast({"type": "log_update", "log": log})
        elif isinstance(result, dict) and "logs" in result:
            await manager.broadcast({"type": "log_update", "logs": result.get("logs", [])})
        if isinstance(result, dict) and result.get("warning_clear"):
            await manager.broadcast({"type": "demo_warning_clear", "warning": result["warning_clear"]})
        if isinstance(result, dict) and "state" in result:
            await manager.broadcast({"type": "state_update", "state": result["state"]})
        return result

    @app.get("/api/system/status")
    async def system_status():
        return await asyncio.to_thread(gateway.system_status)

    @app.post("/api/system/start")
    async def system_start(request: Request):
        payload = await _json_body(request)
        return await run_system_action(gateway.system_start, payload)

    @app.post("/api/system/stop")
    async def system_stop(request: Request):
        payload = await _json_body(request)
        return await run_system_action(gateway.system_stop, payload)

    @app.post("/api/system/restart")
    async def system_restart(request: Request):
        payload = await _json_body(request)
        return await run_system_action(gateway.system_restart, payload)

    @app.get("/api/system/logs")
    async def system_logs(limit: int = 120):
        return await asyncio.to_thread(gateway.system_logs, limit)

    @app.websocket("/ws/status")
    async def websocket_status(websocket: WebSocket):
        await manager.connect(websocket)
        try:
            await websocket.send_json({
                "type": "state_update",
                "state": await asyncio.to_thread(gateway.state),
            })
            await websocket.send_json({
                "type": "log_update",
                "logs": await asyncio.to_thread(gateway.list_logs, 100),
            })
            while True:
                await asyncio.sleep(float(poll_interval_sec))
                await websocket.send_json({
                    "type": "state_update",
                    "state": await asyncio.to_thread(gateway.state),
                })
        except WebSocketDisconnect:
            manager.disconnect(websocket)
        except Exception:
            manager.disconnect(websocket)

    @app.get("/{full_path:path}", include_in_schema=False)
    async def frontend(full_path: str):
        if full_path.startswith("api/") or full_path.startswith("ws/"):
            raise HTTPException(status_code=404, detail="not found")
        if dist_path is None:
            return JSONResponse({
                "ok": True,
                "frontend_dist_present": False,
                "message": "Vue3 PWA dist not found; build apps/robot-control-pwa/dist for same-origin serving.",
            })

        requested = (dist_path / full_path).resolve()
        if dist_path.resolve() in requested.parents and requested.is_file():
            return FileResponse(requested)
        index = dist_path / "index.html"
        if index.exists():
            return FileResponse(index)
        return JSONResponse({
            "ok": True,
            "frontend_dist_present": False,
            "message": "frontend dist exists but index.html is missing",
        })

    return app
