from robot_web.dispatch_adapter import DispatchClient, DispatchResult, DispatchSnapshot
from robot_web.models import CONFIRMATION_TO_ID, TASK_TYPE_TO_ID
from robot_web.points import task_point_from_message


class RosDispatchClient(DispatchClient):
    def __init__(self, service_timeout_sec=1.0, node_name="robot_web_gateway"):
        import rclpy
        from robot_interfaces.srv import (
            CancelTask,
            ConfirmTaskStep,
            CreateTask,
            EmergencyStop,
            GetDispatchState,
            GetTaskPoints,
            PauseTask,
            ResumeTask,
        )

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = rclpy.create_node(node_name)
        self.service_timeout_sec = float(service_timeout_sec)
        self._services = {
            "cancel": CancelTask,
            "confirm": ConfirmTaskStep,
            "create": CreateTask,
            "estop": EmergencyStop,
            "points": GetTaskPoints,
            "resume": ResumeTask,
            "pause": PauseTask,
            "state": GetDispatchState,
        }
        self.clients = {
            "create": self.node.create_client(CreateTask, "/robot_dispatch/create_task"),
            "cancel": self.node.create_client(CancelTask, "/robot_dispatch/cancel_task"),
            "pause": self.node.create_client(PauseTask, "/robot_dispatch/pause_task"),
            "resume": self.node.create_client(ResumeTask, "/robot_dispatch/resume_task"),
            "confirm": self.node.create_client(ConfirmTaskStep, "/robot_dispatch/confirm_task_step"),
            "estop": self.node.create_client(EmergencyStop, "/robot_dispatch/emergency_stop"),
            "state": self.node.create_client(GetDispatchState, "/robot_dispatch/get_state"),
            "points": self.node.create_client(GetTaskPoints, "/robot_dispatch/get_task_points"),
        }

    def destroy(self):
        self.node.destroy_node()

    def is_online(self, timeout_sec=0.1):
        return self.clients["state"].wait_for_service(timeout_sec=float(timeout_sec))

    def _call(self, key, request):
        client = self.clients[key]
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            return DispatchResult(
                False,
                f"{client.srv_name} unavailable",
                reason="dispatch_offline",
            )
        future = client.call_async(request)
        self._rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=self.service_timeout_sec)
        if not future.done():
            return DispatchResult(
                False,
                f"{client.srv_name} timed out",
                reason="service_timeout",
            )
        try:
            response = future.result()
        except Exception as exc:
            return DispatchResult(False, str(exc), reason="service_error")
        return DispatchResult(
            bool(getattr(response, "accepted", True)),
            str(getattr(response, "message", "")),
            data=response,
            reason="" if bool(getattr(response, "accepted", True)) else "dispatch_rejected",
        )

    def get_state(self):
        request = self._services["state"].Request()
        request.requester = "robot_web"
        result = self._call("state", request)
        if not result.accepted:
            return result
        response = result.data
        snapshot = DispatchSnapshot(
            tasks=list(response.tasks),
            robot_states=list(response.robot_states),
            resource_locks=list(response.resource_locks),
            system_state=getattr(response, "system_state", None),
            message=response.message,
        )
        return DispatchResult(True, response.message, data=snapshot)

    def get_task_points(self):
        request = self._services["points"].Request()
        request.requester = "robot_web"
        result = self._call("points", request)
        if not result.accepted:
            return result
        points = {
            point.point_id: point
            for point in (task_point_from_message(item) for item in result.data.points)
            if point.point_id
        }
        return DispatchResult(True, result.data.message, data=points)

    def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
        from robot_interfaces.msg import MissionStep

        request = self._services["create"].Request()
        request.requester = requester
        request.task_type = TASK_TYPE_TO_ID[task_type]
        request.note = note
        request.preferred_robot_id = preferred_robot_id or "auto"
        for sequence, point_id in enumerate(point_ids):
            step = MissionStep()
            step.sequence = sequence
            step.step_type = MissionStep.STEP_NAVIGATE
            step.step_id = f"web_{sequence}_{point_id}"
            step.point_id = point_id
            step.requires_confirmation = True
            step.resource_id = point_id
            step.label = point_id
            request.steps.append(step)
        return self._call("create", request)

    def confirm_task(self, task_id, result, requester, step_index=0, step_id="", point_id="", note=""):
        request = self._services["confirm"].Request()
        request.task_id = task_id
        request.requester = requester
        request.step_index = int(step_index)
        request.step_id = step_id
        request.point_id = point_id
        request.result = CONFIRMATION_TO_ID[result]
        request.note = note
        return self._call("confirm", request)

    def pause_task(self, task_id, requester, reason=""):
        request = self._services["pause"].Request()
        request.task_id = task_id
        request.requester = requester
        request.reason = reason
        return self._call("pause", request)

    def resume_task(self, task_id, requester, reason=""):
        request = self._services["resume"].Request()
        request.task_id = task_id
        request.requester = requester
        request.reason = reason
        return self._call("resume", request)

    def cancel_task(self, task_id, requester, reason=""):
        request = self._services["cancel"].Request()
        request.task_id = task_id
        request.requester = requester
        request.reason = reason
        return self._call("cancel", request)

    def emergency_stop(self, requester, reason=""):
        request = self._services["estop"].Request()
        request.active = True
        request.requester = requester
        request.reason = reason
        return self._call("estop", request)
