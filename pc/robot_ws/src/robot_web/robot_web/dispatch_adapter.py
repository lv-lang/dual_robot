from dataclasses import dataclass, field


@dataclass
class DispatchResult:
    accepted: bool
    message: str = ""
    data: object = None
    reason: str = ""


@dataclass
class DispatchSnapshot:
    tasks: list = field(default_factory=list)
    robot_states: list = field(default_factory=list)
    resource_locks: list = field(default_factory=list)
    system_state: object = None
    message: str = ""


class DispatchClient:
    def is_online(self, timeout_sec=0.1):
        raise NotImplementedError

    def get_state(self):
        raise NotImplementedError

    def get_task_points(self):
        raise NotImplementedError

    def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
        raise NotImplementedError

    def confirm_task(self, task_id, result, requester, step_index=0, step_id="", point_id="", note=""):
        raise NotImplementedError

    def pause_task(self, task_id, requester, reason=""):
        raise NotImplementedError

    def resume_task(self, task_id, requester, reason=""):
        raise NotImplementedError

    def cancel_task(self, task_id, requester, reason=""):
        raise NotImplementedError

    def emergency_stop(self, requester, reason=""):
        raise NotImplementedError


class OfflineDispatchClient(DispatchClient):
    def __init__(self, message="robot_dispatch unavailable"):
        self.message = message

    def is_online(self, timeout_sec=0.1):
        return False

    def get_state(self):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def get_task_points(self):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def confirm_task(self, task_id, result, requester, step_index=0, step_id="", point_id="", note=""):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def pause_task(self, task_id, requester, reason=""):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def resume_task(self, task_id, requester, reason=""):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def cancel_task(self, task_id, requester, reason=""):
        return DispatchResult(False, self.message, reason="dispatch_offline")

    def emergency_stop(self, requester, reason=""):
        return DispatchResult(False, self.message, reason="dispatch_offline")

