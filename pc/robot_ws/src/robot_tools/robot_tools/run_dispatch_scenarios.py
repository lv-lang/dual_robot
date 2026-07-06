import argparse
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MissionStep, Task, TaskState
from robot_interfaces.srv import (
    CancelTask,
    ConfirmTaskStep,
    CreateTask,
    EmergencyStop,
    GetDispatchState,
    PauseTask,
    ResumeTask,
)


class DispatchScenarioRunner(Node):
    def __init__(self, timeout_sec: float) -> None:
        super().__init__('run_dispatch_scenarios')
        self.timeout_sec = timeout_sec
        self.create_task_client = self.create_client(CreateTask, '/robot_dispatch/create_task')
        self.confirm_client = self.create_client(
            ConfirmTaskStep, '/robot_dispatch/confirm_task_step')
        self.cancel_client = self.create_client(CancelTask, '/robot_dispatch/cancel_task')
        self.pause_client = self.create_client(PauseTask, '/robot_dispatch/pause_task')
        self.resume_client = self.create_client(ResumeTask, '/robot_dispatch/resume_task')
        self.estop_client = self.create_client(EmergencyStop, '/robot_dispatch/emergency_stop')
        self.state_client = self.create_client(GetDispatchState, '/robot_dispatch/get_state')

    def run(self) -> bool:
        self._wait_for_services()
        checks = [
            self._delivery_scenario,
            self._inspection_recheck_scenario,
            self._robot_preference_scenario,
            self._pause_resume_cancel_scenario,
            self._estop_scenario,
        ]
        ok = True
        for check in checks:
            name = check.__name__.removeprefix('_').replace('_', ' ')
            try:
                check()
                self.get_logger().info(f'PASS {name}')
            except Exception as exc:  # noqa: BLE001
                ok = False
                self.get_logger().error(f'FAIL {name}: {exc}')
        return ok

    def _delivery_scenario(self) -> None:
        task_id = self._create_task(Task.TYPE_DELIVERY, ['PICKUP_A', 'DELIVERY_C'])
        self._confirm(task_id, ConfirmTaskStep.Request.RESULT_OK)
        self._confirm(task_id, ConfirmTaskStep.Request.RESULT_OK)
        self._expect_task_state(task_id, TaskState.SUCCEEDED)

    def _inspection_recheck_scenario(self) -> None:
        inspection_id = self._create_task(Task.TYPE_INSPECTION, ['P1', 'P2', 'P3'])
        derived = self._confirm(inspection_id, ConfirmTaskStep.Request.RESULT_ABNORMAL)
        if not derived:
            raise RuntimeError('inspection abnormal did not derive RECHECK task')
        self._confirm(derived, ConfirmTaskStep.Request.RESULT_OK)
        self._expect_task_state(derived, TaskState.SUCCEEDED)
        self._confirm(inspection_id, ConfirmTaskStep.Request.RESULT_OK)
        self._confirm(inspection_id, ConfirmTaskStep.Request.RESULT_OK)
        self._expect_task_state(inspection_id, TaskState.SUCCEEDED)

    def _robot_preference_scenario(self) -> None:
        task_id = self._create_task(
            Task.TYPE_DELIVERY,
            ['PICKUP_A', 'DELIVERY_C'],
            preferred_robot_id='robot2')
        task = self._find_task(task_id)
        if task.preferred_robot_id != 'robot2':
            raise RuntimeError(
                f'{task_id} preferred_robot_id {task.preferred_robot_id}, expected robot2')
        if task.assigned_robot_id != 'robot2':
            raise RuntimeError(
                f'{task_id} assigned_robot_id {task.assigned_robot_id}, expected robot2')
        self._cancel(task_id)
        self._expect_task_state(task_id, TaskState.CANCELED)

    def _pause_resume_cancel_scenario(self) -> None:
        task_id = self._create_task(Task.TYPE_DELIVERY, ['PICKUP_A', 'DELIVERY_C'])
        self._pause(task_id)
        self._expect_task_state(task_id, TaskState.PAUSED)
        self._resume(task_id)
        self._cancel(task_id)
        self._expect_task_state(task_id, TaskState.CANCELED)

    def _estop_scenario(self) -> None:
        task_id = self._create_task(Task.TYPE_DELIVERY, ['PICKUP_A', 'DELIVERY_C'])
        request = EmergencyStop.Request()
        request.active = True
        request.requester = 'scenario_runner'
        response = self._call(self.estop_client, request)
        if not response.accepted:
            raise RuntimeError(f'estop rejected: {response.message}')
        self._expect_task_state(task_id, TaskState.FAILED)

    def _create_task(
        self,
        task_type: int,
        point_ids: List[str],
        preferred_robot_id: str = 'auto',
    ) -> str:
        request = CreateTask.Request()
        request.task_type = task_type
        request.requester = 'scenario_runner'
        request.preferred_robot_id = preferred_robot_id
        for index, point_id in enumerate(point_ids):
            step = MissionStep()
            step.sequence = index
            step.step_type = MissionStep.STEP_NAVIGATE
            step.point_id = point_id
            request.steps.append(step)
        response = self._call(self.create_task_client, request)
        if not response.accepted:
            raise RuntimeError(f'create task rejected: {response.message}')
        return response.task_id

    def _confirm(self, task_id: str, result: int) -> str:
        request = ConfirmTaskStep.Request()
        request.task_id = task_id
        request.requester = 'scenario_runner'
        request.result = result
        response = self._call(self.confirm_client, request)
        if not response.accepted:
            raise RuntimeError(f'confirm rejected for {task_id}: {response.message}')
        return response.derived_task_id

    def _pause(self, task_id: str) -> None:
        request = PauseTask.Request()
        request.task_id = task_id
        request.requester = 'scenario_runner'
        response = self._call(self.pause_client, request)
        if not response.accepted:
            raise RuntimeError(f'pause rejected: {response.message}')

    def _resume(self, task_id: str) -> None:
        request = ResumeTask.Request()
        request.task_id = task_id
        request.requester = 'scenario_runner'
        response = self._call(self.resume_client, request)
        if not response.accepted:
            raise RuntimeError(f'resume rejected: {response.message}')

    def _cancel(self, task_id: str) -> None:
        request = CancelTask.Request()
        request.task_id = task_id
        request.requester = 'scenario_runner'
        response = self._call(self.cancel_client, request)
        if not response.accepted:
            raise RuntimeError(f'cancel rejected: {response.message}')

    def _expect_task_state(self, task_id: str, expected_state: int) -> None:
        task = self._find_task(task_id)
        if task.state.state != expected_state:
            raise RuntimeError(
                f'{task_id} state {task.state.state}, expected {expected_state}')

    def _find_task(self, task_id: str):
        state = self._dispatch_state()
        for task in state.tasks:
            if task.task_id == task_id:
                return task
        raise RuntimeError(f'{task_id} not found')

    def _dispatch_state(self):
        request = GetDispatchState.Request()
        request.requester = 'scenario_runner'
        return self._call(self.state_client, request)

    def _wait_for_services(self) -> None:
        for client in [
            self.create_task_client,
            self.confirm_client,
            self.cancel_client,
            self.pause_client,
            self.resume_client,
            self.estop_client,
            self.state_client,
        ]:
            if not client.wait_for_service(timeout_sec=self.timeout_sec):
                raise RuntimeError(f'service unavailable: {client.srv_name}')

    def _call(self, client, request):
        future = client.call_async(request)
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > self.timeout_sec:
                raise TimeoutError(f'service timeout: {client.srv_name}')
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result()


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run robot_dispatch service-level functional scenarios.')
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = DispatchScenarioRunner(args.timeout_sec)
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
