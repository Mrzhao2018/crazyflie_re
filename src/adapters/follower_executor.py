"""Follower执行器 - 只执行follower命令"""

import logging
from ..runtime.command_plan import FollowerAction, HoldAction

logger = logging.getLogger(__name__)


class FollowerExecutor:
    """Follower命令执行器"""

    def __init__(self, transport):
        self.transport = transport

    @staticmethod
    def _group_action_result(kind: str, successes: list[int], failures: list[dict]) -> dict:
        return {
            "kind": kind,
            "successes": successes,
            "failures": failures,
        }

    def _failure(self, drone_id: int, command_kind: str, exc: Exception) -> dict:
        return self.transport.classify_command_failure(
            drone_id=drone_id,
            command_kind=command_kind,
            exception=exc,
        )

    def execute_velocity(self, actions: list[FollowerAction]):
        """执行速度命令"""
        successes = []
        failures = []
        for action in actions:
            vel = action.velocity
            try:
                self.transport.cmd_velocity_world(action.drone_id, vel[0], vel[1], vel[2])
                successes.append(action.drone_id)
            except Exception as exc:
                failures.append(self._failure(action.drone_id, "velocity", exc))
        return self._group_action_result("velocity", successes, failures)

    def execute_hold(self, actions: list[HoldAction]):
        """执行hold命令"""
        successes = []
        failures = []
        for action in actions:
            try:
                self.transport.cmd_velocity_world(action.drone_id, 0, 0, 0)
                successes.append(action.drone_id)
            except Exception as exc:
                failures.append(self._failure(action.drone_id, "hold", exc))
        return self._group_action_result("hold", successes, failures)

    def takeoff(self, drone_ids: list[int], height: float = 0.5, duration: float = 2.0):
        successes = []
        failures = []
        for drone_id in drone_ids:
            try:
                self.transport.hl_takeoff(drone_id, height, duration)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, "takeoff", exc))
        return self._group_action_result("takeoff", successes, failures)

    def land(self, drone_ids: list[int], duration: float = 2.0):
        successes = []
        failures = []
        for drone_id in drone_ids:
            try:
                self.transport.hl_land(drone_id, 0.0, duration)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, "land", exc))
        return self._group_action_result("land", successes, failures)

    def stop_velocity_mode(self, drone_ids: list[int]):
        successes = []
        failures = []
        for drone_id in drone_ids:
            try:
                self.transport.notify_setpoint_stop(drone_id)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, "notify_stop", exc))
        return self._group_action_result("notify_stop", successes, failures)
