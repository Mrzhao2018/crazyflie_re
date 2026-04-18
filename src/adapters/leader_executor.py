"""Leader执行器 - 只执行leader命令"""

import logging
from concurrent.futures import Future
from ..runtime.command_plan import LeaderAction
from .group_executor_pool import GroupExecutorPool

logger = logging.getLogger(__name__)


class LeaderExecutor:
    """Leader命令执行器"""

    def __init__(
        self,
        transport,
        *,
        group_executor_pool: GroupExecutorPool | None = None,
    ):
        self.transport = transport
        self._group_pool = group_executor_pool

    @staticmethod
    def _group_action_result(action: LeaderAction, successes: list[int], failures: list[dict]) -> dict:
        return {
            "kind": action.kind,
            "drone_ids": list(action.drone_ids),
            "successes": successes,
            "failures": failures,
        }

    def _failure(self, drone_id: int, action: LeaderAction, exc: Exception) -> dict:
        return self.transport.classify_command_failure(
            drone_id=drone_id,
            command_kind=action.kind,
            exception=exc,
        )

    def _drone_radio_group(self, drone_id: int) -> int | None:
        radio_group_fn = getattr(self.transport, "radio_group", None)
        if radio_group_fn is None:
            return None
        try:
            return radio_group_fn(drone_id)
        except Exception:
            return None

    def execute(self, actions: list[LeaderAction]):
        """执行leader动作列表"""
        results = []
        for action in actions:
            if action.kind == "takeoff":
                results.append(self._execute_takeoff(action))
            elif action.kind == "batch_goto":
                results.append(self._execute_batch_goto(action))
            elif action.kind == "start_trajectory":
                results.append(self._execute_start_trajectory(action))
            elif action.kind == "land":
                results.append(self._execute_land(action))
        return results

    def _execute_takeoff(self, action: LeaderAction):
        """执行起飞"""
        height = action.payload.get("height", 0.5)
        duration = action.payload.get("duration", 2.0)
        successes = []
        failures = []
        for drone_id in action.drone_ids:
            try:
                self.transport.hl_takeoff(drone_id, height, duration)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, action, exc))
        return self._group_action_result(action, successes, failures)

    def _execute_batch_goto(self, action: LeaderAction):
        """批量go_to（共时更新），跨 radio_group 并行，组内按 drone_ids 顺序串行"""
        positions = action.payload["positions"]  # {drone_id: [x,y,z]}
        duration = action.payload.get("duration", 1.0)

        if self._group_pool:
            grouped: dict[int | None, list[int]] = {}
            for drone_id in action.drone_ids:
                grouped.setdefault(self._drone_radio_group(drone_id), []).append(drone_id)

            futures: list[Future] = []
            inline_drones: list[int] = []
            for group_id, drones in grouped.items():
                if group_id is None or group_id not in self._group_pool.group_ids:
                    inline_drones.extend(drones)
                    continue

                def _run(drones=drones):
                    ok, fail = [], []
                    for drone_id in drones:
                        pos = positions[drone_id]
                        try:
                            self.transport.hl_go_to(
                                drone_id, pos[0], pos[1], pos[2], duration
                            )
                            ok.append(drone_id)
                        except Exception as exc:
                            fail.append(self._failure(drone_id, action, exc))
                    return ok, fail

                futures.append(self._group_pool.submit(group_id, _run))

            successes = []
            failures = []
            for fut in futures:
                ok, fail = fut.result()
                successes.extend(ok)
                failures.extend(fail)
            for drone_id in inline_drones:
                pos = positions[drone_id]
                try:
                    self.transport.hl_go_to(drone_id, pos[0], pos[1], pos[2], duration)
                    successes.append(drone_id)
                except Exception as exc:
                    failures.append(self._failure(drone_id, action, exc))
            return self._group_action_result(action, successes, failures)

        successes = []
        failures = []
        for drone_id in action.drone_ids:
            pos = positions[drone_id]
            try:
                self.transport.hl_go_to(drone_id, pos[0], pos[1], pos[2], duration)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, action, exc))
        return self._group_action_result(action, successes, failures)

    def _execute_land(self, action: LeaderAction):
        """执行降落"""
        duration = action.payload.get("duration", 2.0)
        successes = []
        failures = []
        for drone_id in action.drone_ids:
            try:
                self.transport.hl_land(drone_id, 0.0, duration)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, action, exc))
        return self._group_action_result(action, successes, failures)

    def _execute_start_trajectory(self, action: LeaderAction):
        trajectory_id = action.payload.get("trajectory_id", 1)
        time_scale = action.payload.get("time_scale", 1.0)
        relative_position = action.payload.get("relative_position", False)
        relative_yaw = action.payload.get("relative_yaw", False)
        reversed_mode = action.payload.get("reversed", False)
        successes = []
        failures = []
        for drone_id in action.drone_ids:
            try:
                self.transport.hl_start_trajectory(
                    drone_id,
                    trajectory_id,
                    time_scale=time_scale,
                    relative_position=relative_position,
                    relative_yaw=relative_yaw,
                    reversed=reversed_mode,
                )
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, action, exc))
        return self._group_action_result(action, successes, failures)
