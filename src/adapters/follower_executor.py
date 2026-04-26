"""Follower执行器 - 只执行follower命令"""

import logging
from concurrent.futures import Future
from ..runtime.command_plan import FollowerAction, HoldAction
from .group_executor_pool import GroupExecutorPool

logger = logging.getLogger(__name__)


class FollowerExecutor:
    """Follower命令执行器"""

    def __init__(
        self,
        transport,
        *,
        group_executor_pool: GroupExecutorPool | None = None,
    ):
        self.transport = transport
        self._group_pool = group_executor_pool

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

    def _drone_radio_group(self, drone_id: int) -> int | None:
        radio_group_fn = getattr(self.transport, "radio_group", None)
        if radio_group_fn is None:
            return None
        try:
            return radio_group_fn(drone_id)
        except Exception:
            return None

    def _run_group_parallel(
        self,
        kind: str,
        grouped: dict[int | None, list],
        per_action: callable,
    ) -> dict:
        if not self._group_pool:
            raise RuntimeError("group_executor_pool 未设置")

        futures: list[tuple[int, Future]] = []
        unscheduled: list = []  # 未知 group 的 actions 走 inline 路径
        for group_id, group_actions in grouped.items():
            if group_id is None or group_id not in self._group_pool.group_ids:
                unscheduled.extend(group_actions)
                continue

            def _run_group(actions=group_actions):
                group_successes: list[int] = []
                group_failures: list[dict] = []
                for action in actions:
                    try:
                        per_action(action)
                        group_successes.append(action.drone_id)
                    except Exception as exc:
                        group_failures.append(
                            self._failure(action.drone_id, kind, exc)
                        )
                return group_successes, group_failures

            futures.append((group_id, self._group_pool.submit(group_id, _run_group)))

        successes: list[int] = []
        failures: list[dict] = []
        for _, fut in futures:
            ok, fail = fut.result()
            successes.extend(ok)
            failures.extend(fail)

        # 未知 group 的 actions：inline 执行（兼容旧路径）
        for action in unscheduled:
            try:
                per_action(action)
                successes.append(action.drone_id)
            except Exception as exc:
                failures.append(self._failure(action.drone_id, kind, exc))

        return self._group_action_result(kind, successes, failures)

    def _bucket_by_group(self, actions) -> dict[int | None, list]:
        buckets: dict[int | None, list] = {}
        for action in actions:
            group_id = self._drone_radio_group(action.drone_id)
            buckets.setdefault(group_id, []).append(action)
        return buckets

    def _run_drone_ids_parallel(
        self,
        kind: str,
        drone_ids: list[int],
        send_one,
    ) -> dict:
        if not self._group_pool:
            successes = []
            failures = []
            for drone_id in drone_ids:
                try:
                    send_one(drone_id)
                    successes.append(drone_id)
                except Exception as exc:
                    failures.append(self._failure(drone_id, kind, exc))
            return self._group_action_result(kind, successes, failures)

        grouped: dict[int | None, list[int]] = {}
        for drone_id in drone_ids:
            grouped.setdefault(self._drone_radio_group(drone_id), []).append(drone_id)

        futures: list[tuple[int, Future]] = []
        unscheduled: list[int] = []
        for group_id, group_drone_ids in grouped.items():
            if group_id is None or group_id not in self._group_pool.group_ids:
                unscheduled.extend(group_drone_ids)
                continue

            def _run_group(group_drone_ids=group_drone_ids):
                group_successes: list[int] = []
                group_failures: list[dict] = []
                for drone_id in group_drone_ids:
                    try:
                        send_one(drone_id)
                        group_successes.append(drone_id)
                    except Exception as exc:
                        group_failures.append(self._failure(drone_id, kind, exc))
                return group_successes, group_failures

            futures.append((group_id, self._group_pool.submit(group_id, _run_group)))

        successes: list[int] = []
        failures: list[dict] = []
        for _, fut in futures:
            ok, fail = fut.result()
            successes.extend(ok)
            failures.extend(fail)

        for drone_id in unscheduled:
            try:
                send_one(drone_id)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, kind, exc))

        return self._group_action_result(kind, successes, failures)

    def execute_velocity(self, actions: list[FollowerAction]):
        """执行 follower setpoint。

        每个 action 自描述发送模式：``kind="velocity"`` 走 ``cmd_velocity_world``，
        ``kind="full_state"`` 走 ``cmd_full_state``（onboard Mellinger 闭环）。
        历史上只有 velocity 分支，现在在同一个入口里分发，保持 caller 侧代码不变。
        """

        def _send_one(action: FollowerAction) -> None:
            if action.kind == "full_state":
                if action.position is None or action.acceleration is None:
                    raise ValueError(
                        f"full_state action 缺 position/acceleration: drone={action.drone_id}"
                    )
                pos = action.position
                vel = action.velocity
                acc = action.acceleration
                self.transport.cmd_full_state(
                    action.drone_id,
                    (float(pos[0]), float(pos[1]), float(pos[2])),
                    (float(vel[0]), float(vel[1]), float(vel[2])),
                    (float(acc[0]), float(acc[1]), float(acc[2])),
                )
            else:
                vel = action.velocity
                self.transport.cmd_velocity_world(
                    action.drone_id, float(vel[0]), float(vel[1]), float(vel[2])
                )

        if self._group_pool:
            return self._run_group_parallel(
                "velocity", self._bucket_by_group(actions), _send_one
            )

        successes = []
        failures = []
        for action in actions:
            try:
                _send_one(action)
                successes.append(action.drone_id)
            except Exception as exc:
                failures.append(self._failure(action.drone_id, "velocity", exc))
        return self._group_action_result("velocity", successes, failures)

    def execute_hold(self, actions: list[HoldAction]):
        """执行hold命令"""
        if self._group_pool:
            def _send(action: HoldAction) -> None:
                self.transport.cmd_velocity_world(action.drone_id, 0, 0, 0)

            return self._run_group_parallel(
                "hold", self._bucket_by_group(actions), _send
            )

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
        return self._run_drone_ids_parallel(
            "takeoff",
            drone_ids,
            lambda drone_id: self.transport.hl_takeoff(drone_id, height, duration),
        )

    def land(self, drone_ids: list[int], duration: float = 2.0):
        return self._run_drone_ids_parallel(
            "land",
            drone_ids,
            lambda drone_id: self.transport.hl_land(drone_id, 0.0, duration),
        )

    def stop_velocity_mode(self, drone_ids: list[int]):
        return self._run_drone_ids_parallel(
            "notify_stop",
            drone_ids,
            lambda drone_id: self.transport.notify_setpoint_stop(drone_id),
        )
