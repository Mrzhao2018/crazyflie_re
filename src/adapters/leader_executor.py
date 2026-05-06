"""Leader执行器 - 只执行leader命令"""

import logging
import time
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

    def _execute_grouped(
        self,
        action: LeaderAction,
        send_one,
    ):
        if not self._group_pool:
            successes = []
            failures = []
            for drone_id in action.drone_ids:
                try:
                    send_one(drone_id)
                    successes.append(drone_id)
                except Exception as exc:
                    failures.append(self._failure(drone_id, action, exc))
            return self._group_action_result(action, successes, failures)

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
                    try:
                        send_one(drone_id)
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
            try:
                send_one(drone_id)
                successes.append(drone_id)
            except Exception as exc:
                failures.append(self._failure(drone_id, action, exc))
        return self._group_action_result(action, successes, failures)

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
        return self._execute_grouped(
            action,
            lambda drone_id: self.transport.hl_takeoff(drone_id, height, duration),
        )

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
        return self._execute_grouped(
            action,
            lambda drone_id: self.transport.hl_land(drone_id, 0.0, duration),
        )

    def _execute_start_trajectory(self, action: LeaderAction):
        trajectory_id = action.payload.get("trajectory_id", 1)
        requested_time_scale = action.payload.get("time_scale", 1.0)
        # Temporary rollback to the last known-good real-flight path. Firmware HLC
        # start became non-moving when the configured 1.25 scale was forwarded.
        time_scale = 1.0
        relative_position = action.payload.get("relative_position", False)
        relative_yaw = action.payload.get("relative_yaw", False)
        reversed_mode = action.payload.get("reversed", False)

        send_timings: dict[int, dict[str, float]] = {}

        def _send_start(drone_id: int) -> None:
            started = time.monotonic()
            try:
                self.transport.hl_start_trajectory(
                    drone_id,
                    trajectory_id,
                    time_scale=time_scale,
                    relative_position=relative_position,
                    relative_yaw=relative_yaw,
                    reversed=reversed_mode,
                )
            finally:
                done = time.monotonic()
                send_timings[int(drone_id)] = {
                    "start_monotonic_s": started,
                    "done_monotonic_s": done,
                    "duration_ms": (done - started) * 1000.0,
                }

        result = self._execute_grouped(
            action,
            _send_start,
        )
        if send_timings:
            starts = [item["start_monotonic_s"] for item in send_timings.values()]
            dones = [item["done_monotonic_s"] for item in send_timings.values()]
            first_start = min(starts)
            result["send_timings"] = {
                str(drone_id): {
                    **timing,
                    "start_offset_ms": (timing["start_monotonic_s"] - first_start) * 1000.0,
                }
                for drone_id, timing in sorted(send_timings.items())
            }
            result["send_skew_ms"] = (max(dones) - first_start) * 1000.0
        result["parameters"] = {
            "trajectory_id": trajectory_id,
            "time_scale": time_scale,
            "requested_time_scale": requested_time_scale,
            "relative_position": relative_position,
            "relative_yaw": relative_yaw,
            "reversed": reversed_mode,
        }
        return result
