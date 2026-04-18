"""Follower failure / watchdog policy.

Owns the runtime watchdog state (stale followers, executor-failure streaks,
HOLD entry timestamps) and their telemetry side-effects. Extracted from
``RealMissionApp`` so the main app class can focus on orchestration.
"""

from __future__ import annotations

import logging
import time

from ..app.mission_errors import MissionErrors
from .mission_fsm import MissionState


logger = logging.getLogger(__name__)

VELOCITY_STREAM_WATCHDOG_FACTOR = 3.0
EXECUTOR_GROUP_FAILURE_STREAK_THRESHOLD = 2


class FailurePolicy:
    """Holds the watchdog / degrade / hold-mode state machine for ``RealMissionApp``."""

    def __init__(self, app):
        self.app = app
        self.watchdog_degraded_followers: set[int] = set()
        self.follower_group_failure_streaks: dict[int, int] = {}
        self.hold_entered_at: float | None = None

    # ---- helpers ----------------------------------------------------------

    def _follower_ids_for_groups(self, group_ids: set[int]) -> list[int]:
        fleet = self.app.fleet
        if fleet is None:
            return []
        return sorted(
            drone_id
            for drone_id in fleet.follower_ids()
            if fleet.get_radio_group(drone_id) in group_ids
        )

    def _group_failures(self, failures: list[dict]) -> dict[int, list[dict]]:
        fleet = self.app.fleet
        grouped: dict[int, list[dict]] = {}
        if fleet is None:
            return grouped
        for failure in failures:
            group_id = failure.get("radio_group")
            drone_id = failure.get("drone_id")
            if group_id is None and drone_id is not None:
                group_id = fleet.get_radio_group(int(drone_id))
            if group_id is None:
                continue
            grouped.setdefault(int(group_id), []).append(failure)
        return grouped

    # ---- velocity stream watchdog ----------------------------------------

    def check_velocity_stream_watchdog(
        self, snapshot_t_meas: float
    ) -> list[dict]:
        app = self.app
        transport = app.comp.get("transport")
        telemetry = app.telemetry
        fleet = app.fleet
        config = app.comp.get("config")
        if transport is None or telemetry is None or fleet is None or config is None:
            return []

        follower_interval = 1.0 / config.comm.follower_tx_freq
        watchdog_timeout = follower_interval * VELOCITY_STREAM_WATCHDOG_FACTOR
        stale_followers: list[dict] = []
        for drone_id in fleet.follower_ids():
            last_tx_time = transport.last_velocity_command_time(drone_id)
            if last_tx_time is None:
                continue
            command_age = time.time() - last_tx_time
            if command_age > watchdog_timeout:
                stale_followers.append(
                    {
                        "drone_id": drone_id,
                        "command_age": command_age,
                        "watchdog_timeout": watchdog_timeout,
                        "snapshot_t_meas": snapshot_t_meas,
                    }
                )

        if stale_followers:
            action = config.safety.velocity_stream_watchdog_action
            stale_group_summary = app.telemetry_reporter.radio_group_item_summary(
                stale_followers,
                item_key="stale_followers",
            )
            telemetry.record_event(
                "velocity_stream_watchdog",
                ok=False,
                category=MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.category,
                code=MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.code,
                stage=MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.stage,
                stale_followers=stale_followers,
                radio_groups=stale_group_summary,
                follower_tx_freq=config.comm.follower_tx_freq,
                action=action,
            )

            stale_ids = sorted(
                int(item["drone_id"])
                for item in stale_followers
                if "drone_id" in item
            )
            if action == "hold":
                self.enter_hold_mode(reason="watchdog", follower_ids=stale_ids)
            elif action == "degrade":
                self.apply_watchdog_degrade(stale_followers)

        return stale_followers

    def apply_watchdog_degrade(self, stale_followers: list[dict]) -> None:
        stale_ids = sorted(
            int(item["drone_id"])
            for item in stale_followers
            if "drone_id" in item
        )
        if not stale_ids:
            return

        newly_degraded = [
            drone_id
            for drone_id in stale_ids
            if drone_id not in self.watchdog_degraded_followers
        ]
        self.watchdog_degraded_followers.update(stale_ids)

        self.app.telemetry.record_event(
            "watchdog_degrade",
            ok=True,
            category=MissionErrors.Runtime.WATCHDOG_DEGRADE.category,
            code=MissionErrors.Runtime.WATCHDOG_DEGRADE.code,
            stage=MissionErrors.Runtime.WATCHDOG_DEGRADE.stage,
            follower_ids=stale_ids,
            radio_groups=self.app.telemetry_reporter.radio_group_summary(stale_ids),
            newly_degraded=newly_degraded,
        )

    def apply_fast_gate_group_degrade(self, group_ids: list[int]) -> list[int]:
        """把 fast_gate_decision 报出的掉线 group 下的 follower 降级为 parked hold。

        返回实际被新增到 watchdog_degraded_followers 的 drone 列表（可空）。
        整体复用 watchdog_degrade 的 telemetry 通路，下游 replay / compare-runs
        的 watchdog_summary 会直接把它算进 degrade 次数。
        """

        if not group_ids:
            return []
        follower_ids = self._follower_ids_for_groups(set(group_ids))
        if not follower_ids:
            return []

        stale_followers = [
            {
                "drone_id": drone_id,
                "source": "fast_gate_group_degrade",
                "radio_group": self.app.fleet.get_radio_group(drone_id),
            }
            for drone_id in follower_ids
        ]
        self.apply_watchdog_degrade(stale_followers)
        return follower_ids

    def attempt_reconnect(self, drone_ids: list[int]) -> bool:
        """对指定 drone 尝试有限次数重连。全部成功时返回 True。

        读取 ``config.comm.reconnect_*`` 作为 attempts/backoff/timeout 参数。
        每次尝试都会记录 ``RUNTIME_LINK_RECONNECT_*`` 事件，供 replay / 分析
        定位链路抖动与恢复。
        """

        app = self.app
        config = app.comp.get("config")
        link_manager = app.comp.get("link_manager")
        telemetry = app.telemetry
        if config is None or link_manager is None or telemetry is None:
            return False
        if not drone_ids:
            return True

        attempts = int(getattr(config.comm, "reconnect_attempts", 0))
        backoff_s = float(getattr(config.comm, "reconnect_backoff_s", 0.5))
        timeout_s = float(getattr(config.comm, "reconnect_timeout_s", 5.0))
        if attempts <= 0:
            return False

        all_ok = True
        for drone_id in drone_ids:
            attempt_def = MissionErrors.Runtime.LINK_RECONNECT_ATTEMPT
            telemetry.record_event(
                "link_reconnect_attempt",
                ok=True,
                category=attempt_def.category,
                code=attempt_def.code,
                stage=attempt_def.stage,
                drone_id=drone_id,
                attempts=attempts,
            )

            result = link_manager.reconnect(
                drone_id,
                attempts=attempts,
                backoff_s=backoff_s,
                timeout_s=timeout_s,
            )

            if result.get("ok"):
                ok_def = MissionErrors.Runtime.LINK_RECONNECT_OK
                telemetry.record_event(
                    "link_reconnect_ok",
                    ok=True,
                    category=ok_def.category,
                    code=ok_def.code,
                    stage=ok_def.stage,
                    drone_id=drone_id,
                    attempt_count=result.get("attempt_count"),
                    radio_group=result.get("radio_group"),
                )
            else:
                fail_def = MissionErrors.Runtime.LINK_RECONNECT_FAILED
                telemetry.record_event(
                    "link_reconnect_failed",
                    ok=False,
                    category=fail_def.category,
                    code=fail_def.code,
                    stage=fail_def.stage,
                    drone_id=drone_id,
                    attempt_count=result.get("attempt_count"),
                    error=result.get("error"),
                    radio_group=result.get("radio_group"),
                )
                all_ok = False

        return all_ok

    def clear_watchdog_degrade(
        self, *, active_commands: dict[int, object] | None = None
    ) -> None:
        if not self.watchdog_degraded_followers:
            return

        active_ids = set(active_commands.keys()) if active_commands else set()
        recovered = sorted(self.watchdog_degraded_followers.intersection(active_ids))
        if not recovered:
            return

        self.watchdog_degraded_followers.difference_update(recovered)
        self.app.telemetry.record_event(
            "watchdog_degrade_recovered",
            ok=True,
            category=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.category,
            code=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.code,
            stage=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.stage,
            follower_ids=recovered,
            radio_groups=self.app.telemetry_reporter.radio_group_summary(recovered),
            remaining=sorted(self.watchdog_degraded_followers),
        )

    # ---- executor failure policy -----------------------------------------

    def apply_follower_failure_policy(
        self, follower_velocity_result: dict
    ) -> None:
        fleet = self.app.fleet
        telemetry = self.app.telemetry
        if fleet is None or telemetry is None:
            return

        failures_value = follower_velocity_result.get("failures", [])
        successes_value = follower_velocity_result.get("successes", [])
        failures = (
            [failure for failure in failures_value if isinstance(failure, dict)]
            if isinstance(failures_value, list)
            else []
        )
        successes = (
            [int(drone_id) for drone_id in successes_value if isinstance(drone_id, int)]
            if isinstance(successes_value, list)
            else []
        )

        failure_groups = self._group_failures(failures)
        success_group_ids = {
            fleet.get_radio_group(drone_id) for drone_id in successes
        }

        for group_id in sorted(success_group_ids.difference(failure_groups.keys())):
            self.follower_group_failure_streaks[group_id] = 0

        if not failure_groups:
            return

        triggered_groups: set[int] = set()
        triggered_details = []
        for group_id, group_failures in failure_groups.items():
            next_streak = self.follower_group_failure_streaks.get(group_id, 0) + 1
            self.follower_group_failure_streaks[group_id] = next_streak
            non_retryable = any(
                failure.get("retryable") is False for failure in group_failures
            )
            if (
                non_retryable
                or next_streak >= EXECUTOR_GROUP_FAILURE_STREAK_THRESHOLD
            ):
                triggered_groups.add(group_id)
                triggered_details.append(
                    {
                        "group_id": group_id,
                        "streak": next_streak,
                        "non_retryable": non_retryable,
                        "failure_count": len(group_failures),
                        "failure_categories": sorted(
                            {
                                str(failure.get("failure_category"))
                                for failure in group_failures
                                if failure.get("failure_category") is not None
                            }
                        ),
                        "failures": group_failures,
                    }
                )

        if not triggered_groups:
            return

        active_group_ids = set(failure_groups.keys()).union(success_group_ids)
        hold_triggered = bool(active_group_ids) and active_group_ids.issubset(
            triggered_groups
        )
        follower_ids = self._follower_ids_for_groups(triggered_groups)

        group_failure_streaks = {
            group_id: self.follower_group_failure_streaks.get(group_id, 0)
            for group_id in sorted(
                set(self.follower_group_failure_streaks).union(active_group_ids)
            )
        }

        if hold_triggered:
            definition = MissionErrors.Runtime.EXECUTOR_GROUP_HOLD
            telemetry.record_event(
                "executor_group_hold",
                ok=True,
                category=definition.category,
                code=definition.code,
                stage=definition.stage,
                action="hold",
                streak_threshold=EXECUTOR_GROUP_FAILURE_STREAK_THRESHOLD,
                triggered_groups=triggered_details,
                active_group_ids=sorted(active_group_ids),
                follower_ids=follower_ids,
                radio_groups=self.app.telemetry_reporter.radio_group_summary(
                    follower_ids
                ),
                group_failure_streaks=group_failure_streaks,
            )
            self.enter_hold_mode(reason="executor_failure")
            return

        newly_degraded = [
            drone_id
            for drone_id in follower_ids
            if drone_id not in self.watchdog_degraded_followers
        ]
        self.watchdog_degraded_followers.update(follower_ids)
        definition = MissionErrors.Runtime.EXECUTOR_GROUP_DEGRADE
        telemetry.record_event(
            "executor_group_degrade",
            ok=True,
            category=definition.category,
            code=definition.code,
            stage=definition.stage,
            action="degrade",
            streak_threshold=EXECUTOR_GROUP_FAILURE_STREAK_THRESHOLD,
            triggered_groups=triggered_details,
            active_group_ids=sorted(active_group_ids),
            follower_ids=follower_ids,
            newly_degraded=newly_degraded,
            radio_groups=self.app.telemetry_reporter.radio_group_summary(follower_ids),
            group_failure_streaks=group_failure_streaks,
        )

    # ---- hold-mode machinery ---------------------------------------------

    def enter_hold_mode(
        self,
        *,
        reason: str = "safety",
        follower_ids: list[int] | None = None,
    ) -> None:
        app = self.app
        if app.fsm.state() != MissionState.HOLD:
            app._safe_transition(MissionState.HOLD)
        if self.hold_entered_at is None:
            self.hold_entered_at = time.time()
            hold_definition = (
                MissionErrors.Runtime.WATCHDOG_HOLD
                if reason == "watchdog"
                else (
                    MissionErrors.Runtime.EXECUTOR_GROUP_HOLD
                    if reason == "executor_failure"
                    else None
                )
            )
            app.telemetry.record_event(
                "hold_entered",
                ok=True,
                reason=reason,
                category=(
                    hold_definition.category if hold_definition is not None else None
                ),
                code=(
                    hold_definition.code if hold_definition is not None else None
                ),
                stage=(
                    hold_definition.stage if hold_definition is not None else None
                ),
            )
        if app._trajectory_started:
            app._set_trajectory_state("paused")

        from .command_plan import HoldAction

        target_ids = follower_ids or app.fleet.follower_ids()
        hold_actions = [HoldAction(drone_id=fid) for fid in target_ids]
        hold_result = app.comp["follower_executor"].execute_hold(hold_actions)
        app.telemetry_reporter.record_executor_summary(
            "follower_hold_execution", [hold_result]
        )

    def check_hold_timeout(self, mission_elapsed: float) -> bool:
        if self.hold_entered_at is None:
            return False

        hold_duration = time.time() - self.hold_entered_at
        timeout = self.app.comp["config"].safety.hold_auto_land_timeout
        if hold_duration < timeout:
            return False

        self.app.telemetry.record_event(
            "hold_timeout_land",
            ok=True,
            hold_duration=hold_duration,
            timeout=timeout,
            mission_elapsed=mission_elapsed,
        )
        self.app._orderly_land(
            reason_event="hold_timeout_land",
            safety_action="HOLD_TIMEOUT",
            safety_reasons=["hold_timeout"],
            safety_reason_codes=["HOLD_TIMEOUT"],
            scheduler_reason="hold_timeout",
            scheduler_diagnostics={
                "hold_timeout": True,
                "hold_duration": hold_duration,
            },
            trajectory_terminal_reason="hold_timeout",
        )
        return True

    def clear_hold_tracking(self) -> None:
        self.hold_entered_at = None

    # ---- command filtering utility ---------------------------------------

    @staticmethod
    def split_degraded_commands(
        commands, degraded_follower_ids: set[int]
    ) -> tuple[dict[int, object], dict[int, object]]:
        if commands is None:
            return {}, {}

        active_commands: dict[int, object] = {}
        degraded_commands: dict[int, object] = {}
        for drone_id, command in commands.commands.items():
            if drone_id in degraded_follower_ids:
                degraded_commands[drone_id] = command
            else:
                active_commands[drone_id] = command
        return active_commands, degraded_commands
