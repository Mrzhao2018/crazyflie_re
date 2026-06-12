"""真机主循环"""

import hashlib
import json
import time
import logging
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
import numpy as np
from ..runtime.mission_fsm import MissionState
from ..runtime.pose_snapshot import PoseSnapshot
from ..runtime.telemetry import TelemetryRecord
from ..runtime.follower_controller import FollowerCommandSet
from ..runtime.mission_telemetry_reporter import MissionTelemetryReporter
from ..runtime.failure_policy import FailurePolicy
from ..runtime.landing_flow import LandingFlow
from .mission_errors import MissionErrorDefinition, MissionErrors
from .startup_progress import NullProgressReporter, StartupProgressReporter
from ..adapters.trajectory_common import (
    TRAJECTORY_MEMORY_BYTES,
    estimate_trajectory_bytes,
)
from ..runtime.offline_swarm_sampler import _evaluate_trajectory_spec

logger = logging.getLogger(__name__)


class _StartupAborted(Exception):
    """Internal sentinel: _fail_start was invoked; start() should return False."""


class RealMissionApp:
    """真机任务应用"""

    def __init__(
        self,
        components: dict,
        progress: StartupProgressReporter | None = None,
    ):
        self.comp = components
        # 常用组件在 __init__ 一次性取出，主循环 + _record_* 直接走属性而非 dict 访问。
        self.telemetry = components.get("telemetry")
        self.fleet = components.get("fleet")
        self.scheduler = components.get("scheduler")
        self.transport = components.get("transport")
        self.fsm = components.get("fsm")
        self.telemetry_reporter = MissionTelemetryReporter(
            self.telemetry, self.fleet
        )
        self.failure_policy = FailurePolicy(self)
        self.landing_flow = LandingFlow(self)
        self._running = False
        self._last_processed_seq = -1
        self._last_streaming_setpoint_event_t: float | None = None
        self._trajectory_started = False
        self._trajectory_start_confirmed = False
        self._trajectory_start_attempts = 0
        self._trajectory_start_elapsed: float | None = None
        self._trajectory_start_snapshot_seq: int | None = None
        self._trajectory_start_snapshot_t_meas: float | None = None
        self._trajectory_start_reference_positions: dict[int, np.ndarray] = {}
        self._trajectory_start_moving_leader_ids: list[int] = []
        self._shutdown_flushed = False
        self._terminal_land_executed = False
        self._trajectory_state = "inactive"
        self._trajectory_terminal_reason = None
        self._fast_gate_group_degrade_streaks: dict[int, int] = {}
        self._readiness_report = {
            "wait_for_params": {},
            "reset_estimator": {},
            "trajectory_prepare": {},
            "pose_ready": False,
        }
        self._config_fingerprint = self._build_config_fingerprint()
        self._progress: StartupProgressReporter = progress or NullProgressReporter()

    @contextmanager
    def _phase(self, key: str, title: str, total: int | None = None):
        telemetry = self.telemetry
        if telemetry is not None:
            telemetry.record_event("startup_phase", phase=key, status="begin")
        start = time.monotonic()
        try:
            with self._progress.phase(key, title, total):
                yield
        except Exception as exc:
            if telemetry is not None:
                telemetry.record_event(
                    "startup_phase",
                    phase=key,
                    status="fail",
                    duration_s=time.monotonic() - start,
                    detail=str(exc)[:200],
                )
            raise
        else:
            if telemetry is not None:
                telemetry.record_event(
                    "startup_phase",
                    phase=key,
                    status="ok",
                    duration_s=time.monotonic() - start,
                )

    def _last_connect_report(self) -> dict[str, object]:
        link_manager = self.comp.get("link_manager")
        if link_manager is None or not hasattr(link_manager, "last_connect_report"):
            return {}
        report = link_manager.last_connect_report()
        if not isinstance(report, dict):
            return {}
        return report

    def _mission_state_value(self) -> str | None:
        fsm = self.fsm
        return fsm.state().value if fsm is not None else None

    def _active_follower_ids(self) -> list[int]:
        configured = getattr(self.comp["config"].control, "active_follower_ids", None)
        if configured is None:
            return list(self.comp["fleet"].follower_ids())
        follower_set = set(self.comp["fleet"].follower_ids())
        return [int(drone_id) for drone_id in configured if int(drone_id) in follower_set]

    def _active_drone_ids(self) -> list[int]:
        configured = self.comp.get("active_drone_ids")
        if configured is not None:
            return [int(drone_id) for drone_id in configured]
        return list(self.comp["fleet"].leader_ids()) + self._active_follower_ids()

    def _fast_gate_groups_passing_streak(self, group_ids: list[int]) -> list[int]:
        threshold = int(
            getattr(
                self.comp["config"].safety,
                "fast_gate_group_degrade_streak",
                1,
            )
        )
        active = set(int(group_id) for group_id in group_ids)
        for group_id in list(self._fast_gate_group_degrade_streaks):
            if group_id not in active:
                self._fast_gate_group_degrade_streaks.pop(group_id, None)
        ready = []
        for group_id in sorted(active):
            streak = self._fast_gate_group_degrade_streaks.get(group_id, 0) + 1
            self._fast_gate_group_degrade_streaks[group_id] = streak
            if streak >= threshold:
                ready.append(group_id)
        return ready

    def _startup_onboard_controller_for(
        self, drone_id: int, output_mode: str, onboard_ctrl: str
    ) -> str:
        if output_mode == "full_state" and self.comp["fleet"].is_follower(drone_id):
            return "pid"
        return onboard_ctrl

    def _run_elapsed_start_offset(self, startup_mode: str) -> float:
        if startup_mode != "auto":
            return 0.0
        mission_profile = self.comp.get("mission_profile")
        start_time_fn = getattr(mission_profile, "trajectory_start_time", None)
        if not callable(start_time_fn):
            return 0.0
        try:
            return max(float(start_time_fn()), 0.0)
        except Exception:
            return 0.0

    def _stop_high_level_commander(self, drone_id: int) -> None:
        stop_fn = getattr(self.transport, "stop_high_level_commander", None)
        if not callable(stop_fn):
            return
        stop_fn(drone_id)
        if self.telemetry is not None:
            self.telemetry.record_event(
                "stop_high_level_commander",
                ok=True,
                drone_id=int(drone_id),
            )

    def _current_full_state_targets(self, follower_ids: list[int]) -> dict[int, np.ndarray]:
        snapshot = self.comp["pose_bus"].latest()
        if snapshot is None:
            raise RuntimeError("full_state handoff requires a fresh pose snapshot")

        targets: dict[int, np.ndarray] = {}
        for drone_id in follower_ids:
            idx = self.comp["fleet"].id_to_index(drone_id)
            if not snapshot.fresh_mask[idx]:
                raise RuntimeError(
                    f"full_state handoff requires fresh pose for drone {drone_id}"
                )
            targets[int(drone_id)] = np.array(snapshot.positions[idx], dtype=float)
        return targets

    def _send_full_state_handoff_setpoint(
        self, targets: dict[int, np.ndarray], *, reason: str
    ) -> None:
        from ..runtime.command_plan import FollowerAction

        actions = [
            FollowerAction(
                kind="full_state",
                drone_id=int(drone_id),
                velocity=np.zeros(3, dtype=float),
                position=np.array(target, dtype=float),
                acceleration=np.zeros(3, dtype=float),
            )
            for drone_id, target in targets.items()
        ]
        result = self.comp["follower_executor"].execute_velocity(actions)
        if self.telemetry is not None:
            self.telemetry.record_event(
                "full_state_handoff_setpoint",
                ok=not bool(result.get("failures")),
                reason=reason,
                drone_ids=[int(action.drone_id) for action in actions],
                target_positions={
                    int(drone_id): np.array(target, dtype=float).tolist()
                    for drone_id, target in targets.items()
                },
                result=result,
            )
        failures = result.get("failures") or []
        if failures:
            raise RuntimeError(f"full_state handoff setpoint failed: {failures}")

    def _warmup_full_state_followers(
        self,
        follower_ids: list[int],
        *,
        target_positions: dict[int, np.ndarray] | None = None,
    ) -> None:
        control = self.comp["config"].control
        duration_s = float(getattr(control, "full_state_warmup_s", 0.0))
        if duration_s <= 0.0 or not follower_ids:
            return

        targets = (
            {
                int(drone_id): np.array(target, dtype=float)
                for drone_id, target in target_positions.items()
            }
            if target_positions is not None
            else self._current_full_state_targets(follower_ids)
        )

        from ..runtime.command_plan import FollowerAction

        rate_hz = float(getattr(control, "full_state_warmup_rate_hz", 20.0))
        period_s = 1.0 / max(rate_hz, 1e-6)
        deadline = time.monotonic() + duration_s
        ticks = 0
        while time.monotonic() < deadline:
            actions = [
                FollowerAction(
                    kind="full_state",
                    drone_id=drone_id,
                    velocity=np.zeros(3, dtype=float),
                    position=target,
                    acceleration=np.zeros(3, dtype=float),
                )
                for drone_id, target in targets.items()
            ]
            result = self.comp["follower_executor"].execute_velocity(actions)
            ticks += 1
            failures = result.get("failures") or []
            if failures:
                self.telemetry_reporter.record_executor_summary(
                    "full_state_warmup_execution", [result]
                )
                raise RuntimeError(f"full_state warmup failed: {failures}")
            time.sleep(period_s)

        if self.telemetry is not None:
            self.telemetry.record_event(
                "full_state_warmup",
                ok=True,
                drone_ids=[int(drone_id) for drone_id in follower_ids],
                duration_s=duration_s,
                rate_hz=rate_hz,
                ticks=ticks,
                target_positions={
                    int(drone_id): target.tolist()
                    for drone_id, target in targets.items()
                },
            )

    def _apply_onboard_param_overrides(self, drone_ids: list[int]) -> None:
        overrides = getattr(
            self.comp["config"].control, "onboard_param_overrides", None
        )
        if not overrides or not drone_ids:
            return

        transport = self.comp["transport"]
        for drone_id in drone_ids:
            for name, value in overrides.items():
                transport.set_param(drone_id, name, value)
                if self.telemetry is not None:
                    self.telemetry.record_event(
                        "set_onboard_param_override",
                        ok=True,
                        drone_id=int(drone_id),
                        name=str(name),
                        value=value,
                    )

    def _fail_follower_entry_align(self, reason: str, **details) -> None:
        logger.error(reason)
        self._record_error_event(
            definition=MissionErrors.Readiness.STARTUP_FAILED,
            message=reason,
            follower_entry_align=True,
            **details,
        )
        self._emergency_land(trigger_error=MissionErrors.Readiness.STARTUP_FAILED)
        raise _StartupAborted(reason)

    def _align_followers_to_entry_reference(
        self,
        follower_ids: list[int],
        *,
        entry_leader_positions: dict[int, object] | None = None,
    ) -> None:
        startup_cfg = self.comp["config"].startup
        if not getattr(startup_cfg, "follower_align_enabled", True) or not follower_ids:
            if self.telemetry is not None:
                self.telemetry.record_event(
                    "follower_entry_align",
                    ok=True,
                    skipped=True,
                    reason="disabled" if follower_ids else "no_followers",
                    drone_ids=[int(drone_id) for drone_id in follower_ids],
                )
            return

        duration_s = float(getattr(startup_cfg, "follower_align_duration_s", 2.0))
        settle_s = float(getattr(startup_cfg, "follower_align_settle_s", 0.5))
        tolerance_m = float(getattr(startup_cfg, "follower_align_tolerance_m", 0.25))

        snapshot = self.comp["pose_bus"].latest()
        if snapshot is None:
            self._fail_follower_entry_align("follower 起始对齐失败：缺少 pose snapshot")

        if entry_leader_positions:
            leader_positions = {
                int(drone_id): np.asarray(position, dtype=float)
                for drone_id, position in entry_leader_positions.items()
            }
            ref_t_meas = None
            frame_condition_number = None
            reference_source = "planned_entry_leader_positions"
        else:
            frame = self.comp["frame_estimator"].estimate(
                snapshot,
                self.comp["fleet"].leader_ids(),
            )
            if frame is None or not getattr(frame, "valid", False):
                self._fail_follower_entry_align(
                    "follower 起始对齐失败：leader frame 无效",
                    frame_condition_number=getattr(frame, "condition_number", None),
                    snapshot_seq=snapshot.seq,
                )
            leader_positions = frame.leader_positions
            ref_t_meas = snapshot.t_meas
            frame_condition_number = getattr(frame, "condition_number", None)
            reference_source = "measured_leader_frame"

        follower_ref = self.comp["follower_ref_gen"].compute(
            leader_positions,
            ref_t_meas,
        )
        if follower_ref is None or not getattr(follower_ref, "valid", False):
            self._fail_follower_entry_align(
                "follower 起始对齐失败：follower reference 无效",
                snapshot_seq=snapshot.seq,
            )

        targets: dict[int, np.ndarray] = {}
        missing = []
        for drone_id in follower_ids:
            target = follower_ref.target_positions.get(drone_id)
            if target is None:
                missing.append(int(drone_id))
                continue
            targets[int(drone_id)] = np.asarray(target, dtype=float)
        if missing:
            self._fail_follower_entry_align(
                "follower 起始对齐失败：缺少 follower reference",
                missing_follower_ids=missing,
                snapshot_seq=snapshot.seq,
            )

        result = self.comp["follower_executor"].go_to_positions(
            [int(drone_id) for drone_id in follower_ids],
            targets,
            duration=duration_s,
        )
        self.telemetry_reporter.record_executor_summary(
            "follower_entry_align_execution",
            [result],
        )
        failures = result.get("failures") or []
        if failures:
            self._fail_follower_entry_align(
                "follower 起始对齐失败：go_to 下发失败",
                failures=failures,
                snapshot_seq=snapshot.seq,
            )

        time.sleep(duration_s + settle_s)

        validation_snapshot = self.comp["pose_bus"].latest()
        if validation_snapshot is None:
            self._fail_follower_entry_align("follower 起始对齐失败：缺少验证 pose snapshot")

        measured_positions: dict[int, list[float]] = {}
        target_positions: dict[int, list[float]] = {}
        fresh: dict[int, bool] = {}
        errors: dict[int, float] = {}
        failed_ids: list[int] = []
        for drone_id, target in targets.items():
            idx = self.comp["fleet"].id_to_index(drone_id)
            measured = np.asarray(validation_snapshot.positions[idx], dtype=float)
            is_fresh = bool(validation_snapshot.fresh_mask[idx])
            error = float(np.linalg.norm(measured - target))
            measured_positions[drone_id] = measured.tolist()
            target_positions[drone_id] = target.tolist()
            fresh[drone_id] = is_fresh
            errors[drone_id] = error
            if not is_fresh or error > tolerance_m:
                failed_ids.append(drone_id)

        ok = not failed_ids
        self.comp["telemetry"].record_event(
            "follower_entry_align",
            ok=ok,
            drone_ids=[int(drone_id) for drone_id in follower_ids],
            duration_s=duration_s,
            settle_s=settle_s,
            tolerance_m=tolerance_m,
            target_positions=target_positions,
            measured_positions=measured_positions,
            errors=errors,
            fresh=fresh,
            failed_drone_ids=failed_ids,
            reference_source=reference_source,
            frame_condition_number=frame_condition_number,
            snapshot_seq=validation_snapshot.seq,
        )
        if not ok:
            self._fail_follower_entry_align(
                "follower 起始对齐失败：位置误差或 pose freshness 不满足要求",
                failed_drone_ids=failed_ids,
                errors=errors,
                fresh=fresh,
                tolerance_m=tolerance_m,
                snapshot_seq=validation_snapshot.seq,
            )

    def _record_error_event(
        self,
        *,
        definition: MissionErrorDefinition,
        message: str,
        exception: Exception | None = None,
        **details,
    ) -> None:
        self.telemetry_reporter.record_error(
            definition=definition,
            message=message,
            mission_state=self._mission_state_value(),
            exception=exception,
            **details,
        )

    def _check_velocity_stream_watchdog(self, snapshot_t_meas: float) -> list[dict]:
        return self.failure_policy.check_velocity_stream_watchdog(snapshot_t_meas)

    def _apply_watchdog_degrade(self, stale_followers: list[dict]) -> None:
        self.failure_policy.apply_watchdog_degrade(stale_followers)

    def _clear_watchdog_degrade(
        self, *, active_commands: dict[int, object] | None = None
    ) -> None:
        self.failure_policy.clear_watchdog_degrade(active_commands=active_commands)

    def _apply_follower_failure_policy(
        self, follower_velocity_result: dict
    ) -> None:
        self.failure_policy.apply_follower_failure_policy(follower_velocity_result)

    def _execute_hold_actions(self, actions):
        if self.comp["config"].control.output_mode != "full_state":
            return self.comp["follower_executor"].execute_hold(actions)

        from ..runtime.command_plan import FollowerAction

        snapshot = self.comp["pose_bus"].latest()
        if snapshot is None:
            raise RuntimeError("full_state hold requires a pose snapshot")

        full_state_actions = []
        hold_positions = {}
        fresh = {}
        for action in actions:
            drone_id = int(action.drone_id)
            idx = self.comp["fleet"].id_to_index(drone_id)
            position = np.array(snapshot.positions[idx], dtype=float)
            hold_positions[drone_id] = position.tolist()
            fresh[drone_id] = bool(snapshot.fresh_mask[idx])
            full_state_actions.append(
                FollowerAction(
                    kind="full_state",
                    drone_id=drone_id,
                    velocity=np.zeros(3, dtype=float),
                    position=position,
                    acceleration=np.zeros(3, dtype=float),
                )
            )

        result = self.comp["follower_executor"].execute_velocity(full_state_actions)
        result = dict(result)
        result["kind"] = "full_state_hold"
        if self.telemetry is not None:
            self.telemetry.record_event(
                "full_state_hold_setpoint",
                ok=not bool(result.get("failures")),
                drone_ids=[int(action.drone_id) for action in actions],
                positions=hold_positions,
                fresh=fresh,
                result=result,
            )
        return result

    def _record_streaming_setpoint_active(
        self,
        actions,
        result: dict,
        *,
        plan_diagnostics: dict | None = None,
        execute_duration_s: float | None = None,
    ) -> None:
        if self.telemetry is None:
            return
        now = time.monotonic()
        dt_since_last = (
            None
            if self._last_streaming_setpoint_event_t is None
            else now - self._last_streaming_setpoint_event_t
        )
        self._last_streaming_setpoint_event_t = now
        plan_diagnostics = plan_diagnostics or {}
        drone_ids = [int(action.drone_id) for action in actions]
        kinds = list(dict.fromkeys(getattr(action, "kind", "velocity") for action in actions))
        self.telemetry.record_event(
            "streaming_setpoint_active",
            code="STREAMING_SETPOINT_ACTIVE",
            action_count=len(actions),
            drone_ids=drone_ids,
            kinds=kinds,
            ok=not bool(result.get("failures")),
            successes=list(result.get("successes", [])),
            failure_count=len(result.get("failures", [])),
            dt_since_last_streaming_setpoint=dt_since_last,
            execute_duration_s=execute_duration_s,
            sent_groups=list(plan_diagnostics.get("follower_tx_groups_sent") or []),
            blocked_groups=list(plan_diagnostics.get("follower_tx_groups_blocked") or []),
            stale_groups=list(plan_diagnostics.get("follower_tx_groups_stale") or []),
        )

    @staticmethod
    def _split_degraded_commands(
        commands, degraded_follower_ids: set[int]
    ) -> tuple[dict[int, object], dict[int, object]]:
        return FailurePolicy.split_degraded_commands(commands, degraded_follower_ids)

    def _safe_transition(self, target: MissionState) -> bool:
        try:
            self.comp["fsm"].transition(target)
            if "telemetry" in self.comp:
                self.comp["telemetry"].record_event(
                    "fsm_transition", target=target.value
                )
            return True
        except ValueError as exc:
            logger.error("FSM transition failed: %s", exc)
            self.comp["fsm"].force_abort()
            return False

    def _fail_start(
        self,
        reason: str,
        *,
        definition: MissionErrorDefinition = MissionErrors.Readiness.STARTUP_FAILED,
        exception: Exception | None = None,
        **details,
    ) -> None:
        if exception is None:
            logger.error(reason)
        else:
            logger.error("%s: %s", reason, exception)
        self._record_error_event(
            definition=definition,
            message=reason,
            exception=exception,
            **details,
        )
        self.comp["fsm"].force_abort()
        self.shutdown()
        raise _StartupAborted(reason)

    def start(self):
        """启动"""
        logger.info("=== 启动真机任务 ===")
        try:
            return self._start_impl()
        except _StartupAborted:
            return False
        finally:
            self._progress.close()

    def _start_impl(self):
        comm = self.comp["config"].comm
        startup_mode = self.comp.get("startup_mode", "auto")
        output_mode = self.comp["config"].control.output_mode
        onboard_ctrl = self.comp["config"].control.onboard_controller
        active_follower_ids = self._active_follower_ids()
        leader_ref_gen = self.comp["leader_ref_gen"]
        first_leader_ref = leader_ref_gen.reference_at(0.0)
        trajectory_enabled = (
            startup_mode == "auto"
            and first_leader_ref.mode == "trajectory"
            and first_leader_ref.trajectory is not None
        )
        total_phases = 6  # connect + onboard_controller + pose + health + preflight + takeoff
        if comm.readiness_wait_for_params:
            total_phases += 1
        if comm.readiness_reset_estimator:
            total_phases += 1
        if trajectory_enabled:
            total_phases += 1
        if output_mode == "full_state" and onboard_ctrl != "pid":
            total_phases += 1
            if getattr(self.comp["config"].control, "full_state_warmup_s", 0.0) > 0:
                total_phases += 1
        self._progress.set_total_phases(total_phases)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        telemetry_path = Path("telemetry") / f"run_real_{timestamp}.jsonl"
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self.comp["telemetry_path"] = str(telemetry_path)
        self.comp["telemetry"].open(str(telemetry_path))
        self.comp["telemetry"].write_header(
            config_fingerprint=self._config_fingerprint,
            readiness=self._readiness_report,
            fleet_meta=self._fleet_meta(),
        )
        self.comp["telemetry"].record_event(
            "startup_mode", mode=self.comp.get("startup_mode", "auto")
        )
        self.comp["telemetry"].record_event(
            "config_fingerprint",
            ok=True,
            fingerprint=self._config_fingerprint,
        )
        self.comp["telemetry"].flush()

        # 连接 (phase 1)
        if not self._safe_transition(MissionState.CONNECT):
            self._fail_start(
                "FSM failed before connect",
                definition=MissionErrors.Readiness.FSM_CONNECT_TRANSITION_FAILED,
            )

        active_drone_ids = self._active_drone_ids()
        radio_group_count = len(
            {self.comp["fleet"].get_radio_group(d) for d in active_drone_ids}
        )
        connect_progress_state = {"done": 0}

        def _connect_on_group_start(group_event):
            self.telemetry_reporter.record_connect_group_start(group_event)

        def _connect_on_group_result(group_result):
            self.telemetry_reporter.record_connect_group_result(group_result)
            connect_progress_state["done"] += 1
            group_id = group_result.get("group_id") if isinstance(group_result, dict) else None
            self._progress.step(
                connect_progress_state["done"],
                radio_group_count,
                detail=f"group={group_id}" if group_id is not None else None,
            )
            self._record_link_state_events()

        connect_report: dict[str, object] = {}
        with self._phase("connect", "连接 Crazyflie", total=radio_group_count):
            try:
                connect_report = self.comp["link_manager"].connect_all(
                    on_group_start=_connect_on_group_start,
                    on_group_result=_connect_on_group_result,
                    parallel_groups=comm.connect_groups_in_parallel,
                )
                self.telemetry_reporter.record_connect_all(ok=True, report=connect_report)
            except Exception as exc:
                connect_report = self._last_connect_report()
                self.telemetry_reporter.record_connect_all(
                    ok=False,
                    report=connect_report,
                    error=str(exc),
                )
                self._fail_start(
                    "连接失败",
                    definition=MissionErrors.Connection.CONNECT_ALL_FAILED,
                    exception=exc,
                    connect_outcome=self.telemetry_reporter.connect_all_outcome(connect_report, False),
                    failed_group_ids=self.telemetry_reporter.failed_connect_group_ids(connect_report),
                    connected=connect_report.get("connected", []),
                    failures=connect_report.get("failures", []),
                    radio_groups=connect_report.get("radio_groups", {}),
                )

        if comm.readiness_wait_for_params:
            wfp_total = len(active_drone_ids)
            with self._phase(
                "wait_for_params", "等待参数同步", total=wfp_total
            ):
                drone_id = None
                wfp_done = {"count": 0}
                try:
                    group_pool = self.comp.get("group_executor_pool")

                    def _on_done(drone_id: int) -> None:
                        self._readiness_report["wait_for_params"][drone_id] = True
                        self.comp["telemetry"].record_event(
                            "wait_for_params", drone_id=drone_id, ok=True
                        )
                        wfp_done["count"] += 1
                        self._progress.step(
                            wfp_done["count"],
                            wfp_total,
                            detail=f"drone={drone_id}",
                        )

                    if group_pool is not None:
                        from ..adapters.wait_for_params import wait_for_params_per_group

                        wait_for_params_per_group(
                            self.comp["transport"],
                            self.comp["fleet"],
                            group_pool,
                            drone_ids=active_drone_ids,
                            on_done=_on_done,
                        )
                    else:
                        for drone_id in active_drone_ids:
                            self.comp["transport"].wait_for_params(drone_id)
                            _on_done(drone_id)
                except Exception as exc:
                    self._fail_start(
                        "参数同步失败",
                        definition=MissionErrors.Readiness.WAIT_FOR_PARAMS_FAILED,
                        exception=exc,
                        drone_id=drone_id,
                    )

        if comm.readiness_reset_estimator:
            re_total = len(active_drone_ids)
            with self._phase("reset_estimator", "重置估计器", total=re_total):
                drone_id = None
                re_done = {"count": 0}
                try:
                    def _on_reset_done(done_drone_id: int) -> None:
                        self._readiness_report["reset_estimator"][done_drone_id] = True
                        self.comp["telemetry"].record_event(
                            "reset_estimator", drone_id=done_drone_id, ok=True
                        )
                        re_done["count"] += 1
                        detail = f"drone={done_drone_id}"
                        health_sample = self.comp["health_bus"].latest().get(done_drone_id)
                        if health_sample is not None:
                            values = getattr(health_sample, "values", {})
                            variances = [
                                values.get("kalman.varPX"),
                                values.get("kalman.varPY"),
                                values.get("kalman.varPZ"),
                            ]
                            present = [
                                float(value)
                                for value in variances
                                if value is not None
                            ]
                            if present:
                                detail = (
                                    f"{detail} var_max={max(present):.4g}"
                                )
                        self._progress.step(
                            re_done["count"], re_total, detail=detail
                        )

                    group_pool = self.comp.get("group_executor_pool")
                    if group_pool is not None:
                        from ..adapters.wait_for_params import reset_estimator_per_group

                        reset_estimator_per_group(
                            self.comp["transport"],
                            self.comp["fleet"],
                            group_pool,
                            drone_ids=active_drone_ids,
                            on_done=_on_reset_done,
                        )
                    else:
                        for drone_id in active_drone_ids:
                            self.comp["transport"].reset_estimator_and_wait(drone_id)
                            _on_reset_done(drone_id)
                except Exception as exc:
                    self._fail_start(
                        "重置估计器失败",
                        definition=MissionErrors.Readiness.RESET_ESTIMATOR_FAILED,
                        exception=exc,
                        drone_id=drone_id,
                    )

        startup_onboard_ctrl = "pid" if output_mode == "full_state" else onboard_ctrl
        oc_total = len(active_drone_ids)
        with self._phase(
            "onboard_controller", "设置 onboard controller", total=oc_total
        ):
            drone_id = None
            attempted_controller = None
            controller_switched: list[tuple[int, str]] = []
            oc_done = 0
            try:
                for drone_id in active_drone_ids:
                    attempted_controller = self._startup_onboard_controller_for(
                        drone_id, output_mode, onboard_ctrl
                    )
                    self.comp["transport"].set_onboard_controller(
                        drone_id, attempted_controller
                    )
                    controller_switched.append((drone_id, attempted_controller))
                    self.comp["telemetry"].record_event(
                        "set_onboard_controller",
                        drone_id=drone_id,
                        controller=attempted_controller,
                        requested_runtime_controller=onboard_ctrl,
                        ok=True,
                    )
                    oc_done += 1
                    self._progress.step(
                        oc_done, oc_total, detail=f"drone={drone_id}"
                    )
            except Exception as exc:
                self.comp["telemetry"].record_event(
                    "set_onboard_controller",
                    drone_id=drone_id,
                    controller=attempted_controller,
                    requested_runtime_controller=onboard_ctrl,
                    ok=False,
                    error=str(exc),
                )
                if output_mode == "full_state":
                    for rollback_drone_id, rollback_from in reversed(controller_switched):
                        try:
                            self.comp["transport"].set_onboard_controller(
                                rollback_drone_id, "pid"
                            )
                            self.comp["telemetry"].record_event(
                                "rollback_onboard_controller",
                                drone_id=rollback_drone_id,
                                from_controller=rollback_from,
                                controller="pid",
                                ok=True,
                            )
                        except Exception as rollback_exc:
                            self.comp["telemetry"].record_event(
                                "rollback_onboard_controller",
                                drone_id=rollback_drone_id,
                                from_controller=rollback_from,
                                controller="pid",
                                ok=False,
                                error=str(rollback_exc),
                            )
                    self._fail_start(
                        "full_state 模式下设置 onboard controller 失败，中止启动",
                        exception=exc,
                        drone_id=drone_id,
                        controller=onboard_ctrl,
                        output_mode=output_mode,
                    )
                self._progress.warn(
                    f"onboard controller {startup_onboard_ctrl} 设置失败 (drone={drone_id}): {exc}"
                )
                logger.warning(
                    "onboard controller %s 设置失败 (drone=%s): %s —— 继续启动，但沿用机载当前 controller 状态",
                    startup_onboard_ctrl,
                    drone_id,
                    exc,
                )

        override_ids = (
            active_drone_ids
            if output_mode == "full_state"
            else active_drone_ids
        )
        try:
            self._apply_onboard_param_overrides(override_ids)
        except Exception as exc:
            self.comp["telemetry"].record_event(
                "set_onboard_param_override",
                ok=False,
                drone_ids=[int(drone_id) for drone_id in override_ids],
                overrides=getattr(
                    self.comp["config"].control, "onboard_param_overrides", None
                ),
                error=str(exc),
            )
            self._fail_start(
                "设置 onboard 参数覆盖失败",
                exception=exc,
                output_mode=output_mode,
                affected_role="follower" if output_mode == "full_state" else "all",
            )

        pose_total = len(active_drone_ids)
        with self._phase("pose_source", "定位就绪", total=pose_total):
            try:
                self.comp["pose_source"].register_callback(self._on_pose_update)
                self.comp["pose_source"].start()
            except Exception as exc:
                self._fail_start(
                    "定位源启动失败",
                    definition=MissionErrors.Readiness.POSE_SOURCE_START_FAILED,
                    exception=exc,
                )

            console_tap = self.comp.get("console_tap")
            if console_tap is not None:
                try:
                    console_tap._on_line = self._on_console_line
                    console_tap.start()
                except Exception:
                    logger.exception("Console tap start failed; 继续启动")

            pose_ready = False
            for _ in range(20):
                snapshot = self.comp["pose_bus"].latest()
                if snapshot:
                    fresh_count = sum(
                        1
                        for drone_id in active_drone_ids
                        if snapshot.fresh_mask[
                            self.comp["fleet"].id_to_index(drone_id)
                        ]
                    )
                    self._progress.step(fresh_count, pose_total)
                    if fresh_count == pose_total:
                        self._readiness_report["pose_ready"] = True
                        self.comp["telemetry"].record_event("pose_ready", ok=True)
                        pose_ready = True
                        break
                time.sleep(0.1)
            if not pose_ready:
                self.comp["telemetry"].record_event("pose_ready", ok=False)
                self._fail_start(
                    "部分无人机定位未就绪，中止任务",
                    definition=MissionErrors.Readiness.POSE_TIMEOUT,
                )

        health_total = len(active_drone_ids)
        with self._phase("health_ready", "健康数据就绪", total=health_total):
            health_ok = False
            for _ in range(30):
                health_samples = self.comp["health_bus"].latest()
                ready_ids = [
                    drone_id
                    for drone_id in active_drone_ids
                    if drone_id in health_samples
                    and "pm.vbat" in health_samples[drone_id].values
                ]
                self._progress.step(len(ready_ids), health_total)
                if len(ready_ids) == health_total:
                    self._readiness_report["health_ready"] = True
                    self.comp["telemetry"].record_event("health_ready", ok=True)
                    health_ok = True
                    break
                time.sleep(0.1)
            if not health_ok:
                self.comp["telemetry"].record_event("health_ready", ok=False)
                self._fail_start(
                    "健康状态数据未就绪，中止任务",
                    definition=MissionErrors.Readiness.HEALTH_TIMEOUT,
                )

        leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if trajectory_enabled and leader_ref.mode == "trajectory":
            per_leader = leader_ref.trajectory.get("per_leader", {})
            leader_ids = list(self.comp["fleet"].leader_ids())
            tu_total = len(leader_ids)
            with self._phase("trajectory_upload", "轨迹上传", total=tu_total):
                try:
                    trajectory_upload_specs = {}
                    for drone_id in leader_ids:
                        spec = per_leader.get(drone_id, {})
                        pieces = spec.get("pieces", [])
                        start_addr = spec.get("start_addr", 0)
                        trajectory_id = spec.get("trajectory_id", 1)
                        trajectory_type = spec.get("trajectory_type", "poly4d")
                        estimated_bytes = estimate_trajectory_bytes(
                            pieces, trajectory_type
                        )
                        fits_memory = (
                            start_addr + estimated_bytes <= TRAJECTORY_MEMORY_BYTES
                        )
                        self.comp["telemetry"].record_event(
                            "trajectory_budget_check",
                            drone_id=drone_id,
                            pieces=len(pieces),
                            estimated_bytes=estimated_bytes,
                            start_addr=start_addr,
                            capacity=TRAJECTORY_MEMORY_BYTES,
                            fits_memory=fits_memory,
                            trajectory_type=trajectory_type,
                        )
                        trajectory_upload_specs[drone_id] = {
                            "pieces": pieces,
                            "start_addr": start_addr,
                            "trajectory_id": trajectory_id,
                            "trajectory_type": trajectory_type,
                        }

                    upload_results = self.comp["transport"].upload_trajectories_by_group(
                        trajectory_upload_specs,
                        parallel_groups=comm.trajectory_upload_groups_in_parallel,
                    )

                    tu_done = 0
                    for drone_id in leader_ids:
                        spec = per_leader.get(drone_id, {})
                        pieces = spec.get("pieces", [])
                        start_addr = spec.get("start_addr", 0)
                        trajectory_id = spec.get("trajectory_id", 1)
                        trajectory_type = spec.get("trajectory_type", "poly4d")
                        estimated_bytes = estimate_trajectory_bytes(
                            pieces, trajectory_type
                        )
                        fits_memory = (
                            start_addr + estimated_bytes <= TRAJECTORY_MEMORY_BYTES
                        )
                        piece_count = int(upload_results[drone_id]["piece_count"])
                        self._readiness_report["trajectory_prepare"][drone_id] = {
                            "uploaded": True,
                            "defined": True,
                            "pieces": piece_count,
                            "estimated_bytes": estimated_bytes,
                            "fits_memory": fits_memory,
                            "trajectory_id": trajectory_id,
                            "trajectory_type": trajectory_type,
                            "nominal_position": spec.get("nominal_position"),
                        }
                        self.comp["telemetry"].record_event(
                            "trajectory_prepare",
                            drone_id=drone_id,
                            uploaded=True,
                            defined=True,
                            pieces=piece_count,
                            estimated_bytes=estimated_bytes,
                            fits_memory=fits_memory,
                            trajectory_id=trajectory_id,
                            trajectory_type=trajectory_type,
                            nominal_position=spec.get("nominal_position"),
                        )
                        tu_done += 1
                        self._progress.step(
                            tu_done,
                            tu_total,
                            detail=f"leader={drone_id} pieces={piece_count}",
                        )
                except Exception as exc:
                    self._fail_start(
                        "轨迹准备失败",
                        definition=MissionErrors.Readiness.TRAJECTORY_PREPARE_FAILED,
                        exception=exc,
                    )

                self._set_trajectory_state("prepared")

                self.comp["telemetry"].record_event(
                    "trajectory_readiness_summary",
                    ok=all(
                        item.get("uploaded") and item.get("defined")
                        for item in self._readiness_report["trajectory_prepare"].values()
                    ),
                    leaders=self._readiness_report["trajectory_prepare"],
                )
                self._set_trajectory_state("ready")
                logger.info("=== Trajectory readiness summary ===")
                for drone_id, item in self._readiness_report["trajectory_prepare"].items():
                    logger.info(
                        "leader=%s pieces=%s est_bytes=%s fits=%s uploaded=%s defined=%s",
                        drone_id,
                        item.get("pieces"),
                        item.get("estimated_bytes"),
                        item.get("fits_memory"),
                        item.get("uploaded"),
                        item.get("defined"),
                    )

        if not self._safe_transition(MissionState.POSE_READY):
            self._fail_start(
                "FSM failed entering POSE_READY",
                definition=MissionErrors.Readiness.FSM_POSE_READY_TRANSITION_FAILED,
            )

        if not self._safe_transition(MissionState.PREFLIGHT):
            self._fail_start(
                "FSM failed entering PREFLIGHT",
                definition=MissionErrors.Readiness.FSM_PREFLIGHT_TRANSITION_FAILED,
            )
        self.comp["readiness_report"] = self._readiness_report

        with self._phase("preflight", "preflight"):
            try:
                preflight_report = self.comp["preflight"].run()
            except Exception as exc:
                self._fail_start(
                    "Preflight 执行异常",
                    definition=MissionErrors.Readiness.PREFLIGHT_EXCEPTION,
                    exception=exc,
                )
            self.comp["telemetry"].record_event(
                "preflight",
                ok=preflight_report.ok,
                failed_codes=preflight_report.failed_codes,
            )
            if not preflight_report.ok:
                self._fail_start(
                    f"Preflight failed: {preflight_report.reasons} codes={preflight_report.failed_codes}",
                    definition=MissionErrors.Readiness.PREFLIGHT_FAILED,
                    failed_codes=preflight_report.failed_codes,
                )

        with self._phase("takeoff_settle_align", "起飞 / settle / align"):
            if not self._safe_transition(MissionState.TAKEOFF):
                self._fail_start(
                    "FSM failed entering TAKEOFF",
                    definition=MissionErrors.Readiness.FSM_TAKEOFF_TRANSITION_FAILED,
                )

            self.comp["leader_executor"].execute(
                [self._leader_takeoff_action(self.comp["fleet"].leader_ids())]
            )
            follower_takeoff_result = self.comp["follower_executor"].takeoff(
                active_follower_ids, height=0.5, duration=2.0
            )
            self.telemetry_reporter.record_executor_summary("follower_takeoff_execution", [follower_takeoff_result])

            time.sleep(3.0)

            if not self._safe_transition(MissionState.SETTLE):
                self._fail_start(
                    "FSM failed entering SETTLE",
                    definition=MissionErrors.Readiness.FSM_SETTLE_TRANSITION_FAILED,
                )
            time.sleep(2.0)

            entry_leader_positions = None
            initial_leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
            if startup_mode == "manual_leader":
                initial_leader_ref = self._manual_initial_structure_reference()
            if (
                startup_mode == "auto"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "batch_goto"
            ):
                from ..runtime.command_plan import LeaderAction

                align_action = LeaderAction(
                    kind="batch_goto",
                    drone_ids=self.comp["fleet"].leader_ids(),
                    payload={"positions": initial_leader_ref.positions, "duration": 2.0},
                )
                self.comp["leader_executor"].execute([align_action])
                self.comp["telemetry"].record_event(
                    "formation_align", ok=True, duration=2.0
                )
                time.sleep(2.2)
                entry_leader_positions = initial_leader_ref.positions
            elif (
                startup_mode == "manual_leader"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "batch_goto"
            ):
                from ..runtime.command_plan import LeaderAction

                align_action = LeaderAction(
                    kind="batch_goto",
                    drone_ids=self.comp["fleet"].leader_ids(),
                    payload={"positions": initial_leader_ref.positions, "duration": 2.0},
                )
                self.comp["leader_executor"].execute([align_action])
                self.comp["telemetry"].record_event(
                    "manual_structure_align", ok=True, duration=2.0
                )
                time.sleep(2.2)
                entry_leader_positions = initial_leader_ref.positions
            elif (
                startup_mode == "auto"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "trajectory"
            ):
                trajectory_entry_positions = self._trajectory_entry_start_positions(
                    initial_leader_ref
                )
                if trajectory_entry_positions:
                    from ..runtime.command_plan import LeaderAction

                    align_action = LeaderAction(
                        kind="batch_goto",
                        drone_ids=self.comp["fleet"].leader_ids(),
                        payload={"positions": trajectory_entry_positions, "duration": 2.0},
                    )
                    self.comp["leader_executor"].execute([align_action])
                    self.comp["telemetry"].record_event(
                        "trajectory_entry_align",
                        ok=True,
                        duration=2.0,
                        positions=trajectory_entry_positions,
                    )
                    time.sleep(2.2)
                    entry_leader_positions = trajectory_entry_positions

            self._align_followers_to_entry_reference(
                active_follower_ids,
                entry_leader_positions=entry_leader_positions,
            )

            snapshot = self.comp["pose_bus"].latest()
            if snapshot:
                flying_ids = list(self.comp["fleet"].leader_ids()) + list(
                    active_follower_ids
                )
                for drone_id in flying_ids:
                    idx = self.comp["fleet"].id_to_index(drone_id)
                    if snapshot.positions[idx][2] < 0.3:
                        flying_altitudes = {
                            int(did): float(
                                snapshot.positions[
                                    self.comp["fleet"].id_to_index(did)
                                ][2]
                            )
                            for did in flying_ids
                        }
                        flying_fresh = {
                            int(did): bool(
                                snapshot.fresh_mask[
                                    self.comp["fleet"].id_to_index(did)
                                ]
                            )
                            for did in flying_ids
                        }
                        logger.error(
                            "Takeoff validation failed: drone=%s altitude=%.3f "
                            "flying_altitudes=%s fresh=%s snapshot_seq=%s",
                            drone_id,
                            float(snapshot.positions[idx][2]),
                            flying_altitudes,
                            flying_fresh,
                            snapshot.seq,
                        )
                        self.comp["telemetry"].record_event(
                            "takeoff_validation",
                            ok=False,
                            drone_id=drone_id,
                            altitude=float(snapshot.positions[idx][2]),
                            flying_altitudes=flying_altitudes,
                            flying_fresh=flying_fresh,
                            snapshot_seq=snapshot.seq,
                        )
                        self._record_error_event(
                            definition=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                            message="起飞后高度验证失败",
                            drone_id=drone_id,
                            altitude=float(snapshot.positions[idx][2]),
                            flying_altitudes=flying_altitudes,
                            flying_fresh=flying_fresh,
                            snapshot_seq=snapshot.seq,
                        )
                        self._emergency_land(
                            trigger_error=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                        )
                        raise _StartupAborted("takeoff validation failed")

            if output_mode == "full_state" and onboard_ctrl != startup_onboard_ctrl:
                follower_ids = active_follower_ids
                full_state_handoff_targets = self._current_full_state_targets(
                    follower_ids
                )
                with self._phase(
                    "runtime_onboard_controller",
                    "切换 follower runtime controller",
                    total=len(follower_ids),
                ):
                    switched_followers: list[int] = []
                    switch_done = 0
                    drone_id = None
                    try:
                        for drone_id in follower_ids:
                            self._stop_high_level_commander(drone_id)
                            self._send_full_state_handoff_setpoint(
                                {
                                    int(drone_id): full_state_handoff_targets[
                                        int(drone_id)
                                    ]
                                },
                                reason="runtime_onboard_controller_pre",
                            )
                            self.comp["transport"].set_onboard_controller(
                                drone_id, onboard_ctrl
                            )
                            switched_followers.append(drone_id)
                            self.comp["telemetry"].record_event(
                                "set_runtime_onboard_controller",
                                drone_id=drone_id,
                                controller=onboard_ctrl,
                                startup_controller=startup_onboard_ctrl,
                                ok=True,
                            )
                            self._send_full_state_handoff_setpoint(
                                {
                                    int(drone_id): full_state_handoff_targets[
                                        int(drone_id)
                                    ]
                                },
                                reason="runtime_onboard_controller_post",
                            )
                            switch_done += 1
                            self._progress.step(
                                switch_done,
                                len(follower_ids),
                                detail=f"drone={drone_id}",
                            )
                    except Exception as exc:
                        self.comp["telemetry"].record_event(
                            "set_runtime_onboard_controller",
                            drone_id=drone_id,
                            controller=onboard_ctrl,
                            startup_controller=startup_onboard_ctrl,
                            ok=False,
                            error=str(exc),
                        )
                        for rollback_drone_id in reversed(switched_followers):
                            try:
                                self.comp["transport"].set_onboard_controller(
                                    rollback_drone_id, startup_onboard_ctrl
                                )
                                self.comp["telemetry"].record_event(
                                    "rollback_runtime_onboard_controller",
                                    drone_id=rollback_drone_id,
                                    from_controller=onboard_ctrl,
                                    controller=startup_onboard_ctrl,
                                    ok=True,
                                )
                            except Exception as rollback_exc:
                                self.comp["telemetry"].record_event(
                                    "rollback_runtime_onboard_controller",
                                    drone_id=rollback_drone_id,
                                    from_controller=onboard_ctrl,
                                    controller=startup_onboard_ctrl,
                                    ok=False,
                                    error=str(rollback_exc),
                                )
                        self._record_error_event(
                            definition=MissionErrors.Readiness.STARTUP_FAILED,
                            message="full_state runtime controller 切换失败",
                            exception=exc,
                            drone_id=drone_id,
                            controller=onboard_ctrl,
                            output_mode=output_mode,
                            affected_role="follower",
                        )
                        self._emergency_land(
                            trigger_error=MissionErrors.Readiness.STARTUP_FAILED,
                        )
                        raise _StartupAborted("runtime onboard controller switch failed")

                if getattr(self.comp["config"].control, "full_state_warmup_s", 0.0) > 0:
                    with self._phase(
                        "full_state_warmup",
                        "full-state 接管预热",
                        total=len(follower_ids),
                    ):
                        try:
                            self._warmup_full_state_followers(
                                follower_ids,
                                target_positions=full_state_handoff_targets,
                            )
                        except Exception as exc:
                            self.comp["telemetry"].record_event(
                                "full_state_warmup",
                                ok=False,
                                error=str(exc),
                                drone_ids=follower_ids,
                            )
                            self._record_error_event(
                                definition=MissionErrors.Readiness.STARTUP_FAILED,
                                message="full_state 接管预热失败",
                                exception=exc,
                                output_mode=output_mode,
                            )
                            self._emergency_land(
                                trigger_error=MissionErrors.Readiness.STARTUP_FAILED,
                            )
                            raise _StartupAborted("full_state warmup failed")

            if startup_mode == "manual_leader":
                self._initialize_manual_mode(snapshot)

            start_stabilize_s = float(
                getattr(self.comp["config"].startup, "start_stabilize_s", 0.0)
            )
            if start_stabilize_s > 0.0:
                with self._phase("start_stabilize", "起始点稳定缓冲"):
                    self.comp["telemetry"].record_event(
                        "start_stabilize",
                        ok=True,
                        duration_s=start_stabilize_s,
                    )
                    time.sleep(start_stabilize_s)

        logger.info("系统就绪")
        self.comp["telemetry"].record_event("startup_complete", ok=True)
        self._progress.close()
        return True

    def _get_latest_snapshot(self):
        """获取最新 pose snapshot"""
        return self.comp["pose_bus"].latest()

    def _check_fast_gate(self, snapshot, safety):
        """快速门控检查
        
        Returns:
            (should_abort, fast_gate_pending_ignored_ids)
        """
        fast_gate_pending_ignored_ids: set[int] = set()
        
        if getattr(
            self.comp["config"].safety,
            "fast_gate_group_degrade_enabled",
            False,
        ):
            fg = safety.fast_gate_decision(snapshot)
            if fg.action == "ABORT":
                if self._try_reconnect_on_disconnect(snapshot, fg.reason_codes):
                    return (False, fast_gate_pending_ignored_ids)
                logger.error("Fast-gate ABORT: %s", fg.reason_codes)
                self._emergency_land()
                return (True, fast_gate_pending_ignored_ids)
            if fg.action == "HOLD_GROUP":
                triggered_groups = self._fast_gate_groups_passing_streak(
                    fg.degrade_groups
                )
                degraded = (
                    self.failure_policy.apply_fast_gate_group_degrade(
                        triggered_groups,
                        getattr(
                            self.comp["config"].safety,
                            "fast_gate_group_degrade_streak",
                            1,
                        )
                    ),
                )
                fast_gate_pending_ignored_ids.update(degraded)
            else:
                self._fast_gate_group_degrade_streaks.clear()
        else:
            fast_blocked, fast_reasons = safety.fast_gate(snapshot)
            if fast_blocked:
                if self._try_reconnect_on_disconnect(snapshot, fast_reasons):
                    return (False, fast_gate_pending_ignored_ids)
                logger.error("Fast-gate triggered: %s", fast_reasons)
                self._emergency_land()
                return (True, fast_gate_pending_ignored_ids)
        
        return (False, fast_gate_pending_ignored_ids)

    def _handle_trajectory_lifecycle(
        self, startup_mode, leader_ref, snapshot, t_elapsed
    ):
        """处理轨迹生命周期（启动、检查）
        
        Returns:
            should_abort: bool
        """
        if (
            startup_mode == "auto"
            and leader_ref.mode == "trajectory"
            and not self._trajectory_started
            and self._phase_label(t_elapsed) == "formation_run"
        ):
            self._start_leader_trajectory(
                leader_ref,
                snapshot,
                mission_elapsed=t_elapsed,
            )
        elif (
            startup_mode == "auto"
            and leader_ref.mode == "trajectory"
            and self._trajectory_started
        ):
            start_status = self._check_leader_trajectory_start_motion(
                snapshot,
                mission_elapsed=t_elapsed,
                leader_ref=leader_ref,
            )
            if start_status == "failed":
                logger.error("Leader trajectory failed to start; landing")
                self._orderly_land(
                    reason_event="leader_trajectory_start_failed_land",
                    safety_action="ABORT",
                    safety_reasons=["leader trajectory did not start"],
                    safety_reason_codes=["LEADER_TRAJECTORY_START_FAILED"],
                    scheduler_reason="leader_trajectory_start_failed",
                )
                return True
        return False

    def _compute_control_state(self, snapshot, leader_ref, t_elapsed):
        """计算控制状态（frame、follower_ref、commands）
        
        Returns:
            (frame, follower_ref, commands, parked_follower_ids)
        """
        fleet = self.fleet
        frame_estimator = self.comp["frame_estimator"]
        follower_ref_gen = self.comp["follower_ref_gen"]
        follower_controller = self.comp["follower_controller"]
        
        frame = None
        follower_ref = None
        commands = None
        parked_follower_ids = []

        if self._should_compute_follower_control(snapshot, t_elapsed):
            frame_snapshot = snapshot
            if (
                leader_ref.mode == "trajectory"
                and self._trajectory_started
                and not self._trajectory_start_confirmed
            ):
                frame_snapshot = self._trajectory_start_snapshot()
            
            frame = frame_estimator.estimate(
                frame_snapshot,
                leader_ref.target_leader_ids,
            )

            if frame is not None and frame.valid:
                follower_ref = follower_ref_gen.reference_at(
                    fleet.follower_ids(),
                    frame.leader_positions,
                    frame_snapshot.t_meas,
                )

                if follower_ref is not None and follower_ref.valid:
                    commands = follower_controller.compute(
                        snapshot,
                        follower_ref,
                        self._active_follower_ids(),
                        fleet,
                    )
        
        return frame, follower_ref, commands, parked_follower_ids

    def _evaluate_safety(
        self, snapshot, frame, commands, follower_ref, health_latest, fast_gate_pending_ignored_ids
    ):
        """评估完整安全状态
        
        Returns:
            SafetyDecision
        """
        safety = self.comp["safety"]
        health_bus = self.comp["health_bus"]
        pose_bus = self.comp["pose_bus"]
        
        health_window_fn = getattr(health_bus, "recent_samples", None)
        health_window = (
            health_window_fn(
                self.comp["config"].safety.min_vbat_window_s
            )
            if callable(health_window_fn)
            else None
        )
        pose_window_fn = getattr(pose_bus, "recent_samples", None)
        pose_window = (
            pose_window_fn(self.comp["config"].safety.estimator_variance_window_s)
            if callable(pose_window_fn)
            else None
        )
        
        return safety.evaluate(
            snapshot,
            frame,
            commands,
            follower_ref,
            health=health_latest,
            health_window=health_window,
            pose_window=pose_window,
            ignored_disconnected_ids=(
                set(self.failure_policy.watchdog_degraded_followers)
                | fast_gate_pending_ignored_ids
            ),
        )

    def _handle_safety_decision(self, safety_decision):
        """处理安全决策
        
        Returns:
            should_abort: bool
        """
        if safety_decision.action == "ABORT":
            logger.error("ABORT triggered: %s", safety_decision.reasons)
            self._emergency_land()
            return True

        if safety_decision.action == "HOLD":
            logger.warning("HOLD triggered: %s", safety_decision.reasons)
            self._enter_hold_mode(
                reason_codes=safety_decision.reason_codes,
                reasons=safety_decision.reasons,
            )
        
        return False

    def _execute_control_plan(self, plan, filtered_commands):
        """执行控制计划
        
        Returns:
            success_ids: set[int]
        """
        leader_executor = self.comp["leader_executor"]
        follower_executor = self.comp["follower_executor"]
        
        plan_diagnostics = plan.diagnostics or {}
        success_ids = set()
        
        # 执行 leader 动作
        if plan.leader_actions:
            leader_results = leader_executor.execute(plan.leader_actions)
            self.telemetry_reporter.record_executor_summary("leader_execution", leader_results)
        
        # 执行 follower 动作
        if plan.follower_actions:
            execute_started = time.monotonic()
            follower_velocity_result = follower_executor.execute_velocity(
                plan.follower_actions
            )
            execute_duration_s = time.monotonic() - execute_started
            
            self._record_streaming_setpoint_active(
                plan.follower_actions,
                follower_velocity_result,
                plan_diagnostics=plan_diagnostics,
                execute_duration_s=execute_duration_s,
            )
            self.telemetry_reporter.record_executor_summary(
                "follower_velocity_execution",
                [follower_velocity_result],
            )
            self._apply_follower_failure_policy(follower_velocity_result)
            
            success_ids = set(follower_velocity_result.get("successes", []))
            self._commit_follower_controller_full_state(
                filtered_commands,
                success_ids,
            )
            self._clear_watchdog_degrade(active_commands={drone_id: None for drone_id in success_ids})
        
        return success_ids

    def _check_mission_complete(self, startup_mode, t_elapsed):
        """检查任务是否完成
        
        Returns:
            should_stop: bool
        """
        if startup_mode != "auto":
            return False
        
        mission_profile = self.comp["mission_profile"]
        if t_elapsed >= mission_profile.total_time():
            logger.info(
                "Mission duration reached (%.2fs), starting orderly landing",
                t_elapsed,
            )
            self.telemetry.record_event(
                "mission_complete",
                ok=True,
                elapsed=t_elapsed,
                total_time=mission_profile.total_time(),
            )
            self._orderly_land(
                reason_event="mission_complete_land",
                safety_action="MISSION_COMPLETE",
                safety_reasons=["mission_complete"],
                safety_reason_codes=["MISSION_COMPLETE"],
                scheduler_reason="mission_complete",
                scheduler_diagnostics={"mission_complete": True},
                trajectory_terminal_reason="mission_complete",
            )
            return True
        return False

    def _prepare_filtered_commands(self, commands):
        """准备过滤后的命令（排除降级的 followers）
        
        Returns:
            FollowerCommandSet or None
        """
        if commands is None:
            return None
        
        active_commands, _degraded_commands = self._split_degraded_commands(
            commands, self.failure_policy.watchdog_degraded_followers
        )
        
        return FollowerCommandSet(
            commands=active_commands,
            diagnostics=commands.diagnostics,
            target_positions=(
                {
                    drone_id: position
                    for drone_id, position in commands.target_positions.items()
                    if drone_id in active_commands
                }
                if commands.target_positions is not None
                else None
            ),
            target_accelerations=(
                {
                    drone_id: acceleration
                    for drone_id, acceleration in commands.target_accelerations.items()
                    if drone_id in active_commands
                }
                if commands.target_accelerations is not None
                else None
            ),
            full_state_state=(
                {
                    drone_id: state
                    for drone_id, state in commands.full_state_state.items()
                    if drone_id in active_commands
                }
                if commands.full_state_state is not None
                else None
            ),
        )

    def _record_telemetry(
        self, snapshot, t_elapsed, startup_mode,
        frame, leader_ref, follower_ref, commands,
        safety_decision, plan, health_latest, link_quality_latest
    ):
        """记录遥测数据"""
        # 计算命令范数
        follower_command_norms: dict[int, float] = {}
        if commands is not None:
            precomputed = commands.diagnostics.get("command_norms") or {}
            if precomputed:
                follower_command_norms = {
                    drone_id: float(value)
                    for drone_id, value in precomputed.items()
                }
            else:
                follower_command_norms = {
                    drone_id: float(np.linalg.norm(cmd))
                    for drone_id, cmd in commands.commands.items()
                }

        # 准备位置数据
        measured_positions = self._measured_positions(snapshot)
        leader_reference_positions = self._leader_reference_positions(leader_ref)
        follower_reference_positions = self._follower_reference_positions(
            follower_ref
        )
        phase_label = self._phase_label(t_elapsed)
        leader_mode = getattr(leader_ref, "mode", None)

        # 记录遥测
        self.telemetry.log(
            TelemetryRecord(
                t_wall=time.time(),
                mission_state=self.fsm.state().value,
                startup_mode=startup_mode,
                mission_elapsed=t_elapsed,
                trajectory_state=self._trajectory_state,
                trajectory_terminal_reason=self._trajectory_terminal_reason,
                snapshot_seq=snapshot.seq,
                snapshot_t_meas=snapshot.t_meas,
                measured_positions=measured_positions,
                fresh_mask={
                    drone_id: bool(
                        snapshot.fresh_mask[self.fleet.id_to_index(drone_id)]
                    )
                    for drone_id in self.fleet.all_ids()
                },
                disconnected_ids=list(snapshot.disconnected_ids),
                health={
                    drone_id: sample.values
                    for drone_id, sample in health_latest.items()
                },
                frame_valid=(frame.valid if frame is not None else None),
                frame_condition_number=(
                    frame.condition_number if frame is not None else None
                ),
                phase_label=phase_label,
                leader_mode=leader_mode,
                leader_reference_positions=leader_reference_positions,
                follower_reference_positions=follower_reference_positions,
                safety_action=safety_decision.action,
                safety_reasons=safety_decision.reasons,
                safety_reason_codes=safety_decision.reason_codes,
                scheduler_reason=(plan.diagnostics or {}).get("reason"),
                scheduler_diagnostics=plan.diagnostics or {},
                leader_reference_source=type(self.comp["leader_ref_gen"]).__name__,
                manual_axis=self._manual_axis(),
                manual_input_age=self._manual_input_age(),
                leader_action_count=len(plan.leader_actions),
                follower_action_count=len(plan.follower_actions),
                follower_command_norms=follower_command_norms,
                radio_link_quality=self._radio_link_quality_payload(
                    link_quality_latest
                ),
            )
        )

    def run(self):
        """主循环（完全重构版：清晰的步骤划分）"""
        # 初始化检查
        if self.fsm.state() != MissionState.SETTLE:
            logger.error("系统未就绪，无法进入RUN")
            return

        self._running = True
        if not self._safe_transition(MissionState.RUN):
            return

        # 准备运行环境
        startup_mode = self.comp.get("startup_mode", "auto")
        elapsed_start_offset = self._run_elapsed_start_offset(startup_mode)
        mission_start_time = time.time() - elapsed_start_offset
        
        logger.info("=== 进入主循环 ===")
        self.telemetry.record_event(
            "run_entered",
            ok=True,
            elapsed_start_offset=elapsed_start_offset,
        )

        # 启动手动输入（如果配置）
        manual_input = self.comp.get("manual_input")
        if manual_input is not None:
            manual_input.start()
            manual_cfg = self.comp["config"].startup.manual
            self.telemetry.record_event(
                "manual_input_started",
                ok=True,
                axis=(manual_cfg.default_axis if manual_cfg is not None else None),
            )

        try:
            # === 主控制循环 ===
            while self._running:
                self._record_link_state_events()

                # 步骤 1：获取最新状态
                snapshot = self._get_latest_snapshot()
                if snapshot is None:
                    time.sleep(0.01)
                    continue

                health_latest = self.comp["health_bus"].latest()
                link_quality_latest = (
                    self.comp.get("link_quality_bus").latest()
                    if self.comp.get("link_quality_bus") is not None
                    else {}
                )

                # 步骤 2：快速门控检查
                should_abort, fast_gate_pending_ignored_ids = self._check_fast_gate(
                    snapshot, self.comp["safety"]
                )
                if should_abort:
                    break

                # 步骤 3：计算任务时间和获取参考
                t_elapsed = time.time() - mission_start_time
                self._poll_manual_input()
                leader_ref = self.comp["leader_ref_gen"].reference_at(t_elapsed)

                # 步骤 4：处理轨迹生命周期
                if self._handle_trajectory_lifecycle(
                    startup_mode, leader_ref, snapshot, t_elapsed
                ):
                    break

                # 步骤 5：检查任务完成
                if self._check_mission_complete(startup_mode, t_elapsed):
                    break

                # 步骤 6：计算控制状态（仅在新 pose 时）
                is_new_pose = snapshot.seq > self._last_processed_seq
                frame, follower_ref, commands, parked_follower_ids = None, None, None, []
                
                if is_new_pose:
                    frame_snapshot, _ = self._leader_pose_compensated_snapshot(
                        snapshot, leader_ref, t_elapsed
                    )
                    frame = self.comp["frame_estimator"].estimate(
                        frame_snapshot, self.fleet.leader_ids()
                    )

                    if frame and frame.valid:
                        follower_ref = self.comp["follower_ref_gen"].compute(
                            frame.leader_positions,
                            frame_snapshot.t_meas,
                        )

                        if follower_ref and follower_ref.valid:
                            commands = self.comp["follower_controller"].compute(
                                snapshot,
                                follower_ref,
                                self._active_follower_ids(),
                                self.fleet,
                            )

                # 步骤 7：评估安全状态
                safety_decision = self._evaluate_safety(
                    snapshot, frame, commands, follower_ref,
                    health_latest, fast_gate_pending_ignored_ids
                )

                # 步骤 8：处理安全决策
                if self._handle_safety_decision(safety_decision):
                    break
                
                # 处理 HOLD 恢复
                if self.fsm.state() == MissionState.HOLD:
                    if safety_decision.action == "EXECUTE":
                        self._safe_transition(MissionState.RUN)
                        self.telemetry.record_event("hold_recovered", ok=True)
                        self._reset_follower_controller_full_state(
                            self._active_follower_ids()
                        )
                        self._clear_hold_tracking()

                # HOLD 状态时检查超时并继续循环
                if safety_decision.action == "HOLD":
                    if self._check_hold_timeout(t_elapsed):
                        break
                    time.sleep(0.1)
                    continue

                # 步骤 9：准备执行计划
                filtered_commands = self._prepare_filtered_commands(commands)
                parked_follower_ids = sorted(
                    self.failure_policy.watchdog_degraded_followers
                )

                # 步骤 10：生成调度计划
                plan = self.scheduler.plan(
                    snapshot,
                    self.fsm.state(),
                    leader_ref,
                    filtered_commands,
                    safety_decision,
                    parked_follower_ids=parked_follower_ids,
                )

                # 步骤 11：执行控制计划
                success_ids = self._execute_control_plan(plan, filtered_commands)

                # 步骤 12：速度流看门狗检查
                self._check_velocity_stream_watchdog(snapshot.t_meas)

                # 步骤 13：记录遥测（仅新 pose）
                if is_new_pose:
                    self._record_telemetry(
                        snapshot, t_elapsed, startup_mode,
                        frame, leader_ref, follower_ref, commands,
                        safety_decision, plan, health_latest, link_quality_latest
                    )
                    self._last_processed_seq = snapshot.seq

                time.sleep(0.01)

        except Exception as exc:
            logger.exception("Run loop failed")
            self._record_error_event(
                definition=MissionErrors.Runtime.RUN_LOOP_EXCEPTION,
                message="主循环异常",
                exception=exc,
            )
            self._emergency_land(
                trigger_error=MissionErrors.Runtime.RUN_LOOP_EXCEPTION,
            )

    def shutdown(self):
        """关闭"""
        logger.info("=== 关闭系统 ===")
        self._running = False
        self._graceful_shutdown_land()
        self._flush_terminal_telemetry(
            safety_action="SHUTDOWN",
            safety_reasons=["manual_shutdown"],
            safety_reason_codes=["MANUAL_SHUTDOWN"],
            scheduler_reason="shutdown",
            scheduler_diagnostics={"shutdown": True},
        )
        if "telemetry" in self.comp:
            self.comp["telemetry"].record_event("shutdown", ok=True)
        if "pose_source" in self.comp:
            self.comp["pose_source"].stop()
        console_tap = self.comp.get("console_tap")
        if console_tap is not None:
            try:
                console_tap.stop()
            except Exception:
                logger.exception("Console tap stop failed")
        if self.comp.get("manual_input") is not None:
            self.comp["manual_input"].stop()
        if "telemetry" in self.comp:
            self.comp["telemetry"].close()
        if "link_manager" in self.comp:
            self.comp["link_manager"].close_all()
        group_pool = self.comp.get("group_executor_pool")
        if group_pool is not None:
            group_pool.shutdown(wait=True)

    def _graceful_shutdown_land(self) -> None:
        self.landing_flow.graceful_shutdown_land()

    def _orderly_land(
        self,
        *,
        reason_event: str,
        safety_action: str,
        safety_reasons: list[str],
        safety_reason_codes: list[str],
        scheduler_reason: str,
        scheduler_diagnostics: dict,
        trajectory_terminal_reason: str,
    ) -> None:
        self.landing_flow.orderly_land(
            reason_event=reason_event,
            safety_action=safety_action,
            safety_reasons=safety_reasons,
            safety_reason_codes=safety_reason_codes,
            scheduler_reason=scheduler_reason,
            scheduler_diagnostics=scheduler_diagnostics,
            trajectory_terminal_reason=trajectory_terminal_reason,
        )

    def _flush_terminal_telemetry(
        self,
        *,
        safety_action: str,
        safety_reasons: list[str],
        safety_reason_codes: list[str],
        scheduler_reason: str,
        scheduler_diagnostics: dict,
    ) -> None:
        self.landing_flow.flush_terminal_telemetry(
            safety_action=safety_action,
            safety_reasons=safety_reasons,
            safety_reason_codes=safety_reason_codes,
            scheduler_reason=scheduler_reason,
            scheduler_diagnostics=scheduler_diagnostics,
        )

    def _initialize_manual_mode(self, snapshot) -> None:
        manual_source = self.comp.get("leader_ref_gen")
        if manual_source is None or not hasattr(
            manual_source, "initialize_from_measured_leaders"
        ):
            return

        if snapshot is None:
            return

        leader_positions = {}
        for drone_id in self.comp["fleet"].leader_ids():
            idx = self.comp["fleet"].id_to_index(drone_id)
            leader_positions[drone_id] = np.array(snapshot.positions[idx], dtype=float)
        manual_source.initialize_from_measured_leaders(leader_positions)
        self.comp["telemetry"].record_event("manual_mode_armed", ok=True)

    def _poll_manual_input(self) -> None:
        manual_input = self.comp.get("manual_input")
        manual_state = self.comp.get("manual_leader_state")
        if manual_input is None or manual_state is None:
            return

        intent = manual_input.poll()
        if intent is None:
            return

        previous_axis = manual_state.snapshot().selected_axis
        manual_state.apply_intent(intent)
        self.comp["telemetry"].record_event(
            "manual_intent",
            translation_delta=list(intent.translation_delta),
            scale_delta=intent.scale_delta,
            rotation_delta_deg=intent.rotation_delta_deg,
            axis_switch=intent.axis_switch,
            target_switch=intent.target_switch,
        )
        current_axis = manual_state.snapshot().selected_axis
        if intent.axis_switch and current_axis != previous_axis:
            self.comp["telemetry"].record_event("manual_axis_switch", axis=current_axis)
        snapshot = manual_state.snapshot()
        if intent.target_switch:
            self.comp["telemetry"].record_event(
                "manual_target_switch",
                target_mode=snapshot.target_mode,
                leader_id=snapshot.selected_leader_id,
            )

    def _manual_axis(self) -> str | None:
        manual_state = self.comp.get("manual_leader_state")
        if manual_state is None:
            return None
        return manual_state.snapshot().selected_axis

    def _set_trajectory_state(
        self, state: str, terminal_reason: str | None = None
    ) -> None:
        self._trajectory_state = state
        if terminal_reason is not None:
            self._trajectory_terminal_reason = terminal_reason
        self.comp["telemetry"].record_event(
            "trajectory_state",
            state=state,
            terminal_reason=self._trajectory_terminal_reason,
        )

    def _phase_label(self, t_elapsed: float) -> str | None:
        mission_profile = self.comp.get("mission_profile")
        if mission_profile is None:
            return None
        return mission_profile.phase_at(t_elapsed).name

    def _measured_positions(self, snapshot) -> dict[int, list[float]]:
        fleet = self.comp["fleet"]
        return {
            drone_id: snapshot.positions[fleet.id_to_index(drone_id)].tolist()
            for drone_id in fleet.all_ids()
        }

    @staticmethod
    def _leader_reference_positions(leader_ref) -> dict[int, list[float]]:
        positions = getattr(leader_ref, "positions", {}) or {}
        result: dict[int, list[float]] = {}
        for drone_id, position in positions.items():
            if isinstance(position, np.ndarray):
                result[int(drone_id)] = position.tolist()
            else:
                result[int(drone_id)] = [float(v) for v in position]
        return result

    @staticmethod
    def _follower_reference_positions(follower_ref) -> dict[int, list[float]]:
        if follower_ref is None or not follower_ref.valid:
            return {}
        result: dict[int, list[float]] = {}
        for drone_id, position in follower_ref.target_positions.items():
            if isinstance(position, np.ndarray):
                result[int(drone_id)] = position.tolist()
            else:
                result[int(drone_id)] = [float(v) for v in position]
        return result

    def _manual_initial_structure_reference(self):
        manual_source = self.comp.get("leader_ref_gen")
        if manual_source is None or not hasattr(
            manual_source, "initial_structure_reference"
        ):
            return None
        return manual_source.initial_structure_reference(0.0)

    def _trajectory_entry_start_positions(self, leader_ref) -> dict[int, list[float]]:
        per_leader = (leader_ref.trajectory or {}).get("per_leader", {})
        positions = {}
        for drone_id, spec in per_leader.items():
            pieces = spec.get("pieces") or []
            if not pieces and spec.get("nominal_position") is not None:
                positions[int(drone_id)] = (
                    np.asarray(spec["nominal_position"], dtype=float)
                    .round(9)
                    .tolist()
                )
                continue
            positions[int(drone_id)] = (
                _evaluate_trajectory_spec(spec, 0.0).round(9).tolist()
            )
        return positions

    def _leader_trajectory_start_settings(self) -> tuple[float, float, int]:
        safety_cfg = self.comp["config"].safety
        verify_delay_s = float(
            getattr(safety_cfg, "leader_trajectory_start_verify_delay_s", 3.0)
        )
        min_displacement_m = float(
            getattr(safety_cfg, "leader_trajectory_start_min_displacement_m", 0.06)
        )
        max_retries = int(
            getattr(safety_cfg, "leader_trajectory_start_max_retries", 1)
        )
        return max(verify_delay_s, 0.0), max(min_displacement_m, 0.0), max_retries

    def _snapshot_leader_positions(self, snapshot, leader_ids: list[int]) -> dict[int, np.ndarray]:
        fleet = self.comp["fleet"]
        return {
            int(drone_id): np.asarray(
                snapshot.positions[fleet.id_to_index(drone_id)], dtype=float
            )
            for drone_id in leader_ids
        }

    def _planned_leader_position_from_ref(self, leader_ref, drone_id: int, mission_elapsed: float) -> np.ndarray | None:
        trajectory = leader_ref.trajectory or {}
        per_leader = trajectory.get("per_leader", {}) or {}
        spec = per_leader.get(int(drone_id))
        if not spec:
            return None
        start_time_fn = getattr(self.comp.get("mission_profile"), "trajectory_start_time", None)
        t0 = float(start_time_fn()) if callable(start_time_fn) else 0.0
        local_t = max(0.0, float(mission_elapsed) - t0)
        return _evaluate_trajectory_spec(spec, local_t)

    def _leader_pose_compensated_snapshot(self, snapshot, leader_ref, mission_elapsed: float):
        safety_cfg = self.comp["config"].safety
        if (
            not getattr(safety_cfg, "leader_pose_planned_fallback_enabled", False)
            or not self._trajectory_started
            or getattr(leader_ref, "mode", None) != "trajectory"
        ):
            return snapshot, []
        timestamps = getattr(snapshot, "pose_timestamps", None)
        if timestamps is None:
            return snapshot, []

        stale_after_s = float(
            getattr(safety_cfg, "leader_pose_planned_fallback_stale_after_s", 0.18)
        )
        max_age_s = float(
            getattr(safety_cfg, "leader_pose_planned_fallback_max_age_s", 0.8)
        )
        fallback_ids = [
            int(drone_id)
            for drone_id in getattr(
                safety_cfg, "leader_pose_planned_fallback_ids", []
            )
        ]
        now = time.time()
        positions = np.array(snapshot.positions, dtype=float).copy()
        fresh_mask = np.array(snapshot.fresh_mask, dtype=bool).copy()
        disconnected_ids = set(int(drone_id) for drone_id in snapshot.disconnected_ids)
        applied = []
        fleet = self.comp["fleet"]
        for drone_id in fallback_ids:
            idx = fleet.id_to_index(drone_id)
            ts = float(timestamps[idx]) if np.isfinite(timestamps[idx]) else None
            if ts is None:
                continue
            age_s = now - ts
            if age_s <= stale_after_s or age_s > max_age_s:
                continue
            planned = self._planned_leader_position_from_ref(
                leader_ref, drone_id, mission_elapsed
            )
            if planned is None:
                continue
            measured = positions[idx].copy()
            blend = float(
                np.clip(
                    (age_s - stale_after_s) / max(max_age_s - stale_after_s, 1e-9),
                    0.0,
                    1.0,
                )
            )
            compensated_position = (1.0 - blend) * measured + blend * np.asarray(
                planned, dtype=float
            )
            positions[idx] = compensated_position
            fresh_mask[idx] = True
            disconnected_ids.discard(drone_id)
            applied.append(
                {
                    "drone_id": int(drone_id),
                    "age_s": age_s,
                    "blend": blend,
                    "measured": measured.tolist(),
                    "planned": np.asarray(planned, dtype=float).tolist(),
                    "compensated": compensated_position.tolist(),
                    "delta_m": float(np.linalg.norm(np.asarray(planned) - measured)),
                    "applied_delta_m": float(
                        np.linalg.norm(compensated_position - measured)
                    ),
                }
            )

        if not applied:
            return snapshot, []

        compensated = PoseSnapshot(
            seq=snapshot.seq,
            t_meas=snapshot.t_meas,
            positions=positions,
            fresh_mask=fresh_mask,
            disconnected_ids=sorted(disconnected_ids),
            velocities=snapshot.velocities,
            velocity_fresh_mask=snapshot.velocity_fresh_mask,
            pose_timestamps=snapshot.pose_timestamps,
        )
        self.comp["telemetry"].record_event(
            "leader_pose_planned_fallback",
            ok=True,
            mission_elapsed=float(mission_elapsed),
            stale_after_s=stale_after_s,
            max_age_s=max_age_s,
            applied=applied,
        )
        return compensated, applied

    def _expected_moving_leader_ids(self, leader_ref, verify_delay_s: float) -> list[int]:
        trajectory = leader_ref.trajectory or {}
        per_leader = trajectory.get("per_leader", {}) or {}
        _, min_displacement_m, _ = self._leader_trajectory_start_settings()
        probe_s = max(float(verify_delay_s), 1.0)
        moving: list[int] = []
        for drone_id, spec in per_leader.items():
            pieces = spec.get("pieces") or []
            if not pieces:
                continue
            start = _evaluate_trajectory_spec(spec, 0.0)
            probe = _evaluate_trajectory_spec(spec, probe_s)
            if float(np.linalg.norm(probe - start)) >= min_displacement_m:
                moving.append(int(drone_id))
        return sorted(moving)

    def _start_leader_trajectory(
        self,
        leader_ref,
        snapshot,
        *,
        mission_elapsed: float,
        reason: str = "initial",
    ) -> dict:
        from ..runtime.command_plan import LeaderAction

        leader_ids = list(leader_ref.leader_ids)
        action = LeaderAction(
            kind="start_trajectory",
            drone_ids=leader_ids,
            payload=leader_ref.trajectory or {},
        )
        results = self.comp["leader_executor"].execute([action])
        result = results[0] if results else {"kind": action.kind, "failures": []}
        failures = result.get("failures") or []
        successes = set(int(drone_id) for drone_id in result.get("successes", []))
        ok = not failures and successes.issuperset(int(drone_id) for drone_id in leader_ids)

        verify_delay_s, _, _ = self._leader_trajectory_start_settings()
        self._trajectory_start_attempts += 1
        self._trajectory_start_elapsed = float(mission_elapsed)
        self._trajectory_start_snapshot_seq = int(getattr(snapshot, "seq", -1))
        self._trajectory_start_snapshot_t_meas = float(getattr(snapshot, "t_meas", 0.0))
        self._trajectory_start_reference_positions = self._snapshot_leader_positions(
            snapshot, leader_ids
        )
        self._trajectory_start_moving_leader_ids = self._expected_moving_leader_ids(
            leader_ref, verify_delay_s
        )
        self._trajectory_start_confirmed = not self._trajectory_start_moving_leader_ids
        self._trajectory_started = True
        self._set_trajectory_state("running")

        payload = leader_ref.trajectory or {}
        effective_parameters = result.get("parameters", {}) if isinstance(result, dict) else {}
        self.comp["telemetry"].record_event(
            "trajectory_start",
            ok=ok,
            mission_elapsed=float(mission_elapsed),
            phase_label=self._phase_label(mission_elapsed),
            attempt=self._trajectory_start_attempts,
            reason=reason,
            drone_ids=[int(drone_id) for drone_id in leader_ids],
            moving_leader_ids=list(self._trajectory_start_moving_leader_ids),
            trajectory_id=payload.get("trajectory_id"),
            time_scale=payload.get("time_scale"),
            effective_time_scale=effective_parameters.get("time_scale"),
            relative_position=payload.get("relative_position"),
            relative_yaw=payload.get("relative_yaw"),
            reversed=payload.get("reversed"),
            result=result,
        )
        return result

    def _check_leader_trajectory_start_motion(
        self,
        snapshot,
        *,
        mission_elapsed: float,
        leader_ref,
    ) -> str:
        if (
            not self._trajectory_started
            or self._trajectory_start_confirmed
            or self._trajectory_start_elapsed is None
        ):
            return "not_applicable"

        verify_delay_s, min_displacement_m, max_retries = (
            self._leader_trajectory_start_settings()
        )
        snapshot_seq = int(getattr(snapshot, "seq", -1))
        if (
            self._trajectory_start_snapshot_seq is not None
            and snapshot_seq <= self._trajectory_start_snapshot_seq
        ):
            return "pending"
        snapshot_t_meas = float(getattr(snapshot, "t_meas", mission_elapsed))
        if (
            self._trajectory_start_snapshot_t_meas is not None
            and snapshot_t_meas - self._trajectory_start_snapshot_t_meas < verify_delay_s
        ):
            return "pending"

        current_positions = self._snapshot_leader_positions(
            snapshot, list(leader_ref.leader_ids)
        )
        displacements = {
            int(drone_id): float(
                np.linalg.norm(
                    current_positions[int(drone_id)]
                    - self._trajectory_start_reference_positions[int(drone_id)]
                )
            )
            for drone_id in self._trajectory_start_moving_leader_ids
            if int(drone_id) in current_positions
            and int(drone_id) in self._trajectory_start_reference_positions
        }
        moved_ids = sorted(
            drone_id
            for drone_id, displacement in displacements.items()
            if displacement >= min_displacement_m
        )
        moving_ids = list(self._trajectory_start_moving_leader_ids)
        if moving_ids and moved_ids == moving_ids:
            self._trajectory_start_confirmed = True
            self.comp["telemetry"].record_event(
                "leader_trajectory_motion_confirmed",
                ok=True,
                attempt=self._trajectory_start_attempts,
                mission_elapsed=float(mission_elapsed),
                moving_leader_ids=moving_ids,
                displacements=displacements,
                min_displacement_m=min_displacement_m,
            )
            return "confirmed"

        self.comp["telemetry"].record_event(
            "leader_trajectory_motion_unconfirmed",
            ok=False,
            attempt=self._trajectory_start_attempts,
            mission_elapsed=float(mission_elapsed),
            moving_leader_ids=moving_ids,
            moved_leader_ids=moved_ids,
            displacements=displacements,
            min_displacement_m=min_displacement_m,
        )

        if not moved_ids and self._trajectory_start_attempts <= max_retries:
            self.comp["telemetry"].record_event(
                "leader_trajectory_start_retry",
                ok=True,
                next_attempt=self._trajectory_start_attempts + 1,
                mission_elapsed=float(mission_elapsed),
            )
            self._start_leader_trajectory(
                leader_ref,
                snapshot,
                mission_elapsed=mission_elapsed,
                reason="retry_no_motion",
            )
            return "retried"

        self.comp["telemetry"].record_event(
            "leader_trajectory_start_failed",
            ok=False,
            attempt=self._trajectory_start_attempts,
            mission_elapsed=float(mission_elapsed),
            moving_leader_ids=moving_ids,
            moved_leader_ids=moved_ids,
            displacements=displacements,
            reason="partial_motion" if moved_ids else "no_motion_after_retry",
        )
        return "failed"

    def _manual_input_age(self) -> float | None:
        manual_state = self.comp.get("manual_leader_state")
        if manual_state is None:
            return None
        snapshot = manual_state.snapshot()
        if snapshot.last_input_t is None:
            return None
        return time.time() - snapshot.last_input_t

    def _build_config_fingerprint(self) -> dict:
        config_dir = Path(self.comp.get("config_dir", "config"))
        app_config = self.comp.get("config")
        cached_raw_files = getattr(app_config, "raw_files", None) if app_config is not None else None
        raw_files = dict(cached_raw_files) if cached_raw_files else {}
        if not raw_files and config_dir.exists():
            # 没有提前缓存原始文本时兜底再读一次盘
            for path in sorted(config_dir.glob("*.yaml")):
                raw_files[path.name] = path.read_text(encoding="utf-8")
        config_blob = json.dumps(raw_files, ensure_ascii=False, sort_keys=True)
        lighthouse_blob = json.dumps(
            {
                name: raw_files.get(name, "")
                for name in ("comm.yaml", "fleet.yaml", "safety.yaml")
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        mission = app_config.mission if app_config is not None else None
        fleet = self.comp.get("fleet")
        return {
            "config_dir": str(config_dir),
            "repo_root": self.comp.get("repo_root"),
            "startup_mode": self.comp.get("startup_mode"),
            "config_sha256": hashlib.sha256(config_blob.encode("utf-8")).hexdigest(),
            "lighthouse_config_sha256": hashlib.sha256(
                lighthouse_blob.encode("utf-8")
            ).hexdigest(),
            "config_files": sorted(raw_files.keys()),
            "drone_count": len(fleet.all_ids()) if fleet is not None else None,
            "leader_count": len(fleet.leader_ids()) if fleet is not None else None,
            "follower_count": len(fleet.follower_ids()) if fleet is not None else None,
            "mission_duration": mission.duration if mission is not None else None,
            "trajectory_enabled": (
                mission.leader_motion.trajectory_enabled if mission is not None else None
            ),
        }

    def _fleet_meta(self) -> dict:
        fleet = self.comp.get("fleet")
        if fleet is None:
            return {}
        return {
            "drone_count": len(fleet.all_ids()),
            "leader_ids": list(fleet.leader_ids()),
            "follower_ids": list(fleet.follower_ids()),
            "radio_groups": {
                drone_id: fleet.get_radio_group(drone_id)
                for drone_id in fleet.all_ids()
            },
        }

    def _on_pose_update(self, drone_id, pos, timestamp, velocity=None):
        """定位数据回调 - 直接推送到PoseBus"""
        self.comp["pose_bus"].update_agent(drone_id, pos, timestamp, velocity)

    def _on_console_line(self, drone_id: int, line: str) -> None:
        """onboard firmware consolePrintf 行回调 —— 落到 telemetry event。"""
        try:
            self.comp["telemetry"].record_event(
                "onboard_console", drone_id=drone_id, line=line
            )
        except Exception:
            logger.exception("Failed to record onboard_console event")

    @staticmethod
    def _radio_link_quality_payload(
        link_quality_latest: dict,
    ) -> dict[int, dict[str, float | None]]:
        """把 LinkQualityBus 的 snapshot 映射成 telemetry record 可序列化的结构。"""
        payload: dict[int, dict[str, float | None]] = {}
        for drone_id, sample in link_quality_latest.items():
            payload[int(drone_id)] = {
                "link_quality": sample.link_quality,
                "uplink_rssi": sample.uplink_rssi,
                "uplink_rate": sample.uplink_rate,
                "downlink_rate": sample.downlink_rate,
                "uplink_congestion": sample.uplink_congestion,
                "downlink_congestion": sample.downlink_congestion,
                "last_update_t": sample.last_update_t,
            }
        return payload

    def _record_link_state_events(self) -> None:
        link_state_bus = self.comp.get("link_state_bus")
        telemetry = self.telemetry
        if link_state_bus is None or telemetry is None:
            return
        definition = MissionErrors.Runtime.LINK_STATE_CHANGE
        for event in link_state_bus.drain_events():
            state = event.get("state")
            telemetry.record_event(
                "link_state_change",
                ok=(state == "connected"),
                category=definition.category,
                code=definition.code,
                stage=definition.stage,
                drone_id=event.get("drone_id"),
                state=state,
                error=event.get("error"),
                t_wall=event.get("t_wall"),
            )

    def _try_reconnect_on_disconnect(
        self, snapshot, reason_codes: list[str]
    ) -> bool:
        """fast_gate 触发 ABORT 时，若 reconnect 已启用且原因是纯 disconnect，
        给 disconnected_ids 一次有限次数重连的机会。全部成功返回 True，主循环
        应当 `continue` 跳过本帧 emergency_land；否则返回 False。
        """

        comm = self.comp["config"].comm
        if not getattr(comm, "reconnect_enabled", False):
            return False
        if any(code == "OUT_OF_BOUNDS" for code in reason_codes):
            return False
        drone_ids = set(snapshot.disconnected_ids)
        link_state_bus = self.comp.get("link_state_bus")
        if link_state_bus is not None:
            try:
                drone_ids.update(link_state_bus.disconnected_ids())
            except Exception:
                logger.exception("Failed to query link_state_bus disconnected ids")
                return False
        drone_ids = list(drone_ids)
        if not drone_ids:
            return False
        return self.failure_policy.attempt_reconnect(drone_ids)

    def _emergency_land(
        self,
        *,
        trigger_error: MissionErrorDefinition | None = None,
    ):
        self.landing_flow.emergency_land(trigger_error=trigger_error)

    def _enter_hold_mode(
        self,
        *,
        reason: str = "safety",
        follower_ids: list[int] | None = None,
        reason_codes: list[str] | None = None,
        structured_reasons: list[object] | None = None,
    ):
        self.failure_policy.enter_hold_mode(
            reason=reason,
            follower_ids=follower_ids,
            reason_codes=reason_codes,
            structured_reasons=structured_reasons,
        )

    def _check_hold_timeout(self, mission_elapsed: float) -> bool:
        return self.failure_policy.check_hold_timeout(mission_elapsed)

    def _clear_hold_tracking(self) -> None:
        self.failure_policy.clear_hold_tracking()

    def _reset_follower_controller_full_state(
        self, follower_ids: list[int] | set[int] | None = None
    ) -> None:
        reset_fn = getattr(
            self.comp.get("follower_controller"), "reset_full_state_state", None
        )
        if callable(reset_fn):
            reset_fn(follower_ids)

    def _commit_follower_controller_full_state(
        self,
        commands,
        follower_ids: list[int] | set[int] | None = None,
    ) -> None:
        if commands is None:
            return
        commit_fn = getattr(
            self.comp.get("follower_controller"), "commit_full_state_state", None
        )
        if callable(commit_fn):
            commit_fn(commands, follower_ids)

    @staticmethod
    def _leader_takeoff_action(leader_ids: list[int]):
        from ..runtime.command_plan import LeaderAction

        return LeaderAction(
            kind="takeoff",
            drone_ids=leader_ids,
            payload={"height": 0.5, "duration": 2.0},
        )

    @staticmethod
    def _leader_land_action(leader_ids: list[int]):
        from ..runtime.command_plan import LeaderAction

        return LeaderAction(
            kind="land",
            drone_ids=leader_ids,
            payload={"duration": 4.0},
        )
