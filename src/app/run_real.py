"""真机主循环"""

import hashlib
import json
import time
import logging
from datetime import datetime
from pathlib import Path
import numpy as np
from ..runtime.mission_fsm import MissionState
from ..runtime.telemetry import TelemetryRecord
from ..runtime.follower_controller import FollowerCommandSet
from .mission_errors import MissionErrorDefinition, MissionErrors
from ..adapters.cflib_command_transport import (
    POLY4D_RAW_PIECE_BYTES,
    TRAJECTORY_MEMORY_BYTES,
)
from ..runtime.offline_swarm_sampler import _evaluate_trajectory_spec

logger = logging.getLogger(__name__)


VELOCITY_STREAM_WATCHDOG_FACTOR = 3.0


class RealMissionApp:
    """真机任务应用"""

    def __init__(self, components: dict):
        self.comp = components
        self._running = False
        self._last_processed_seq = -1
        self._trajectory_started = False
        self._shutdown_flushed = False
        self._terminal_land_executed = False
        self._trajectory_state = "inactive"
        self._trajectory_terminal_reason = None
        self._hold_entered_at: float | None = None
        self._watchdog_degraded_followers: set[int] = set()
        self._readiness_report = {
            "wait_for_params": {},
            "reset_estimator": {},
            "trajectory_prepare": {},
            "pose_ready": False,
        }
        self._config_fingerprint = self._build_config_fingerprint()

    def _radio_group_summary(self, drone_ids: list[int]) -> dict[int, dict[str, list[int]]]:
        fleet = self.comp.get("fleet")
        if fleet is None:
            return {}
        groups: dict[int, dict[str, list[int]]] = {}
        for drone_id in drone_ids:
            group_id = fleet.get_radio_group(drone_id)
            entry = groups.setdefault(group_id, {"drone_ids": []})
            entry["drone_ids"].append(drone_id)
        return groups

    def _radio_group_item_summary(
        self,
        items: list[dict],
        *,
        drone_key: str = "drone_id",
        item_key: str = "items",
    ) -> dict[int, dict[str, list[object]]]:
        fleet = self.comp.get("fleet")
        if fleet is None:
            return {}
        groups: dict[int, dict[str, list[object]]] = {}
        for item in items:
            drone_id = item.get(drone_key)
            if drone_id is None:
                continue
            group_id = fleet.get_radio_group(drone_id)
            entry = groups.setdefault(group_id, {"drone_ids": [], item_key: []})
            entry["drone_ids"].append(drone_id)
            entry[item_key].append(item)
        return groups

    def _record_error_event(
        self,
        *,
        definition: MissionErrorDefinition,
        message: str,
        exception: Exception | None = None,
        **details,
    ) -> None:
        telemetry = self.comp.get("telemetry")
        if telemetry is None:
            return

        payload = {
            "ok": False,
            "category": definition.category,
            "code": definition.code,
            "stage": definition.stage,
            "message": message,
            "mission_state": (
                self.comp["fsm"].state().value if "fsm" in self.comp else None
            ),
        }
        if exception is not None:
            payload["exception_type"] = type(exception).__name__
            payload["exception_message"] = str(exception)
        payload.update(details)
        telemetry.record_event("mission_error", **payload)

    def _record_executor_summary(self, event_name: str, results: list[dict]) -> None:
        telemetry = self.comp.get("telemetry")
        fleet = self.comp.get("fleet")
        if telemetry is None or fleet is None:
            return

        total_successes = []
        total_failures = []
        groups: dict[int, dict[str, list[object]]] = {}
        for result in results:
            for drone_id in result.get("successes", []):
                total_successes.append(drone_id)
                group_id = fleet.get_radio_group(drone_id)
                group_entry = groups.setdefault(
                    group_id,
                    {"drone_ids": [], "successes": [], "failures": []},
                )
                group_entry["drone_ids"].append(drone_id)
                group_entry["successes"].append(drone_id)
            for failure in result.get("failures", []):
                total_failures.append(failure)
                drone_id = failure.get("drone_id")
                if drone_id is None:
                    continue
                group_id = fleet.get_radio_group(drone_id)
                group_entry = groups.setdefault(
                    group_id,
                    {"drone_ids": [], "successes": [], "failures": []},
                )
                group_entry["drone_ids"].append(drone_id)
                group_entry["failures"].append(failure)

        telemetry.record_event(
            event_name,
            ok=not total_failures,
            successes=total_successes,
            failures=total_failures,
            radio_groups=groups,
            batch_count=len(results),
        )

    def _check_velocity_stream_watchdog(self, snapshot_t_meas: float) -> list[dict]:
        transport = self.comp.get("transport")
        telemetry = self.comp.get("telemetry")
        fleet = self.comp.get("fleet")
        config = self.comp.get("config")
        if transport is None or telemetry is None or fleet is None or config is None:
            return []

        follower_interval = 1.0 / config.comm.follower_tx_freq
        watchdog_timeout = follower_interval * VELOCITY_STREAM_WATCHDOG_FACTOR
        stale_followers = []
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
            stale_group_summary = self._radio_group_item_summary(
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
                int(item["drone_id"]) for item in stale_followers if "drone_id" in item
            )
            if action == "hold":
                self._enter_hold_mode(reason="watchdog", follower_ids=stale_ids)
            elif action == "degrade":
                self._apply_watchdog_degrade(stale_followers)

        return stale_followers

    def _apply_watchdog_degrade(self, stale_followers: list[dict]) -> None:
        stale_ids = sorted(
            int(item["drone_id"]) for item in stale_followers if "drone_id" in item
        )
        if not stale_ids:
            return

        newly_degraded = [
            drone_id
            for drone_id in stale_ids
            if drone_id not in self._watchdog_degraded_followers
        ]
        self._watchdog_degraded_followers.update(stale_ids)

        self.comp["telemetry"].record_event(
            "watchdog_degrade",
            ok=True,
            category=MissionErrors.Runtime.WATCHDOG_DEGRADE.category,
            code=MissionErrors.Runtime.WATCHDOG_DEGRADE.code,
            stage=MissionErrors.Runtime.WATCHDOG_DEGRADE.stage,
            follower_ids=stale_ids,
            radio_groups=self._radio_group_summary(stale_ids),
            newly_degraded=newly_degraded,
        )

    def _clear_watchdog_degrade(
        self, *, active_commands: dict[int, object] | None = None
    ) -> None:
        if not self._watchdog_degraded_followers:
            return

        active_ids = set(active_commands.keys()) if active_commands else set()
        recovered = sorted(self._watchdog_degraded_followers.intersection(active_ids))
        if not recovered:
            return

        self._watchdog_degraded_followers.difference_update(recovered)
        self.comp["telemetry"].record_event(
            "watchdog_degrade_recovered",
            ok=True,
            category=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.category,
            code=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.code,
            stage=MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.stage,
            follower_ids=recovered,
            radio_groups=self._radio_group_summary(recovered),
            remaining=sorted(self._watchdog_degraded_followers),
        )

    @staticmethod
    def _split_degraded_commands(
        commands, degraded_follower_ids: set[int]
    ) -> tuple[dict[int, object], dict[int, object]]:
        if commands is None:
            return {}, {}

        active_commands = {}
        degraded_commands = {}
        for drone_id, command in commands.commands.items():
            if drone_id in degraded_follower_ids:
                degraded_commands[drone_id] = command
            else:
                active_commands[drone_id] = command
        return active_commands, degraded_commands

    def _safe_transition(self, target: MissionState) -> bool:
        try:
            self.comp["fsm"].transition(target)
            if "telemetry" in self.comp:
                self.comp["telemetry"].record_event(
                    "fsm_transition", target=target.value
                )
            return True
        except ValueError as exc:
            logger.error(f"FSM transition failed: {exc}")
            self.comp["fsm"].force_abort()
            return False

    def _fail_start(
        self,
        reason: str,
        *,
        definition: MissionErrorDefinition = MissionErrors.Readiness.STARTUP_FAILED,
        exception: Exception | None = None,
        **details,
    ) -> bool:
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
        return False

    def start(self):
        """启动"""
        logger.info("=== 启动真机任务 ===")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        telemetry_path = Path("telemetry") / f"run_real_{timestamp}.jsonl"
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self.comp["telemetry_path"] = str(telemetry_path)
        self.comp["telemetry"].open(str(telemetry_path))
        self.comp["telemetry"].record_event(
            "startup_mode", mode=self.comp.get("startup_mode", "auto")
        )
        self.comp["telemetry"].record_event(
            "config_fingerprint",
            ok=True,
            fingerprint=self._config_fingerprint,
        )

        # 连接
        if not self._safe_transition(MissionState.CONNECT):
            return self._fail_start(
                "FSM failed before connect",
                definition=MissionErrors.Readiness.FSM_CONNECT_TRANSITION_FAILED,
            )
        try:
            self.comp["link_manager"].connect_all()
            self.comp["telemetry"].record_event("connect_all", ok=True)
        except Exception as exc:
            self.comp["telemetry"].record_event(
                "connect_all", ok=False, error=str(exc)
            )
            return self._fail_start(
                "连接失败",
                definition=MissionErrors.Connection.CONNECT_ALL_FAILED,
                exception=exc,
            )

        if self.comp["config"].comm.readiness_wait_for_params:
            drone_id = None
            try:
                for drone_id in self.comp["fleet"].all_ids():
                    self.comp["transport"].wait_for_params(drone_id)
                    self._readiness_report["wait_for_params"][drone_id] = True
                    self.comp["telemetry"].record_event(
                        "wait_for_params", drone_id=drone_id, ok=True
                    )
            except Exception as exc:
                return self._fail_start(
                    "参数同步失败",
                    definition=MissionErrors.Readiness.WAIT_FOR_PARAMS_FAILED,
                    exception=exc,
                    drone_id=drone_id,
                )

        if self.comp["config"].comm.readiness_reset_estimator:
            drone_id = None
            try:
                for drone_id in self.comp["fleet"].all_ids():
                    self.comp["transport"].reset_estimator_and_wait(drone_id)
                    self._readiness_report["reset_estimator"][drone_id] = True
                    self.comp["telemetry"].record_event(
                        "reset_estimator", drone_id=drone_id, ok=True
                    )
            except Exception as exc:
                return self._fail_start(
                    "重置估计器失败",
                    definition=MissionErrors.Readiness.RESET_ESTIMATOR_FAILED,
                    exception=exc,
                    drone_id=drone_id,
                )

        # 启动定位
        try:
            self.comp["pose_source"].register_callback(self._on_pose_update)
            self.comp["pose_source"].start()
        except Exception as exc:
            return self._fail_start(
                "定位源启动失败",
                definition=MissionErrors.Readiness.POSE_SOURCE_START_FAILED,
                exception=exc,
            )

        # 等待定位稳定
        logger.info("等待定位数据...")
        for _ in range(20):
            snapshot = self.comp["pose_bus"].latest()
            if snapshot and all(snapshot.fresh_mask):
                logger.info("所有无人机定位就绪")
                self._readiness_report["pose_ready"] = True
                self.comp["telemetry"].record_event("pose_ready", ok=True)
                break
            time.sleep(0.1)
        else:
            self.comp["telemetry"].record_event("pose_ready", ok=False)
            return self._fail_start(
                "部分无人机定位未就绪，中止任务",
                definition=MissionErrors.Readiness.POSE_TIMEOUT,
            )

        logger.info("等待健康数据...")
        for _ in range(30):
            health_samples = self.comp["health_bus"].latest()
            if all(
                drone_id in health_samples
                and "pm.vbat" in health_samples[drone_id].values
                for drone_id in self.comp["fleet"].all_ids()
            ):
                self._readiness_report["health_ready"] = True
                self.comp["telemetry"].record_event("health_ready", ok=True)
                break
            time.sleep(0.1)
        else:
            self.comp["telemetry"].record_event("health_ready", ok=False)
            return self._fail_start(
                "健康状态数据未就绪，中止任务",
                definition=MissionErrors.Readiness.HEALTH_TIMEOUT,
            )

        leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if (
            self.comp.get("startup_mode", "auto") == "auto"
            and leader_ref.mode == "trajectory"
            and leader_ref.trajectory is not None
        ):
            per_leader = leader_ref.trajectory.get("per_leader", {})
            drone_id = None
            try:
                for drone_id in self.comp["fleet"].leader_ids():
                    spec = per_leader.get(drone_id, {})
                    pieces = spec.get("pieces", [])
                    start_addr = spec.get("start_addr", 0)
                    trajectory_id = spec.get("trajectory_id", 1)
                    estimated_bytes = len(pieces) * POLY4D_RAW_PIECE_BYTES
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
                    )
                    piece_count = self.comp["transport"].upload_trajectory(
                        drone_id, pieces, start_addr=start_addr
                    )
                    self.comp["transport"].hl_define_trajectory(
                        drone_id, trajectory_id, start_addr, piece_count
                    )
                    self._readiness_report["trajectory_prepare"][drone_id] = {
                        "uploaded": True,
                        "defined": True,
                        "pieces": piece_count,
                        "estimated_bytes": estimated_bytes,
                        "fits_memory": fits_memory,
                        "trajectory_id": trajectory_id,
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
                        nominal_position=spec.get("nominal_position"),
                    )
            except Exception as exc:
                return self._fail_start(
                    "轨迹准备失败",
                    definition=MissionErrors.Readiness.TRAJECTORY_PREPARE_FAILED,
                    exception=exc,
                    drone_id=drone_id,
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
            return self._fail_start(
                "FSM failed entering POSE_READY",
                definition=MissionErrors.Readiness.FSM_POSE_READY_TRANSITION_FAILED,
            )

        if not self._safe_transition(MissionState.PREFLIGHT):
            return self._fail_start(
                "FSM failed entering PREFLIGHT",
                definition=MissionErrors.Readiness.FSM_PREFLIGHT_TRANSITION_FAILED,
            )
        self.comp["readiness_report"] = self._readiness_report
        try:
            preflight_report = self.comp["preflight"].run()
        except Exception as exc:
            return self._fail_start(
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
            return self._fail_start(
                f"Preflight failed: {preflight_report.reasons} codes={preflight_report.failed_codes}",
                definition=MissionErrors.Readiness.PREFLIGHT_FAILED,
                failed_codes=preflight_report.failed_codes,
            )

        # Takeoff - 所有无人机起飞
        logger.info("=== 起飞 ===")
        if not self._safe_transition(MissionState.TAKEOFF):
            return self._fail_start(
                "FSM failed entering TAKEOFF",
                definition=MissionErrors.Readiness.FSM_TAKEOFF_TRANSITION_FAILED,
            )

        self.comp["leader_executor"].execute(
            [self._leader_takeoff_action(self.comp["fleet"].leader_ids())]
        )
        follower_takeoff_result = self.comp["follower_executor"].takeoff(
            self.comp["fleet"].follower_ids(), height=0.5, duration=2.0
        )
        self._record_executor_summary("follower_takeoff_execution", [follower_takeoff_result])

        time.sleep(3.0)

        # Settle - 等待稳定并验证
        if not self._safe_transition(MissionState.SETTLE):
            return self._fail_start(
                "FSM failed entering SETTLE",
                definition=MissionErrors.Readiness.FSM_SETTLE_TRANSITION_FAILED,
            )
        logger.info("等待稳定...")
        time.sleep(2.0)

        # 起飞后先对齐到初始编队，再进入 RUN
        initial_leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if self.comp.get("startup_mode", "auto") == "manual_leader":
            initial_leader_ref = self._manual_initial_structure_reference()
        if (
            self.comp.get("startup_mode", "auto") == "auto"
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
        elif (
            self.comp.get("startup_mode", "auto") == "manual_leader"
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
        elif (
            self.comp.get("startup_mode", "auto") == "auto"
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

        # 验证所有无人机在空中
        snapshot = self.comp["pose_bus"].latest()
        if snapshot:
            for drone_id in self.comp["fleet"].all_ids():
                idx = self.comp["fleet"].id_to_index(drone_id)
                if snapshot.positions[idx][2] < 0.3:
                    self.comp["telemetry"].record_event(
                        "takeoff_validation", ok=False, drone_id=drone_id
                    )
                    self._record_error_event(
                        definition=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                        message="起飞后高度验证失败",
                        drone_id=drone_id,
                        altitude=float(snapshot.positions[idx][2]),
                    )
                    self._emergency_land(
                        trigger_error=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                    )
                    return False

        if self.comp.get("startup_mode", "auto") == "manual_leader":
            self._initialize_manual_mode(snapshot)

        logger.info("系统就绪")
        self.comp["telemetry"].record_event("startup_complete", ok=True)
        return True

    def run(self):
        """主循环"""
        # 检查start是否成功
        if self.comp["fsm"].state() != MissionState.SETTLE:
            logger.error("系统未就绪，无法进入RUN")
            return

        self._running = True
        if not self._safe_transition(MissionState.RUN):
            return
        logger.info("=== 进入主循环 ===")
        self.comp["telemetry"].record_event("run_entered", ok=True)

        mission_start_time = time.time()
        manual_input = self.comp.get("manual_input")
        try:
            if manual_input is not None:
                manual_input.start()
                manual_cfg = self.comp["config"].startup.manual
                self.comp["telemetry"].record_event(
                    "manual_input_started",
                    ok=True,
                    axis=(manual_cfg.default_axis if manual_cfg is not None else None),
                )

            while self._running:
                # 1. 获取最新pose
                snapshot = self.comp["pose_bus"].latest()
                if snapshot is None:
                    time.sleep(0.01)
                    continue

                # 2. Pre-safety检查（基于snapshot）
                pre_safety = self.comp["safety"].evaluate(
                    snapshot,
                    frame=None,
                    commands=None,
                    health=self.comp["health_bus"].latest(),
                )

                if pre_safety.action == "ABORT":
                    logger.error(f"Pre-safety ABORT: {pre_safety.reasons}")
                    self._emergency_land()
                    break

                if pre_safety.action == "HOLD":
                    logger.warning(f"Pre-safety HOLD: {pre_safety.reasons}")
                    self._enter_hold_mode()
                    if self._check_hold_timeout(time.time() - mission_start_time):
                        break
                    time.sleep(0.1)
                    continue

                if self.comp["fsm"].state() == MissionState.HOLD:
                    self._safe_transition(MissionState.RUN)
                    self.comp["telemetry"].record_event("hold_recovered", ok=True)
                    self._clear_hold_tracking()

                t_elapsed = time.time() - mission_start_time
                self._poll_manual_input()
                leader_ref = self.comp["leader_ref_gen"].reference_at(t_elapsed)

                if (
                    self.comp.get("startup_mode", "auto") == "auto"
                    and leader_ref.mode == "trajectory"
                    and not self._trajectory_started
                    and self._phase_label(t_elapsed) == "formation_run"
                ):
                    from ..runtime.command_plan import LeaderAction

                    self.comp["leader_executor"].execute(
                        [
                            LeaderAction(
                                kind="start_trajectory",
                                drone_ids=self.comp["fleet"].leader_ids(),
                                payload=leader_ref.trajectory or {},
                            )
                        ]
                    )
                    self._trajectory_started = True
                    self._set_trajectory_state("running")
                    self.comp["telemetry"].record_event(
                        "trajectory_start",
                        ok=True,
                        mission_elapsed=t_elapsed,
                        phase_label=self._phase_label(t_elapsed),
                    )

                if (
                    self.comp.get("startup_mode", "auto") == "auto"
                    and t_elapsed >= self.comp["mission_profile"].total_time()
                ):
                    logger.info(
                        "Mission duration reached (%.2fs), starting orderly landing",
                        t_elapsed,
                    )
                    self.comp["telemetry"].record_event(
                        "mission_complete",
                        ok=True,
                        elapsed=t_elapsed,
                        total_time=self.comp["mission_profile"].total_time(),
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
                    break

                frame = None
                follower_ref = None
                commands = None

                # 只有新pose才进行frame/ref/control计算
                if snapshot.seq > self._last_processed_seq:
                    frame = self.comp["frame_estimator"].estimate(
                        snapshot, self.comp["fleet"].leader_ids()
                    )

                    if frame.valid:
                        follower_ref = self.comp["follower_ref_gen"].compute(
                            frame.leader_positions,
                            snapshot.t_meas,
                        )

                    if follower_ref is not None and follower_ref.valid:
                        commands = self.comp["follower_controller"].compute(
                            snapshot,
                            follower_ref,
                            self.comp["fleet"].follower_ids(),
                            self.comp["fleet"],
                        )

                # 7. Full-safety检查（包含frame、commands和follower_ref）
                safety_decision = self.comp["safety"].evaluate(
                    snapshot,
                    frame,
                    commands,
                    follower_ref,
                    health=self.comp["health_bus"].latest(),
                )

                # 处理安全决策
                if safety_decision.action == "ABORT":
                    logger.error(f"ABORT triggered: {safety_decision.reasons}")
                    self._emergency_land()
                    break

                if safety_decision.action == "HOLD":
                    logger.warning(f"HOLD triggered: {safety_decision.reasons}")
                    self._enter_hold_mode()
                    if self._check_hold_timeout(t_elapsed):
                        break
                    time.sleep(0.1)
                    continue

                if self.comp["fsm"].state() == MissionState.HOLD:
                    self._safe_transition(MissionState.RUN)
                    self.comp["telemetry"].record_event("hold_recovered", ok=True)
                    self._clear_hold_tracking()

                active_commands, _degraded_commands = self._split_degraded_commands(
                    commands, self._watchdog_degraded_followers
                )
                parked_follower_ids = sorted(self._watchdog_degraded_followers)
                filtered_commands = (
                    FollowerCommandSet(
                        commands=active_commands,
                        diagnostics=commands.diagnostics,
                    )
                    if commands is not None
                    else None
                )

                # 8. 调度器生成发送计划
                plan = self.comp["scheduler"].plan(
                    snapshot,
                    self.comp["fsm"].state(),
                    leader_ref,
                    filtered_commands,
                    safety_decision,
                    parked_follower_ids=parked_follower_ids,
                )

                # 9. 执行
                if plan.leader_actions:
                    leader_results = self.comp["leader_executor"].execute(plan.leader_actions)
                    self._record_executor_summary("leader_execution", leader_results)
                if plan.follower_actions:
                    follower_velocity_result = self.comp["follower_executor"].execute_velocity(
                        plan.follower_actions
                    )
                    self._record_executor_summary(
                        "follower_velocity_execution",
                        [follower_velocity_result],
                    )
                    success_ids = set(follower_velocity_result.get("successes", []))
                    self._clear_watchdog_degrade(active_commands={drone_id: None for drone_id in success_ids})
                if plan.hold_actions:
                    follower_hold_result = self.comp["follower_executor"].execute_hold(plan.hold_actions)
                    self._record_executor_summary(
                        "follower_hold_execution",
                        [follower_hold_result],
                    )

                self._check_velocity_stream_watchdog(snapshot.t_meas)

                if snapshot.seq > self._last_processed_seq:
                    self._last_processed_seq = snapshot.seq

                follower_command_norms = {}
                if commands is not None:
                    follower_command_norms = {
                        drone_id: float((cmd**2).sum() ** 0.5)
                        for drone_id, cmd in commands.commands.items()
                    }

                measured_positions = self._measured_positions(snapshot)
                leader_reference_positions = self._leader_reference_positions(leader_ref)
                follower_reference_positions = self._follower_reference_positions(
                    follower_ref
                )
                phase_label = self._phase_label(t_elapsed)
                leader_mode = getattr(leader_ref, "mode", None)

                self.comp["telemetry"].log(
                    TelemetryRecord(
                        t_wall=time.time(),
                        mission_state=self.comp["fsm"].state().value,
                        startup_mode=self.comp.get("startup_mode"),
                        mission_elapsed=t_elapsed,
                        trajectory_state=self._trajectory_state,
                        trajectory_terminal_reason=self._trajectory_terminal_reason,
                        readiness=self._readiness_report,
                        config_fingerprint=self._config_fingerprint,
                        phase_events=self.comp["telemetry"].phase_events(),
                        snapshot_seq=snapshot.seq,
                        snapshot_t_meas=snapshot.t_meas,
                        measured_positions=measured_positions,
                        fresh_mask={
                            drone_id: bool(
                                snapshot.fresh_mask[
                                    self.comp["fleet"].id_to_index(drone_id)
                                ]
                            )
                            for drone_id in self.comp["fleet"].all_ids()
                        },
                        disconnected_ids=list(snapshot.disconnected_ids),
                        health={
                            drone_id: sample.values
                            for drone_id, sample in self.comp["health_bus"].latest().items()
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
                        leader_reference_source=(
                            type(self.comp["leader_ref_gen"]).__name__
                            if "leader_ref_gen" in self.comp
                            else None
                        ),
                        manual_axis=self._manual_axis(),
                        manual_input_age=self._manual_input_age(),
                        leader_action_count=len(plan.leader_actions),
                        follower_action_count=len(plan.follower_actions),
                        follower_command_norms=follower_command_norms,
                    )
                )

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
        if self.comp.get("manual_input") is not None:
            self.comp["manual_input"].stop()
        if "telemetry" in self.comp:
            self.comp["telemetry"].close()
        if "link_manager" in self.comp:
            self.comp["link_manager"].close_all()

    def _graceful_shutdown_land(self) -> None:
        state = self.comp["fsm"].state()
        if state in {MissionState.INIT, MissionState.CONNECT, MissionState.ABORT}:
            return

        self._orderly_land(
            reason_event="manual_shutdown_land",
            safety_action="SHUTDOWN",
            safety_reasons=["manual_shutdown"],
            safety_reason_codes=["MANUAL_SHUTDOWN"],
            scheduler_reason="shutdown",
            scheduler_diagnostics={"shutdown": True},
            trajectory_terminal_reason="shutdown",
        )

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
        if self._terminal_land_executed:
            self._set_trajectory_state("terminated", trajectory_terminal_reason)
            self._flush_terminal_telemetry(
                safety_action=safety_action,
                safety_reasons=safety_reasons,
                safety_reason_codes=safety_reason_codes,
                scheduler_reason=scheduler_reason,
                scheduler_diagnostics=scheduler_diagnostics,
            )
            return

        try:
            if self.comp["fsm"].state() != MissionState.LAND:
                self._safe_transition(MissionState.LAND)
        except Exception:
            logger.exception("Failed to enter LAND during shutdown")

        try:
            self.comp["telemetry"].record_event(reason_event, ok=True)
            stop_result = self.comp["follower_executor"].stop_velocity_mode(
                self.comp["fleet"].follower_ids()
            )
            self._record_executor_summary("follower_stop_velocity_execution", [stop_result])
            time.sleep(0.1)
            leader_land_results = self.comp["leader_executor"].execute(
                [self._leader_land_action(self.comp["fleet"].leader_ids())]
            )
            self._record_executor_summary("leader_land_execution", leader_land_results)
            follower_land_result = self.comp["follower_executor"].land(
                self.comp["fleet"].follower_ids(), duration=2.0
            )
            self._record_executor_summary("follower_land_execution", [follower_land_result])
            self._terminal_land_executed = True
            self._set_trajectory_state("terminated", trajectory_terminal_reason)
            time.sleep(3.0)
        except Exception:
            logger.exception("Failed graceful land during shutdown")
            self._set_trajectory_state("failed", trajectory_terminal_reason)

        self._flush_terminal_telemetry(
            safety_action=safety_action,
            safety_reasons=safety_reasons,
            safety_reason_codes=safety_reason_codes,
            scheduler_reason=scheduler_reason,
            scheduler_diagnostics=scheduler_diagnostics,
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
        if self._shutdown_flushed or "telemetry" not in self.comp:
            return
        telemetry = self.comp["telemetry"]
        pose_bus = self.comp.get("pose_bus")
        if pose_bus is None:
            return
        snapshot = pose_bus.latest()
        if snapshot is None:
            return

        t_elapsed = 0.0
        measured_positions = self._measured_positions(snapshot)
        leader_ref = self.comp["leader_ref_gen"].reference_at(t_elapsed)
        telemetry.log(
            TelemetryRecord(
                t_wall=time.time(),
                mission_state=self.comp["fsm"].state().value,
                startup_mode=self.comp.get("startup_mode"),
                mission_elapsed=t_elapsed,
                trajectory_state=self._trajectory_state,
                trajectory_terminal_reason=self._trajectory_terminal_reason,
                readiness=self._readiness_report,
                config_fingerprint=self._config_fingerprint,
                phase_events=telemetry.phase_events(),
                snapshot_seq=snapshot.seq,
                snapshot_t_meas=snapshot.t_meas,
                measured_positions=measured_positions,
                fresh_mask={
                    drone_id: bool(
                        snapshot.fresh_mask[self.comp["fleet"].id_to_index(drone_id)]
                    )
                    for drone_id in self.comp["fleet"].all_ids()
                },
                disconnected_ids=list(snapshot.disconnected_ids),
                health={
                    drone_id: sample.values
                    for drone_id, sample in self.comp["health_bus"].latest().items()
                },
                frame_valid=None,
                frame_condition_number=None,
                phase_label=self._phase_label(t_elapsed),
                leader_mode=getattr(leader_ref, "mode", None),
                leader_reference_positions=self._leader_reference_positions(leader_ref),
                follower_reference_positions={},
                safety_action=safety_action,
                safety_reasons=safety_reasons,
                safety_reason_codes=safety_reason_codes,
                scheduler_reason=scheduler_reason,
                scheduler_diagnostics=scheduler_diagnostics,
                leader_reference_source=(
                    type(self.comp["leader_ref_gen"]).__name__
                    if "leader_ref_gen" in self.comp
                    else None
                ),
                manual_axis=self._manual_axis(),
                manual_input_age=self._manual_input_age(),
                leader_action_count=0,
                follower_action_count=0,
                follower_command_norms={},
            )
        )
        self._shutdown_flushed = True

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
        return {
            drone_id: np.array(
                snapshot.positions[self.comp["fleet"].id_to_index(drone_id)],
                dtype=float,
            )
            .round(9)
            .tolist()
            for drone_id in self.comp["fleet"].all_ids()
        }

    @staticmethod
    def _leader_reference_positions(leader_ref) -> dict[int, list[float]]:
        positions = getattr(leader_ref, "positions", {}) or {}
        return {
            int(drone_id): np.array(position, dtype=float).round(9).tolist()
            for drone_id, position in positions.items()
        }

    @staticmethod
    def _follower_reference_positions(follower_ref) -> dict[int, list[float]]:
        if follower_ref is None or not follower_ref.valid:
            return {}
        return {
            int(drone_id): np.array(position, dtype=float).round(9).tolist()
            for drone_id, position in follower_ref.target_positions.items()
        }

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
            positions[int(drone_id)] = (
                _evaluate_trajectory_spec(spec, 0.0).round(9).tolist()
            )
        return positions

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
        yaml_files = []
        if config_dir.exists():
            yaml_files = sorted(config_dir.glob("*.yaml"))
        config_files = {}
        for path in yaml_files:
            config_files[path.name] = path.read_text(encoding="utf-8")
        config_blob = json.dumps(config_files, ensure_ascii=False, sort_keys=True)
        app_config = self.comp.get("config")
        mission = app_config.mission if app_config is not None else None
        fleet = self.comp.get("fleet")
        return {
            "config_dir": str(config_dir),
            "repo_root": self.comp.get("repo_root"),
            "startup_mode": self.comp.get("startup_mode"),
            "config_sha256": hashlib.sha256(config_blob.encode("utf-8")).hexdigest(),
            "config_files": sorted(config_files.keys()),
            "drone_count": len(fleet.all_ids()) if fleet is not None else None,
            "leader_count": len(fleet.leader_ids()) if fleet is not None else None,
            "follower_count": len(fleet.follower_ids()) if fleet is not None else None,
            "mission_duration": mission.duration if mission is not None else None,
            "trajectory_enabled": (
                mission.leader_motion.trajectory_enabled if mission is not None else None
            ),
        }

    def _on_pose_update(self, drone_id, pos, timestamp):
        """定位数据回调 - 直接推送到PoseBus"""
        self.comp["pose_bus"].update_agent(drone_id, pos, timestamp)

    def _emergency_land(
        self,
        *,
        trigger_error: MissionErrorDefinition | None = None,
    ):
        """紧急降落"""
        logger.error("=== 紧急降落 ===")
        event_details: dict[str, object] = {"ok": True}
        if trigger_error is not None:
            event_details["trigger_code"] = trigger_error.code
            event_details["trigger_category"] = trigger_error.category
            event_details["trigger_stage"] = trigger_error.stage
        self.comp["telemetry"].record_event("emergency_land", **event_details)
        self.comp["fsm"].force_abort()
        self._set_trajectory_state("terminated", "abort")

        # Followers先停velocity，恢复high-level权限
        self.comp["follower_executor"].stop_velocity_mode(
            self.comp["fleet"].follower_ids()
        )

        time.sleep(0.1)

        try:
            self.comp["leader_executor"].execute(
                [self._leader_land_action(self.comp["fleet"].leader_ids())]
            )
            self.comp["follower_executor"].land(
                self.comp["fleet"].follower_ids(), duration=2.0
            )
            self._terminal_land_executed = True
        except Exception as e:
            logger.error(f"Failed to land swarm: {e}")

        self._flush_terminal_telemetry(
            safety_action="ABORT",
            safety_reasons=["emergency_land"],
            safety_reason_codes=["EMERGENCY_LAND"],
            scheduler_reason="emergency_land",
            scheduler_diagnostics={"emergency_land": True},
        )

        time.sleep(3.0)

    def _enter_hold_mode(
        self,
        *,
        reason: str = "safety",
        follower_ids: list[int] | None = None,
    ):
        if self.comp["fsm"].state() != MissionState.HOLD:
            self._safe_transition(MissionState.HOLD)
        if self._hold_entered_at is None:
            self._hold_entered_at = time.time()
            self.comp["telemetry"].record_event(
                "hold_entered",
                ok=True,
                reason=reason,
                category=(
                    MissionErrors.Runtime.WATCHDOG_HOLD.category
                    if reason == "watchdog"
                    else None
                ),
                code=(
                    MissionErrors.Runtime.WATCHDOG_HOLD.code
                    if reason == "watchdog"
                    else None
                ),
                stage=(
                    MissionErrors.Runtime.WATCHDOG_HOLD.stage
                    if reason == "watchdog"
                    else None
                ),
            )
        if self._trajectory_started:
            self._set_trajectory_state("paused")

        from ..runtime.command_plan import HoldAction

        target_ids = follower_ids or self.comp["fleet"].follower_ids()
        hold_actions = [HoldAction(drone_id=fid) for fid in target_ids]
        hold_result = self.comp["follower_executor"].execute_hold(hold_actions)
        self._record_executor_summary("follower_hold_execution", [hold_result])

    def _check_hold_timeout(self, mission_elapsed: float) -> bool:
        if self._hold_entered_at is None:
            return False

        hold_duration = time.time() - self._hold_entered_at
        timeout = self.comp["config"].safety.hold_auto_land_timeout
        if hold_duration < timeout:
            return False

        self.comp["telemetry"].record_event(
            "hold_timeout_land",
            ok=True,
            hold_duration=hold_duration,
            timeout=timeout,
            mission_elapsed=mission_elapsed,
        )
        self._orderly_land(
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

    def _clear_hold_tracking(self) -> None:
        self._hold_entered_at = None

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
            payload={"duration": 2.0},
        )
