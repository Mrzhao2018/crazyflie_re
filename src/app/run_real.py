"""真机主循环"""

import time
import logging
from pathlib import Path
from ..runtime.mission_fsm import MissionState
from ..runtime.telemetry import TelemetryRecord

logger = logging.getLogger(__name__)


class RealMissionApp:
    """真机任务应用"""

    def __init__(self, components: dict):
        self.comp = components
        self._running = False
        self._last_processed_seq = -1
        self._trajectory_started = False
        self._readiness_report = {
            "wait_for_params": {},
            "reset_estimator": {},
            "trajectory_prepare": {},
            "pose_ready": False,
        }

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

    def _fail_start(self, reason: str) -> bool:
        logger.error(reason)
        self.comp["fsm"].force_abort()
        self.shutdown()
        return False

    def start(self):
        """启动"""
        logger.info("=== 启动真机任务 ===")
        telemetry_path = Path("telemetry") / "run_real.jsonl"
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        self.comp["telemetry"].open(str(telemetry_path))

        # 连接
        if not self._safe_transition(MissionState.CONNECT):
            return self._fail_start("FSM failed before connect")
        try:
            self.comp["link_manager"].connect_all()
            self.comp["telemetry"].record_event("connect_all", ok=True)
        except Exception as e:
            self.comp["telemetry"].record_event("connect_all", ok=False, error=str(e))
            return self._fail_start(f"连接失败: {e}")

        if self.comp["config"].comm.readiness_wait_for_params:
            for drone_id in self.comp["fleet"].all_ids():
                self.comp["transport"].wait_for_params(drone_id)
                self._readiness_report["wait_for_params"][drone_id] = True
                self.comp["telemetry"].record_event(
                    "wait_for_params", drone_id=drone_id, ok=True
                )

        if self.comp["config"].comm.readiness_reset_estimator:
            for drone_id in self.comp["fleet"].all_ids():
                self.comp["transport"].reset_estimator_and_wait(drone_id)
                self._readiness_report["reset_estimator"][drone_id] = True
                self.comp["telemetry"].record_event(
                    "reset_estimator", drone_id=drone_id, ok=True
                )

        # 启动定位
        self.comp["pose_source"].register_callback(self._on_pose_update)
        self.comp["pose_source"].start()

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
            return self._fail_start("部分无人机定位未就绪，中止任务")

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
            return self._fail_start("健康状态数据未就绪，中止任务")

        leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if leader_ref.mode == "trajectory" and leader_ref.trajectory is not None:
            per_leader = leader_ref.trajectory.get("per_leader", {})
            for drone_id in self.comp["fleet"].leader_ids():
                spec = per_leader.get(drone_id, {})
                pieces = spec.get("pieces", [])
                start_addr = spec.get("start_addr", 0)
                trajectory_id = spec.get("trajectory_id", 1)
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
                    "trajectory_id": trajectory_id,
                    "nominal_position": spec.get("nominal_position"),
                }
                self.comp["telemetry"].record_event(
                    "trajectory_prepare",
                    drone_id=drone_id,
                    uploaded=True,
                    defined=True,
                    pieces=piece_count,
                    trajectory_id=trajectory_id,
                    nominal_position=spec.get("nominal_position"),
                )

        if not self._safe_transition(MissionState.POSE_READY):
            return self._fail_start("FSM failed entering POSE_READY")

        if not self._safe_transition(MissionState.PREFLIGHT):
            return self._fail_start("FSM failed entering PREFLIGHT")
        self.comp["readiness_report"] = self._readiness_report
        preflight_report = self.comp["preflight"].run()
        self.comp["telemetry"].record_event(
            "preflight",
            ok=preflight_report.ok,
            failed_codes=preflight_report.failed_codes,
        )
        if not preflight_report.ok:
            return self._fail_start(
                f"Preflight failed: {preflight_report.reasons} codes={preflight_report.failed_codes}"
            )

        # Takeoff - 所有无人机起飞
        logger.info("=== 起飞 ===")
        if not self._safe_transition(MissionState.TAKEOFF):
            return self._fail_start("FSM failed entering TAKEOFF")

        self.comp["leader_executor"].execute(
            [self._leader_takeoff_action(self.comp["fleet"].leader_ids())]
        )
        self.comp["follower_executor"].takeoff(
            self.comp["fleet"].follower_ids(), height=0.5, duration=2.0
        )

        time.sleep(3.0)

        # Settle - 等待稳定并验证
        if not self._safe_transition(MissionState.SETTLE):
            return self._fail_start("FSM failed entering SETTLE")
        logger.info("等待稳定...")
        time.sleep(2.0)

        # 起飞后先对齐到初始编队，再进入 RUN
        initial_leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if initial_leader_ref.mode == "batch_goto":
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

        # 验证所有无人机在空中
        snapshot = self.comp["pose_bus"].latest()
        if snapshot:
            for drone_id in self.comp["fleet"].all_ids():
                idx = self.comp["fleet"].id_to_index(drone_id)
                if snapshot.positions[idx][2] < 0.3:
                    self.comp["telemetry"].record_event(
                        "takeoff_validation", ok=False, drone_id=drone_id
                    )
                    self._emergency_land()
                    return False

        leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if leader_ref.mode == "trajectory" and not self._trajectory_started:
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
            self.comp["telemetry"].record_event("trajectory_start", ok=True)

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
                time.sleep(0.1)
                continue

            if self.comp["fsm"].state() == MissionState.HOLD:
                self._safe_transition(MissionState.RUN)
                self.comp["telemetry"].record_event("hold_recovered", ok=True)

            t_elapsed = time.time() - mission_start_time
            leader_ref = self.comp["leader_ref_gen"].reference_at(t_elapsed)
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
                        frame.leader_positions
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
                time.sleep(0.1)
                continue

            if self.comp["fsm"].state() == MissionState.HOLD:
                self._safe_transition(MissionState.RUN)
                self.comp["telemetry"].record_event("hold_recovered", ok=True)

            # 8. 调度器生成发送计划
            plan = self.comp["scheduler"].plan(
                snapshot,
                self.comp["fsm"].state(),
                leader_ref,
                commands,
                safety_decision,
                parked_follower_ids=[],
            )

            # 9. 执行
            if plan.leader_actions:
                self.comp["leader_executor"].execute(plan.leader_actions)
            if plan.follower_actions:
                self.comp["follower_executor"].execute_velocity(plan.follower_actions)
            if plan.hold_actions:
                self.comp["follower_executor"].execute_hold(plan.hold_actions)

            if snapshot.seq > self._last_processed_seq:
                self._last_processed_seq = snapshot.seq

            follower_command_norms = {}
            if commands is not None:
                follower_command_norms = {
                    drone_id: float((cmd**2).sum() ** 0.5)
                    for drone_id, cmd in commands.commands.items()
                }

            self.comp["telemetry"].log(
                TelemetryRecord(
                    t_wall=time.time(),
                    mission_state=self.comp["fsm"].state().value,
                    readiness=self._readiness_report,
                    phase_events=self.comp["telemetry"].phase_events(),
                    snapshot_seq=snapshot.seq,
                    snapshot_t_meas=snapshot.t_meas,
                    health={
                        drone_id: sample.values
                        for drone_id, sample in self.comp["health_bus"].latest().items()
                    },
                    frame_valid=(frame.valid if frame is not None else None),
                    frame_condition_number=(
                        frame.condition_number if frame is not None else None
                    ),
                    safety_action=safety_decision.action,
                    safety_reasons=safety_decision.reasons,
                    safety_reason_codes=safety_decision.reason_codes,
                    scheduler_reason=(plan.diagnostics or {}).get("reason"),
                    scheduler_diagnostics=plan.diagnostics or {},
                    leader_action_count=len(plan.leader_actions),
                    follower_action_count=len(plan.follower_actions),
                    follower_command_norms=follower_command_norms,
                )
            )

            time.sleep(0.01)

    def shutdown(self):
        """关闭"""
        logger.info("=== 关闭系统 ===")
        self._running = False
        if "telemetry" in self.comp:
            self.comp["telemetry"].record_event("shutdown", ok=True)
        if "pose_source" in self.comp:
            self.comp["pose_source"].stop()
        if "telemetry" in self.comp:
            self.comp["telemetry"].close()
        if "link_manager" in self.comp:
            self.comp["link_manager"].close_all()

    def _on_pose_update(self, drone_id, pos, timestamp):
        """定位数据回调 - 直接推送到PoseBus"""
        self.comp["pose_bus"].update_agent(drone_id, pos, timestamp)

    def _emergency_land(self):
        """紧急降落"""
        logger.error("=== 紧急降落 ===")
        self.comp["telemetry"].record_event("emergency_land", ok=True)
        self.comp["fsm"].force_abort()

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
        except Exception as e:
            logger.error(f"Failed to land swarm: {e}")

        time.sleep(3.0)

    def _enter_hold_mode(self):
        if self.comp["fsm"].state() != MissionState.HOLD:
            self._safe_transition(MissionState.HOLD)

        from ..runtime.command_plan import HoldAction

        hold_actions = [
            HoldAction(drone_id=fid) for fid in self.comp["fleet"].follower_ids()
        ]
        self.comp["follower_executor"].execute_hold(hold_actions)

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
