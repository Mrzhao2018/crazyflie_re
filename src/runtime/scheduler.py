"""命令调度器 - 控制发包频率和门控"""

import time
from .pose_snapshot import PoseSnapshot
from .mission_fsm import MissionState, MissionFSM, CommandPolicy
from .command_plan import TxPlan, FollowerAction, LeaderAction, HoldAction
from .safety_manager import SafetyDecision
from .follower_controller import FollowerCommandSet
from ..domain.leader_reference import LeaderReferenceFrame
from ..config.schema import CommConfig


class CommandScheduler:
    @staticmethod
    def _interval_from_frequency(freq: float) -> float:
        return 1.0 / freq

    def __init__(self, config: CommConfig, mission_fsm: MissionFSM | None = None):
        self.config = config
        self.mission_fsm = mission_fsm
        self.last_pose_seq = -1
        self.last_follower_tx_time = 0.0
        self.last_leader_update_time = 0.0
        self.last_hold_tx_time = 0.0
        self.last_follower_cmd: dict[int, object] = {}
        self.follower_tx_interval = self._interval_from_frequency(config.follower_tx_freq)
        self.leader_update_interval = self._interval_from_frequency(config.leader_update_freq)
        self.hold_tx_interval = self._interval_from_frequency(config.parked_hold_freq)
        self.follower_cmd_deadband = config.follower_cmd_deadband

    def should_update_leaders(self) -> bool:
        """判断是否该更新leaders"""
        now = time.time()
        if now - self.last_leader_update_time >= self.leader_update_interval:
            self.last_leader_update_time = now
            return True
        return False

    def plan(
        self,
        snapshot: PoseSnapshot,
        mission_state: MissionState,
        leader_ref: LeaderReferenceFrame | None,
        follower_cmds: FollowerCommandSet | None,
        safety: SafetyDecision,
        parked_follower_ids: list | None = None,
    ) -> TxPlan:
        """生成发送计划"""
        now = time.time()
        policy = self.mission_fsm.allowed_command_policy() if self.mission_fsm else None
        diagnostics = {
            "policy": policy,
            "safety_action": safety.action,
            "safety_reason_codes": safety.reason_codes,
            "snapshot_seq": snapshot.seq,
            "mission_state": mission_state.value,
        }
        parked_follower_ids = parked_follower_ids or []

        if policy is None:
            policy = self._policy_for_state(mission_state)

        if safety.action == "ABORT":
            return TxPlan([], [], [], diagnostics={**diagnostics, "reason": "abort"})

        hold_actions = []
        if safety.action == "HOLD" or policy.follower_mode == "hold":
            hold_targets = parked_follower_ids or []
            if not hold_targets and mission_state in {
                MissionState.HOLD,
                MissionState.LAND,
                MissionState.ABORT,
                MissionState.SETTLE,
            }:
                command_dict = (
                    follower_cmds.commands if follower_cmds is not None else {}
                )
                hold_targets = list(command_dict.keys())
            if hold_targets and now - self.last_hold_tx_time >= self.hold_tx_interval:
                hold_actions = [
                    HoldAction(drone_id=drone_id) for drone_id in hold_targets
                ]
                self.last_hold_tx_time = now
            return TxPlan(
                [], [], hold_actions, diagnostics={**diagnostics, "reason": "hold"}
            )

        leader_actions = []
        if (
            leader_ref is not None
            and policy.leader_mode == "batch_goto"
            and now - self.last_leader_update_time >= self.leader_update_interval
        ):
            if leader_ref.mode == "batch_goto":
                leader_actions.append(
                    LeaderAction(
                        kind="batch_goto",
                        drone_ids=leader_ref.leader_ids,
                        payload={
                            "positions": leader_ref.positions,
                            "duration": 0.8 * self.leader_update_interval,
                        },
                    )
                )
                self.last_leader_update_time = now

        if snapshot.seq <= self.last_pose_seq:
            return TxPlan(
                leader_actions,
                [],
                [],
                diagnostics={**diagnostics, "reason": "stale_seq"},
            )

        if now - self.last_follower_tx_time < self.follower_tx_interval:
            return TxPlan(
                leader_actions,
                [],
                [],
                diagnostics={**diagnostics, "reason": "rate_limited"},
            )

        if policy.follower_mode != "velocity":
            return TxPlan(
                leader_actions,
                [],
                [],
                diagnostics={**diagnostics, "reason": "policy_blocks_follower"},
            )

        command_dict = follower_cmds.commands if follower_cmds is not None else {}
        if not command_dict:
            return TxPlan(
                leader_actions,
                [],
                [],
                diagnostics={**diagnostics, "reason": "no_follower_commands"},
            )

        filtered_commands = self._filter_commands_by_deadband(command_dict)
        if not filtered_commands:
            return TxPlan(
                leader_actions,
                [],
                [],
                diagnostics={**diagnostics, "reason": "deadband_blocked"},
            )

        # 生成follower动作
        actions = []
        for drone_id, vel in filtered_commands.items():
            actions.append(
                FollowerAction(kind="velocity", drone_id=drone_id, velocity=vel)
            )

        # 更新状态
        self.last_pose_seq = snapshot.seq
        self.last_follower_tx_time = now
        self.last_follower_cmd = {
            drone_id: vel.copy() for drone_id, vel in filtered_commands.items()
        }

        return TxPlan(
            leader_actions=leader_actions,
            follower_actions=actions,
            hold_actions=[],
            diagnostics={**diagnostics, "reason": "execute"},
        )

    def _filter_commands_by_deadband(self, command_dict):
        if self.follower_cmd_deadband <= 0.0:
            return command_dict

        filtered = {}
        for drone_id, vel in command_dict.items():
            last_vel = self.last_follower_cmd.get(drone_id)
            if last_vel is None:
                filtered[drone_id] = vel
                continue

            delta = float(((vel - last_vel) ** 2).sum() ** 0.5)
            if delta >= self.follower_cmd_deadband:
                filtered[drone_id] = vel

        return filtered

    @staticmethod
    def _policy_for_state(mission_state: MissionState) -> CommandPolicy:
        if mission_state == MissionState.RUN:
            return CommandPolicy(leader_mode="batch_goto", follower_mode="velocity")
        if mission_state in {
            MissionState.HOLD,
            MissionState.SETTLE,
            MissionState.LAND,
            MissionState.ABORT,
        }:
            return CommandPolicy(leader_mode="none", follower_mode="hold")
        return CommandPolicy(leader_mode="none", follower_mode="none")
