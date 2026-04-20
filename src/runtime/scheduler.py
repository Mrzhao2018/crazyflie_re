"""命令调度器 - 控制发包频率和门控"""

import time
from collections.abc import Mapping, Sized

import numpy as np

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

    def __init__(
        self,
        config: CommConfig,
        mission_fsm: MissionFSM | None = None,
        fleet_model=None,
        *,
        link_quality_provider=None,
    ):
        self.config = config
        self.mission_fsm = mission_fsm
        self.fleet = fleet_model
        self._link_quality_provider = link_quality_provider
        self._link_quality_soft_floor = float(
            getattr(config, "link_quality_soft_floor", 0.0)
        )
        self._link_quality_backoff_scale = float(
            getattr(config, "link_quality_backoff_scale", 1.0)
        )
        self._link_quality_deadband_scale = float(
            getattr(config, "link_quality_deadband_scale", 1.0)
        )
        self.last_pose_seq_by_group: dict[int, int] = {}
        self.last_follower_tx_time: dict[int, float] = {}
        self.last_leader_update_time = 0.0
        self.last_hold_tx_time: dict[int, float] = {}
        self.last_follower_cmd: dict[int, object] = {}
        self.follower_tx_interval = self._interval_from_frequency(config.follower_tx_freq)
        self.leader_update_interval = self._interval_from_frequency(config.leader_update_freq)
        self.hold_tx_interval = self._interval_from_frequency(config.parked_hold_freq)
        self.follower_cmd_deadband = config.follower_cmd_deadband
        # 预先按 radio group 分桶，避免每次 plan() 都重新聚合
        self._follower_groups: dict[int, list[int]] = {}
        self._leader_groups: dict[int, list[int]] = {}
        if fleet_model is not None:
            for drone_id in fleet_model.follower_ids():
                self._follower_groups.setdefault(
                    fleet_model.get_radio_group(drone_id), []
                ).append(drone_id)
            for drone_id in fleet_model.leader_ids():
                self._leader_groups.setdefault(
                    fleet_model.get_radio_group(drone_id), []
                ).append(drone_id)

    def _radio_group(self, drone_id: int) -> int:
        if self.fleet is None:
            return 0
        return self.fleet.get_radio_group(drone_id)

    def _link_quality_for_group(self, group_id: int) -> float | None:
        if (
            self._link_quality_provider is None
            or self._link_quality_soft_floor <= 0.0
        ):
            return None
        try:
            return self._link_quality_provider(group_id)
        except Exception:
            return None

    def _group_is_degraded(self, group_id: int) -> bool:
        quality = self._link_quality_for_group(group_id)
        if quality is None:
            return False
        return quality < self._link_quality_soft_floor

    def _follower_tx_interval_for_group(self, group_id: int) -> float:
        if self._group_is_degraded(group_id):
            return self.follower_tx_interval * self._link_quality_backoff_scale
        return self.follower_tx_interval

    def _deadband_for_drone(self, drone_id: int) -> float:
        group_id = self._radio_group(drone_id)
        if self._group_is_degraded(group_id):
            return self.follower_cmd_deadband * self._link_quality_deadband_scale
        return self.follower_cmd_deadband

    def _group_drone_ids(self, drone_ids: list[int]) -> dict[int, list[int]]:
        if not drone_ids:
            return {}

        # Fast path: drone_ids 恰好等于 follower_ids 全集或 leader_ids 全集
        if self.fleet is not None:
            drone_tuple = tuple(drone_ids)
            if drone_tuple == tuple(self.fleet.follower_ids()):
                return {g: list(m) for g, m in self._follower_groups.items()}
            if drone_tuple == tuple(self.fleet.leader_ids()):
                return {g: list(m) for g, m in self._leader_groups.items()}

        drone_set = set(drone_ids)
        grouped: dict[int, list[int]] = {}
        # fast path: restrict the prebuilt groups to the active subset
        if self._follower_groups or self._leader_groups:
            for group_id, members in self._follower_groups.items():
                selection = [d for d in members if d in drone_set]
                if selection:
                    grouped[group_id] = selection
            for group_id, members in self._leader_groups.items():
                selection = [d for d in members if d in drone_set]
                if selection:
                    existing = grouped.setdefault(group_id, [])
                    existing.extend(selection)
            # any drone not covered (e.g. tests with bare ints) falls back below
            covered = {drone for members in grouped.values() for drone in members}
            missing = [d for d in drone_ids if d not in covered]
            if missing:
                for drone_id in missing:
                    grouped.setdefault(self._radio_group(drone_id), []).append(drone_id)
            return grouped

        for drone_id in drone_ids:
            grouped.setdefault(self._radio_group(drone_id), []).append(drone_id)
        return grouped

    def _group_commands(self, command_dict: dict) -> dict[int, dict]:
        grouped: dict[int, dict] = {}
        for drone_id, command in command_dict.items():
            grouped.setdefault(self._radio_group(drone_id), {})[drone_id] = command
        return grouped

    @staticmethod
    def _group_counts(grouped: Mapping[int, Sized]) -> dict[int, int]:
        counts = {}
        for group_id, items in grouped.items():
            counts[group_id] = len(items)
        return counts

    def _build_hold_actions(
        self,
        drone_ids: list[int],
        now: float,
    ) -> tuple[list[HoldAction], list[int], list[int], dict[int, list[int]]]:
        grouped = self._group_drone_ids(drone_ids)
        hold_actions: list[HoldAction] = []
        sent_groups: list[int] = []
        blocked_groups: list[int] = []
        for group_id, members in grouped.items():
            last_tx = self.last_hold_tx_time.get(group_id, 0.0)
            if now - last_tx < self.hold_tx_interval:
                blocked_groups.append(group_id)
                continue
            hold_actions.extend(HoldAction(drone_id=drone_id) for drone_id in members)
            self.last_hold_tx_time[group_id] = now
            sent_groups.append(group_id)
        return hold_actions, sent_groups, blocked_groups, grouped

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
        parked_grouped = self._group_drone_ids(parked_follower_ids)
        diagnostics["parked_follower_ids"] = parked_follower_ids
        diagnostics["parked_radio_groups"] = sorted(parked_grouped)
        diagnostics["parked_group_counts"] = self._group_counts(parked_grouped)

        if policy is None:
            policy = self._policy_for_state(mission_state)

        if safety.action == "ABORT":
            diagnostics["reason"] = "abort"
            return TxPlan([], [], [], diagnostics=diagnostics)

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
            hold_actions, hold_sent_groups, hold_blocked_groups, hold_grouped = (
                self._build_hold_actions(hold_targets, now)
            )
            diagnostics["reason"] = (
                "hold_rate_limited"
                if hold_targets and not hold_actions
                else "hold"
            )
            diagnostics["hold_tx_groups_sent"] = hold_sent_groups
            diagnostics["hold_tx_groups_blocked"] = hold_blocked_groups
            diagnostics["hold_group_counts"] = self._group_counts(hold_grouped)
            return TxPlan([], [], hold_actions, diagnostics=diagnostics)

        if parked_follower_ids:
            hold_actions, hold_sent_groups, hold_blocked_groups, hold_grouped = (
                self._build_hold_actions(parked_follower_ids, now)
            )
            diagnostics["hold_tx_groups_sent"] = hold_sent_groups
            diagnostics["hold_tx_groups_blocked"] = hold_blocked_groups
            diagnostics["hold_group_counts"] = self._group_counts(hold_grouped)
        else:
            diagnostics["hold_tx_groups_sent"] = []
            diagnostics["hold_tx_groups_blocked"] = []
            diagnostics["hold_group_counts"] = {}

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

        if policy.follower_mode != "velocity":
            diagnostics["reason"] = "policy_blocks_follower"
            return TxPlan(leader_actions, [], hold_actions, diagnostics=diagnostics)

        command_dict = follower_cmds.commands if follower_cmds is not None else {}
        if parked_follower_ids:
            parked_set = set(parked_follower_ids)
            command_dict = {
                drone_id: command
                for drone_id, command in command_dict.items()
                if drone_id not in parked_set
            }
        if not command_dict:
            diagnostics["reason"] = "hold_only" if hold_actions else "no_follower_commands"
            return TxPlan(leader_actions, [], hold_actions, diagnostics=diagnostics)

        filtered_commands = self._filter_commands_by_deadband(command_dict, follower_cmds)
        if not filtered_commands:
            diagnostics["reason"] = "deadband_blocked"
            return TxPlan(leader_actions, [], hold_actions, diagnostics=diagnostics)

        grouped_commands = self._group_commands(filtered_commands)
        actions = []
        sent_groups = []
        blocked_groups = []
        stale_groups = []
        sent_group_counts = {}
        blocked_group_counts = {}
        backoff_groups = []
        for group_id, group_commands in grouped_commands.items():
            if snapshot.seq <= self.last_pose_seq_by_group.get(group_id, -1):
                stale_groups.append(group_id)
                continue
            last_tx = self.last_follower_tx_time.get(group_id, 0.0)
            interval = self._follower_tx_interval_for_group(group_id)
            if interval > self.follower_tx_interval:
                backoff_groups.append(group_id)
            if now - last_tx < interval:
                blocked_groups.append(group_id)
                blocked_group_counts[group_id] = len(group_commands)
                continue

            for drone_id, vel in group_commands.items():
                kind = "velocity"
                position = None
                acceleration = None
                if follower_cmds is not None and follower_cmds.target_positions is not None:
                    ref_pos = follower_cmds.target_positions.get(drone_id)
                    ref_acc = (
                        follower_cmds.target_accelerations.get(drone_id)
                        if follower_cmds.target_accelerations is not None
                        else None
                    )
                    if ref_pos is not None:
                        kind = "full_state"
                        position = ref_pos
                        acceleration = (
                            ref_acc if ref_acc is not None else np.zeros(3, dtype=float)
                        )
                actions.append(
                    FollowerAction(
                        kind=kind,
                        drone_id=drone_id,
                        velocity=vel,
                        position=position,
                        acceleration=acceleration,
                    )
                )
            self.last_pose_seq_by_group[group_id] = snapshot.seq
            self.last_follower_tx_time[group_id] = now
            sent_groups.append(group_id)
            sent_group_counts[group_id] = len(group_commands)
            self.last_follower_cmd.update(
                {drone_id: vel.copy() for drone_id, vel in group_commands.items()}
            )

        diagnostics["follower_tx_groups_sent"] = sent_groups
        diagnostics["follower_tx_groups_blocked"] = blocked_groups
        diagnostics["follower_tx_groups_stale"] = stale_groups
        diagnostics["follower_tx_group_counts"] = sent_group_counts
        diagnostics["follower_tx_blocked_group_counts"] = blocked_group_counts
        diagnostics["link_quality_backoff_groups"] = sorted(set(backoff_groups))

        reason = "execute"
        if not actions and hold_actions:
            reason = "hold_only"
        elif not actions and stale_groups:
            reason = "stale_seq"
        elif not actions and blocked_groups:
            reason = "rate_limited"

        diagnostics["reason"] = reason
        return TxPlan(
            leader_actions=leader_actions,
            follower_actions=actions,
            hold_actions=hold_actions,
            diagnostics=diagnostics,
        )

    @staticmethod
    def _has_full_state_reference(
        follower_cmds: FollowerCommandSet | None, drone_id: int
    ) -> bool:
        if follower_cmds is None or follower_cmds.target_positions is None:
            return False
        return drone_id in follower_cmds.target_positions

    def _filter_commands_by_deadband(self, command_dict, follower_cmds=None):
        if self.follower_cmd_deadband <= 0.0:
            return command_dict

        filtered = {}
        for drone_id, vel in command_dict.items():
            if self._has_full_state_reference(follower_cmds, drone_id):
                filtered[drone_id] = vel
                continue

            deadband = self._deadband_for_drone(drone_id)
            last_vel = self.last_follower_cmd.get(drone_id)
            if last_vel is None:
                filtered[drone_id] = vel
                continue

            delta = float(np.linalg.norm(vel - last_vel))
            if delta >= deadband:
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
