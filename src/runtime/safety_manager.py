"""安全管理器"""

from dataclasses import dataclass, field
from typing import Literal
import logging
import numpy as np
from .pose_snapshot import PoseSnapshot
from ..config.schema import SafetyConfig
from .follower_controller import FollowerCommandSet

logger = logging.getLogger(__name__)


@dataclass
class SafetyReason:
    code: str
    severity: Literal["TELEMETRY", "HOLD", "ABORT"]
    message: str
    details: dict = field(default_factory=dict)


@dataclass
class SafetyDecision:
    action: Literal["EXECUTE", "HOLD", "ABORT"]
    reasons: list[str]
    reason_codes: list[str] = field(default_factory=list)
    structured_reasons: list[SafetyReason] = field(default_factory=list)


@dataclass
class FastGateDecision:
    """fast_gate 细粒度结果。

    ``HOLD_GROUP`` 表示 disconnect 只覆盖了部分 radio_group；主循环可以把这些
    group 的 follower 降级为 parked hold 而不是整队 ABORT。``degrade_groups``
    仅在 ``action == "HOLD_GROUP"`` 时有意义。
    """

    action: Literal["EXECUTE", "HOLD_GROUP", "ABORT"]
    reason_codes: list[str] = field(default_factory=list)
    degrade_groups: list[int] = field(default_factory=list)


class SafetyManager:
    def __init__(self, config: SafetyConfig, fleet_model, link_state_bus=None):
        self.config = config
        self.fleet = fleet_model
        self.link_state_bus = link_state_bus
        self._all_ids_cache = tuple(fleet_model.all_ids())
        self._all_idx = np.asarray(
            [fleet_model.id_to_index(d) for d in self._all_ids_cache], dtype=int
        )
        self._boundary_min = np.asarray(config.boundary_min, dtype=float)
        self._boundary_max = np.asarray(config.boundary_max, dtype=float)
        self._pose_jump_streaks: dict[int, int] = {}

    def _combined_disconnected_ids(self, snapshot: PoseSnapshot) -> list[int]:
        combined = set(snapshot.disconnected_ids)
        if self.link_state_bus is not None:
            try:
                combined.update(self.link_state_bus.disconnected_ids())
            except (AttributeError, TypeError, KeyError) as exc:
                logger.warning("Failed to get link_state disconnected IDs: %s", exc)
        return sorted(combined)

    def _check_boundaries(self, snapshot: PoseSnapshot) -> tuple[bool, list[int]]:
        """检查边界违规（向量化）
        
        Returns:
            (has_violation, violated_drone_ids)
        """
        fresh_all = snapshot.fresh_mask[self._all_idx]
        if not fresh_all.any():
            return False, []
        
        idx_active = self._all_idx[fresh_all]
        positions_active = snapshot.positions[idx_active]
        below = np.any(positions_active < self._boundary_min, axis=1)
        above = np.any(positions_active > self._boundary_max, axis=1)
        violated = below | above
        
        if not violated.any():
            return False, []
        
        ids_active = [
            self._all_ids_cache[i]
            for i, ok in enumerate(fresh_all) if ok
        ]
        violated_ids = [
            ids_active[i] for i, bad in enumerate(violated) if bad
        ]
        return True, violated_ids

    def _check_disconnections(
        self, snapshot: PoseSnapshot, ignored_disconnected_ids: set[int]
    ) -> list[SafetyReason]:
        """检查断连"""
        remaining_disconnected = [
            drone_id
            for drone_id in self._combined_disconnected_ids(snapshot)
            if drone_id not in ignored_disconnected_ids
        ]
        if not remaining_disconnected:
            return []
        
        return [SafetyReason(
            code="DISCONNECTED",
            severity="ABORT",
            message=f"Disconnected drones: {remaining_disconnected}",
            details={
                "drone_ids": remaining_disconnected,
                "ignored_drone_ids": sorted(ignored_disconnected_ids),
            }
        )]

    def _check_boundaries_detailed(self, snapshot: PoseSnapshot) -> list[SafetyReason]:
        """检查边界违规（返回详细的 SafetyReason）"""
        has_violation, violated_ids = self._check_boundaries(snapshot)
        if not has_violation:
            return []
        
        reasons = []
        for drone_id in violated_ids:
            idx = self.fleet.id_to_index(drone_id)
            pos = snapshot.positions[idx]
            reasons.append(SafetyReason(
                code="OUT_OF_BOUNDS",
                severity="ABORT",
                message=f"Drone {drone_id} out of bounds",
                details={"drone_id": drone_id, "position": pos.tolist()}
            ))
        return reasons

    def _check_frame(self, frame) -> list[SafetyReason]:
        """检查 frame 条件数"""
        if frame is None:
            return []
        
        reasons = []
        if not frame.valid:
            reasons.append(SafetyReason(
                code="FRAME_INVALID",
                severity="HOLD",
                message="Frame invalid"
            ))
        if frame.condition_number > self.config.max_condition_number:
            reasons.append(SafetyReason(
                code="FRAME_DEGENERATE",
                severity="HOLD",
                message=f"Frame degenerate: cond={frame.condition_number:.2f}",
                details={"condition_number": frame.condition_number}
            ))
        return reasons

    def _check_follower_ref(self, follower_ref) -> list[SafetyReason]:
        """检查 follower_ref 有效性"""
        if follower_ref is not None and not follower_ref.valid:
            return [SafetyReason(
                code="FOLLOWER_REF_INVALID",
                severity="HOLD",
                message="Follower ref invalid"
            )]
        return []

    def _check_command_saturation(self, commands: FollowerCommandSet | None) -> list[SafetyReason]:
        """检查命令饱和"""
        if commands is None:
            return []
        
        reasons = []
        precomputed_norms = commands.diagnostics.get("command_norms") or {}
        for drone_id, cmd in commands.commands.items():
            norm_value = precomputed_norms.get(drone_id)
            norm = (
                float(norm_value)
                if norm_value is not None
                else float(np.linalg.norm(cmd))
            )
            if norm > self.config.max_command_norm:
                reasons.append(SafetyReason(
                    code="COMMAND_SATURATED",
                    severity="HOLD",
                    message=f"Drone {drone_id} command saturated: {norm:.2f}m/s",
                    details={"drone_id": drone_id, "norm": norm}
                ))
        return reasons

    def _check_pose_jumps(
        self, pose_window: dict[int, list[tuple[float, np.ndarray]]] | None
    ) -> list[SafetyReason]:
        """检查运行时 pose 跳变 / 垂直速度"""
        if not pose_window:
            return []
        
        jump_thr = float(getattr(self.config, "runtime_pose_jump_threshold", 0.0))
        speed_thr = float(getattr(self.config, "runtime_pose_speed_threshold", 0.0))
        vz_thr = float(getattr(self.config, "runtime_vertical_speed_threshold", 0.0))
        speed_min_dt = float(getattr(self.config, "runtime_pose_speed_min_dt", 0.0))
        speed_min_jump = float(getattr(self.config, "runtime_pose_speed_min_jump", 0.0))
        hold_streak_required = max(
            1, int(getattr(self.config, "runtime_pose_jump_hold_streak", 1))
        )
        
        if jump_thr <= 0 and speed_thr <= 0 and vz_thr <= 0:
            return []
        
        reasons = []
        pose_jump_drone_ids: set[int] = set()
        
        for drone_id, samples in pose_window.items():
            if len(samples) < 2:
                continue
            ordered = sorted(samples, key=lambda item: item[0])
            t_prev, p_prev = ordered[-2]
            t_cur, p_cur = ordered[-1]
            dt = float(t_cur) - float(t_prev)
            
            if dt <= 1e-6:
                continue
            
            delta = np.asarray(p_cur, dtype=float) - np.asarray(p_prev, dtype=float)
            jump = float(np.linalg.norm(delta))
            speed = jump / dt
            vertical_speed = abs(float(delta[2])) / dt
            speed_check_enabled = dt >= speed_min_dt and jump >= speed_min_jump
            
            if (
                (jump_thr > 0 and jump > jump_thr)
                or (speed_check_enabled and speed_thr > 0 and speed > speed_thr)
                or (speed_check_enabled and vz_thr > 0 and vertical_speed > vz_thr)
            ):
                pose_jump_drone_ids.add(drone_id)
                streak = self._pose_jump_streaks.get(drone_id, 0) + 1
                self._pose_jump_streaks[drone_id] = streak
                
                reasons.append(SafetyReason(
                    code="POSE_JUMP",
                    severity="HOLD" if streak >= hold_streak_required else "TELEMETRY",
                    message=f"Drone {drone_id} pose jump detected",
                    details={
                        "drone_id": drone_id,
                        "jump": jump,
                        "speed": speed,
                        "vertical_speed": vertical_speed,
                        "dt": dt,
                        "jump_threshold": jump_thr,
                        "speed_threshold": speed_thr,
                        "vertical_speed_threshold": vz_thr,
                        "speed_min_dt": speed_min_dt,
                        "speed_min_jump": speed_min_jump,
                        "speed_check_enabled": speed_check_enabled,
                        "streak": streak,
                        "required_streak": hold_streak_required,
                    }
                ))
        
        # 清除未跳变无人机的计数
        for drone_id in list(self._pose_jump_streaks):
            if drone_id not in pose_jump_drone_ids:
                self._pose_jump_streaks.pop(drone_id, None)
        
        return reasons

    def _check_battery(
        self, health: dict | None, health_window: dict | None
    ) -> list[SafetyReason]:
        """检查电池健康状态"""
        if health is None or self.config.min_vbat <= 0:
            return []
        
        low_required = max(1, int(getattr(self.config, "min_vbat_abort_samples", 1)))
        critical = float(getattr(self.config, "min_vbat_critical", 0.0))
        reasons = []
        
        for drone_id, sample in health.items():
            vbat = sample.values.get("pm.vbat")
            if vbat is None:
                continue
            
            vbat_value = float(vbat)
            immediate_critical = critical > 0 and vbat_value <= critical
            
            window_samples = (health_window or {}).get(drone_id, [])
            window_values = [
                float(window_sample.values["pm.vbat"])
                for window_sample in window_samples
                if "pm.vbat" in window_sample.values
            ]
            if vbat_value not in window_values:
                window_values.append(vbat_value)
            
            window_count = len(window_values)
            window_min = min(window_values) if window_values else vbat_value
            window_median = (
                float(np.median(np.array(window_values, dtype=float)))
                if window_values
                else vbat_value
            )
            sustained_low = (
                window_count >= low_required
                and window_median < self.config.min_vbat
            )
            
            if immediate_critical or sustained_low:
                reasons.append(SafetyReason(
                    code="LOW_BATTERY",
                    severity="ABORT",
                    message=f"Drone {drone_id} battery low: {vbat_value:.2f}V",
                    details={
                        "drone_id": drone_id,
                        "vbat": vbat_value,
                        "threshold": self.config.min_vbat,
                        "window_min": window_min,
                        "window_median": window_median,
                        "window_sample_count": window_count,
                        "required_samples": low_required,
                        "window_s": float(getattr(self.config, "min_vbat_window_s", 0.0)),
                        "critical_threshold": critical,
                        "immediate_critical": immediate_critical,
                        "sustained_low": sustained_low,
                    }
                ))
        
        return reasons

    def _check_estimator_variance(self, health: dict | None) -> list[SafetyReason]:
        """检查 onboard EKF 置信度"""
        if health is None or self.config.estimator_variance_threshold <= 0:
            return []
        
        thr = float(self.config.estimator_variance_threshold)
        reasons = []
        
        for drone_id, sample in health.items():
            var_values = [
                sample.values.get("kalman.varPX"),
                sample.values.get("kalman.varPY"),
                sample.values.get("kalman.varPZ"),
            ]
            present = [float(v) for v in var_values if v is not None]
            if not present:
                continue
            
            vmax = max(present)
            if vmax > thr:
                reasons.append(SafetyReason(
                    code="ESTIMATOR_DIVERGENCE",
                    severity="HOLD",
                    message=f"Drone {drone_id} EKF variance high: {vmax:.4f}",
                    details={"drone_id": drone_id, "variance": vmax, "threshold": thr}
                ))
        
        return reasons

    def _check_separation(self, snapshot: PoseSnapshot) -> list[SafetyReason]:
        """检查互机距离"""
        if self.config.min_inter_drone_distance <= 0:
            return []
        
        min_pair = None
        min_distance = None
        fresh_all = snapshot.fresh_mask[self._all_idx]
        active_ids = [
            self._all_ids_cache[i]
            for i, ok in enumerate(fresh_all)
            if ok
        ]
        
        for i, left_id in enumerate(active_ids):
            left_pos = snapshot.positions[self.fleet.id_to_index(left_id)]
            for right_id in active_ids[i + 1:]:
                right_pos = snapshot.positions[self.fleet.id_to_index(right_id)]
                distance = float(np.linalg.norm(left_pos - right_pos))
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    min_pair = [left_id, right_id]
        
        if (
            min_distance is not None
            and min_distance < self.config.min_inter_drone_distance
        ):
            severity = (
                "HOLD"
                if self.config.inter_drone_separation_action == "hold"
                else "TELEMETRY"
            )
            return [SafetyReason(
                code="INTER_DRONE_SEPARATION",
                severity=severity,
                message=f"Inter-drone separation low: {min_distance:.3f}m",
                details={
                    "pair": min_pair,
                    "distance": min_distance,
                    "threshold": self.config.min_inter_drone_distance,
                }
            )]
        
        return []

    def _make_decision(self, structured_reasons: list[SafetyReason]) -> SafetyDecision:
        """根据检查结果做出安全决策"""
        reasons = [reason.message for reason in structured_reasons]
        reason_codes = [reason.code for reason in structured_reasons]
        
        if any(reason.severity == "ABORT" for reason in structured_reasons):
            return SafetyDecision(
                action="ABORT",
                reasons=reasons,
                reason_codes=reason_codes,
                structured_reasons=structured_reasons,
            )
        elif any(reason.severity == "HOLD" for reason in structured_reasons):
            return SafetyDecision(
                action="HOLD",
                reasons=reasons,
                reason_codes=reason_codes,
                structured_reasons=structured_reasons,
            )
        else:
            return SafetyDecision(
                action="EXECUTE",
                reasons=reasons,
                reason_codes=reason_codes,
                structured_reasons=structured_reasons,
            )

    def evaluate(
        self,
        snapshot: PoseSnapshot,
        frame=None,
        commands: FollowerCommandSet | None = None,
        follower_ref=None,
        health: dict | None = None,
        health_window: dict | None = None,
        pose_window: dict[int, list[tuple[float, np.ndarray]]] | None = None,
        ignored_disconnected_ids: set[int] | None = None,
    ) -> SafetyDecision:
        """评估安全状态（重构版：组合各独立检查）"""
        ignored_disconnects = set(ignored_disconnected_ids or ())
        
        # 组合所有检查
        structured_reasons: list[SafetyReason] = []
        structured_reasons.extend(self._check_disconnections(snapshot, ignored_disconnects))
        structured_reasons.extend(self._check_boundaries_detailed(snapshot))
        structured_reasons.extend(self._check_frame(frame))
        structured_reasons.extend(self._check_follower_ref(follower_ref))
        structured_reasons.extend(self._check_command_saturation(commands))
        structured_reasons.extend(self._check_pose_jumps(pose_window))
        structured_reasons.extend(self._check_battery(health, health_window))
        structured_reasons.extend(self._check_estimator_variance(health))
        structured_reasons.extend(self._check_separation(snapshot))
        
        return self._make_decision(structured_reasons)

    def fast_gate(self, snapshot: PoseSnapshot) -> tuple[bool, list[str]]:
        """轻量前置检查 —— 断连 / 越界。返回 (blocked, reason_codes)。

        不生成 SafetyReason 对象，也不接受 frame / commands / health。
        若 blocked=True，主循环应当按 ABORT 处理，跳过后续 frame / control 计算。
        完整原因（含 severity / message / details）由后续 evaluate() 补齐。
        """
        reasons: list[str] = []
        disconnected_ids = self._combined_disconnected_ids(snapshot)
        if disconnected_ids:
            reasons.append(f"DISCONNECTED:{disconnected_ids}")

        fresh = snapshot.fresh_mask
        if fresh.any():
            positions = snapshot.positions[fresh]
            below = np.any(positions < self._boundary_min, axis=1)
            above = np.any(positions > self._boundary_max, axis=1)
            if below.any() or above.any():
                reasons.append("OUT_OF_BOUNDS")

        return (bool(reasons), reasons)

    def fast_gate_decision(self, snapshot: PoseSnapshot) -> FastGateDecision:
        """细粒度 fast_gate：区分整体 ABORT、部分组掉线 HOLD_GROUP、通过三种情况。

        规则：

        * 越界 => ABORT（始终）
        * disconnected_ids 覆盖所有已知 radio_group => ABORT
        * disconnected_ids 只覆盖部分 group => HOLD_GROUP，degrade_groups 列出受影响的 group id
        * 其他 => EXECUTE
        """

        reason_codes: list[str] = []

        # 边界检查（复用 _check_boundaries）
        out_of_bounds, _ = self._check_boundaries(snapshot)
        if out_of_bounds:
            reason_codes.append("OUT_OF_BOUNDS")
            return FastGateDecision(action="ABORT", reason_codes=reason_codes, degrade_groups=[])

        disconnected_ids = self._combined_disconnected_ids(snapshot)
        if not disconnected_ids:
            return FastGateDecision(action="EXECUTE", reason_codes=[], degrade_groups=[])

        reason_codes.append(f"DISCONNECTED:{disconnected_ids}")
        disconnected_groups = {
            self.fleet.get_radio_group(drone_id)
            for drone_id in disconnected_ids
        }
        all_groups = {
            self.fleet.get_radio_group(drone_id) for drone_id in self._all_ids_cache
        }

        if disconnected_groups >= all_groups:
            return FastGateDecision(
                action="ABORT", reason_codes=reason_codes, degrade_groups=[]
            )

        return FastGateDecision(
            action="HOLD_GROUP",
            reason_codes=reason_codes,
            degrade_groups=sorted(disconnected_groups),
        )
