"""安全管理器"""

from dataclasses import dataclass, field
from typing import Literal
import numpy as np
from .pose_snapshot import PoseSnapshot
from ..config.schema import SafetyConfig
from .follower_controller import FollowerCommandSet


@dataclass
class SafetyReason:
    code: str
    severity: Literal["HOLD", "ABORT"]
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

    def _combined_disconnected_ids(self, snapshot: PoseSnapshot) -> list[int]:
        combined = set(snapshot.disconnected_ids)
        if self.link_state_bus is not None:
            try:
                combined.update(self.link_state_bus.disconnected_ids())
            except Exception:
                pass
        return sorted(combined)

    def evaluate(
        self,
        snapshot: PoseSnapshot,
        frame=None,
        commands: FollowerCommandSet | None = None,
        follower_ref=None,
        health: dict | None = None,
        ignored_disconnected_ids: set[int] | None = None,
    ) -> SafetyDecision:
        """评估安全状态"""
        structured_reasons: list[SafetyReason] = []
        ignored_disconnects = set(ignored_disconnected_ids or ())

        def add_reason(
            code: str, severity: Literal["HOLD", "ABORT"], message: str, **details
        ):
            structured_reasons.append(
                SafetyReason(
                    code=code, severity=severity, message=message, details=details
                )
            )

        # 1. 检查断连
        remaining_disconnected = [
            drone_id
            for drone_id in self._combined_disconnected_ids(snapshot)
            if drone_id not in ignored_disconnects
        ]
        if remaining_disconnected:
            add_reason(
                "DISCONNECTED",
                "ABORT",
                f"Disconnected drones: {remaining_disconnected}",
                drone_ids=remaining_disconnected,
                ignored_drone_ids=sorted(ignored_disconnects),
            )

        # 2. 检查边界（向量化）
        fresh_all = snapshot.fresh_mask[self._all_idx]
        if fresh_all.any():
            idx_active = self._all_idx[fresh_all]
            positions_active = snapshot.positions[idx_active]
            below = np.any(positions_active < self._boundary_min, axis=1)
            above = np.any(positions_active > self._boundary_max, axis=1)
            violated = below | above
            if violated.any():
                ids_active = [
                    self._all_ids_cache[i]
                    for i, ok in enumerate(fresh_all) if ok
                ]
                for i, bad in enumerate(violated):
                    if bad:
                        drone_id = ids_active[i]
                        pos = positions_active[i]
                        add_reason(
                            "OUT_OF_BOUNDS",
                            "ABORT",
                            f"Drone {drone_id} out of bounds",
                            drone_id=drone_id,
                            position=pos.tolist(),
                        )

        # 3. 检查frame条件数
        if frame is not None:
            if not frame.valid:
                add_reason("FRAME_INVALID", "HOLD", "Frame invalid")
            if frame.condition_number > self.config.max_condition_number:
                add_reason(
                    "FRAME_DEGENERATE",
                    "HOLD",
                    f"Frame degenerate: cond={frame.condition_number:.2f}",
                    condition_number=frame.condition_number,
                )

        # 4. 检查follower_ref有效性（cond 的权威由 frame 持有，这里只看 valid）
        if follower_ref is not None and not follower_ref.valid:
            add_reason(
                "FOLLOWER_REF_INVALID",
                "HOLD",
                "Follower ref invalid",
            )

        # 5. 检查命令饱和（优先复用 controller diagnostics 里的范数，避免重复计算）
        if commands is not None:
            precomputed_norms = commands.diagnostics.get("command_norms") or {}
            for drone_id, cmd in commands.commands.items():
                norm_value = precomputed_norms.get(drone_id)
                norm = (
                    float(norm_value)
                    if norm_value is not None
                    else float(np.linalg.norm(cmd))
                )
                if norm > self.config.max_command_norm:
                    add_reason(
                        "COMMAND_SATURATED",
                        "HOLD",
                        f"Drone {drone_id} command saturated: {norm:.2f}m/s",
                        drone_id=drone_id,
                        norm=norm,
                    )

        # 6. 检查健康状态
        if health is not None and self.config.min_vbat > 0:
            for drone_id, sample in health.items():
                vbat = sample.values.get("pm.vbat")
                if vbat is not None and float(vbat) < self.config.min_vbat:
                    add_reason(
                        "LOW_BATTERY",
                        "ABORT",
                        f"Drone {drone_id} battery low: {float(vbat):.2f}V",
                        drone_id=drone_id,
                        vbat=float(vbat),
                        threshold=self.config.min_vbat,
                    )

        # 7. 检查 onboard EKF 置信度（kalman variance）
        if health is not None and self.config.estimator_variance_threshold > 0:
            thr = float(self.config.estimator_variance_threshold)
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
                    add_reason(
                        "ESTIMATOR_DIVERGENCE",
                        "HOLD",
                        f"Drone {drone_id} EKF variance high: {vmax:.4f}",
                        drone_id=drone_id,
                        variance=vmax,
                        threshold=thr,
                    )

        reasons = [reason.message for reason in structured_reasons]
        reason_codes = [reason.code for reason in structured_reasons]

        # 决策
        if any(reason.severity == "ABORT" for reason in structured_reasons):
            return SafetyDecision(
                action="ABORT",
                reasons=reasons,
                reason_codes=reason_codes,
                structured_reasons=structured_reasons,
            )
        elif structured_reasons:
            return SafetyDecision(
                action="HOLD",
                reasons=reasons,
                reason_codes=reason_codes,
                structured_reasons=structured_reasons,
            )
        else:
            return SafetyDecision(
                action="EXECUTE", reasons=[], reason_codes=[], structured_reasons=[]
            )

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

        out_of_bounds = False
        fresh = snapshot.fresh_mask
        if fresh.any():
            positions = snapshot.positions[fresh]
            below = np.any(positions < self._boundary_min, axis=1)
            above = np.any(positions > self._boundary_max, axis=1)
            if below.any() or above.any():
                out_of_bounds = True
                reason_codes.append("OUT_OF_BOUNDS")

        if out_of_bounds:
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
