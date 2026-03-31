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


class SafetyManager:
    def __init__(self, config: SafetyConfig, fleet_model):
        self.config = config
        self.fleet = fleet_model

    def evaluate(
        self,
        snapshot: PoseSnapshot,
        frame=None,
        commands: FollowerCommandSet | None = None,
        follower_ref=None,
        health: dict | None = None,
    ) -> SafetyDecision:
        """评估安全状态"""
        structured_reasons: list[SafetyReason] = []

        def add_reason(
            code: str, severity: Literal["HOLD", "ABORT"], message: str, **details
        ):
            structured_reasons.append(
                SafetyReason(
                    code=code, severity=severity, message=message, details=details
                )
            )

        # 1. 检查断连
        if snapshot.disconnected_ids:
            add_reason(
                "DISCONNECTED",
                "ABORT",
                f"Disconnected drones: {snapshot.disconnected_ids}",
                drone_ids=snapshot.disconnected_ids,
            )

        # 2. 检查边界
        for drone_id in self.fleet.all_ids():
            idx = self.fleet.id_to_index(drone_id)
            if not snapshot.fresh_mask[idx]:
                continue

            pos = snapshot.positions[idx]
            if np.any(pos < self.config.boundary_min) or np.any(
                pos > self.config.boundary_max
            ):
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

        # 4. 检查follower_ref有效性
        if follower_ref is not None and not follower_ref.valid:
            add_reason(
                "FOLLOWER_REF_INVALID",
                "HOLD",
                f"Follower ref invalid: cond={follower_ref.frame_condition_number:.2f}",
                condition_number=follower_ref.frame_condition_number,
            )

        # 5. 检查命令饱和
        if commands is not None:
            for drone_id, cmd in commands.commands.items():
                norm = float(np.linalg.norm(cmd))
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
