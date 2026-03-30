"""Follower控制器 - 在线速度命令生成"""

from dataclasses import dataclass
import numpy as np
from .pose_snapshot import PoseSnapshot
from ..domain.follower_reference import FollowerReferenceSet
from ..config.schema import ControlConfig


@dataclass
class FollowerCommandSet:
    commands: dict
    diagnostics: dict


class FollowerController:
    """Follower在线控制器 - 简单P控制"""

    def __init__(self, config: ControlConfig):
        self.gain = config.gain
        self.max_velocity = config.max_velocity

    def compute(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        """计算follower速度命令

        u_i = -K_p * (p_i - p_i*)
        """
        commands = {}
        skipped_stale = []
        missing_reference = []

        for fid in active_follower_ids:
            if fid not in references.target_positions:
                missing_reference.append(fid)
                continue

            idx = fleet_model.id_to_index(fid)
            if not snapshot.fresh_mask[idx]:
                skipped_stale.append(fid)
                continue

            p_current = snapshot.positions[idx]
            p_target = references.target_positions[fid]

            # P控制
            error = p_current - p_target
            velocity = -self.gain * error

            # 限幅
            speed = np.linalg.norm(velocity)
            if speed > self.max_velocity:
                velocity = velocity / speed * self.max_velocity

            commands[fid] = velocity.copy()

        return FollowerCommandSet(
            commands=commands,
            diagnostics={
                "skipped_stale_followers": skipped_stale,
                "missing_reference_followers": missing_reference,
            },
        )
