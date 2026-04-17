"""Follower控制器 - 在线速度命令生成"""

from dataclasses import dataclass
import numpy as np
from .pose_snapshot import PoseSnapshot
from ..domain.follower_reference import FollowerReferenceSet
from ..config.schema import ControlConfig
from .follower_controller_base import FollowerControllerBase


@dataclass
class FollowerCommandSet:
    commands: dict
    diagnostics: dict


class FollowerController(FollowerControllerBase):
    """Follower在线控制器 - 简单P控制"""

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
        commands: dict[int, np.ndarray] = {}
        skipped_stale: list[int] = []
        missing_reference: list[int] = []
        feedforward_applied: list[int] = []

        radial_scales, radial_scaled_followers = self._compute_radial_scales(
            references, active_follower_ids
        )

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
            gain_scale_xy, ff_scale_xy = radial_scales.get(fid, (1.0, 1.0))

            # P控制
            error = p_current - p_target
            velocity = np.array(
                [
                    -(self.gain_xy * gain_scale_xy) * error[0],
                    -(self.gain_xy * gain_scale_xy) * error[1],
                    -self.gain_z * error[2],
                ],
                dtype=float,
            )

            if references.target_velocities is not None:
                target_velocity = references.target_velocities.get(fid)
                if target_velocity is not None:
                    target_velocity = self._clip_feedforward_velocity(target_velocity)
                    velocity = velocity + np.array(
                        [
                            self.feedforward_gain_xy * ff_scale_xy * target_velocity[0],
                            self.feedforward_gain_xy * ff_scale_xy * target_velocity[1],
                            self.feedforward_gain_z * target_velocity[2],
                        ],
                        dtype=float,
                    )
                    feedforward_applied.append(fid)

            velocity = self._clip_output_velocity(velocity)
            commands[fid] = velocity.copy()

        command_norms = {
            fid: float(np.linalg.norm(velocity))
            for fid, velocity in commands.items()
        }

        return FollowerCommandSet(
            commands=commands,
            diagnostics={
                "skipped_stale_followers": skipped_stale,
                "missing_reference_followers": missing_reference,
                "feedforward_followers": feedforward_applied,
                "radial_scaled_followers": radial_scaled_followers,
                "command_norms": command_norms,
            },
        )
