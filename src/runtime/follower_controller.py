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

    @staticmethod
    def _resolve_axis_gain(value: float | None, fallback: float) -> float:
        return fallback if value is None else value

    def __init__(self, config: ControlConfig):
        self.gain = config.gain
        self.max_velocity = config.max_velocity
        self.feedforward_gain = config.feedforward_gain
        self.max_feedforward_velocity = config.max_feedforward_velocity
        self.gain_xy = self._resolve_axis_gain(config.gain_xy, config.gain)
        self.gain_z = self._resolve_axis_gain(config.gain_z, config.gain)
        self.feedforward_gain_xy = self._resolve_axis_gain(
            config.feedforward_gain_xy, config.feedforward_gain
        )
        self.feedforward_gain_z = self._resolve_axis_gain(
            config.feedforward_gain_z, config.feedforward_gain
        )
        self.max_feedforward_velocity_xy = self._resolve_axis_gain(
            config.max_feedforward_velocity_xy, config.max_feedforward_velocity
        )
        self.max_feedforward_velocity_z = self._resolve_axis_gain(
            config.max_feedforward_velocity_z, config.max_feedforward_velocity
        )
        self.radial_gain_scale_xy = config.radial_gain_scale_xy
        self.radial_feedforward_scale_xy = config.radial_feedforward_scale_xy

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
        feedforward_applied = []
        radial_scaled_followers = []
        follower_radii = {
            fid: float(
                np.linalg.norm(
                    np.array(references.target_positions[fid], dtype=float)[:2]
                )
            )
            for fid in active_follower_ids
            if fid in references.target_positions
        }
        max_radius = max(follower_radii.values(), default=0.0)

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
            radius_xy = follower_radii.get(fid, 0.0)
            radius_ratio = radius_xy / max_radius if max_radius > 1e-9 else 0.0
            gain_scale_xy = 1.0 + self.radial_gain_scale_xy * radius_ratio
            ff_scale_xy = 1.0 + self.radial_feedforward_scale_xy * radius_ratio
            if gain_scale_xy > 1.0 or ff_scale_xy > 1.0:
                radial_scaled_followers.append(fid)

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
                    target_velocity = np.array(target_velocity, dtype=float)
                    xy_norm = np.linalg.norm(target_velocity[:2])
                    if xy_norm > self.max_feedforward_velocity_xy > 0:
                        target_velocity[:2] = (
                            target_velocity[:2]
                            / xy_norm
                            * self.max_feedforward_velocity_xy
                        )
                    if abs(target_velocity[2]) > self.max_feedforward_velocity_z > 0:
                        target_velocity[2] = (
                            np.sign(target_velocity[2])
                            * self.max_feedforward_velocity_z
                        )
                    velocity = velocity + np.array(
                        [
                            self.feedforward_gain_xy * ff_scale_xy * target_velocity[0],
                            self.feedforward_gain_xy * ff_scale_xy * target_velocity[1],
                            self.feedforward_gain_z * target_velocity[2],
                        ],
                        dtype=float,
                    )
                    feedforward_applied.append(fid)

            # 限幅
            speed = np.linalg.norm(velocity)
            if speed > self.max_velocity:
                velocity = velocity / speed * self.max_velocity

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
