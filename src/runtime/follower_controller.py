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
    # full_state 模式额外携带的 reference 三元组；velocity 模式下保持为 None
    target_positions: dict | None = None
    target_accelerations: dict | None = None


class FollowerController(FollowerControllerBase):
    """Follower在线控制器 - 简单P控制（向量化内循环）"""

    def compute(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        """计算follower速度命令

        u_i = -K_p * (p_i - p_i*) + K_ff * v_i*
        """
        missing_reference: list[int] = []
        skipped_stale: list[int] = []

        resolvable_ids: list[int] = []
        idx_list: list[int] = []
        for fid in active_follower_ids:
            if fid not in references.target_positions:
                missing_reference.append(fid)
                continue
            idx = fleet_model.id_to_index(fid)
            if not snapshot.fresh_mask[idx]:
                skipped_stale.append(fid)
                continue
            resolvable_ids.append(fid)
            idx_list.append(idx)

        commands: dict[int, np.ndarray] = {}
        feedforward_applied: list[int] = []

        radial_scales, radial_scaled_followers = self._compute_radial_scales(
            references, active_follower_ids
        )

        if resolvable_ids:
            idx_arr = np.asarray(idx_list, dtype=int)
            P = snapshot.positions[idx_arr]  # (m, 3)
            T = np.stack(
                [np.asarray(references.target_positions[fid], dtype=float) for fid in resolvable_ids]
            )  # (m, 3)
            error = P - T  # (m, 3)

            gain_scale_xy = np.array(
                [radial_scales.get(fid, (1.0, 1.0))[0] for fid in resolvable_ids]
            )
            ff_scale_xy = np.array(
                [radial_scales.get(fid, (1.0, 1.0))[1] for fid in resolvable_ids]
            )

            V = np.empty_like(error)
            V[:, 0] = -(self.gain_xy * gain_scale_xy) * error[:, 0]
            V[:, 1] = -(self.gain_xy * gain_scale_xy) * error[:, 1]
            V[:, 2] = -self.gain_z * error[:, 2]

            if references.target_velocities is not None:
                for row, fid in enumerate(resolvable_ids):
                    tv = references.target_velocities.get(fid)
                    if tv is None:
                        continue
                    tv_clipped = self._clip_feedforward_velocity(tv)
                    V[row, 0] += self.feedforward_gain_xy * ff_scale_xy[row] * tv_clipped[0]
                    V[row, 1] += self.feedforward_gain_xy * ff_scale_xy[row] * tv_clipped[1]
                    V[row, 2] += self.feedforward_gain_z * tv_clipped[2]
                    feedforward_applied.append(fid)

            for row in range(V.shape[0]):
                V[row] = self._clip_output_velocity(V[row])

            norms = np.linalg.norm(V, axis=1)
            for row, fid in enumerate(resolvable_ids):
                commands[fid] = V[row].copy()
            command_norms = {
                fid: float(norms[row]) for row, fid in enumerate(resolvable_ids)
            }
        else:
            command_norms = {}

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
