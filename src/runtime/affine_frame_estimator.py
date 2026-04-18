"""Affine Frame估计器 - 从leader实测位置恢复当前frame"""

import numpy as np
from dataclasses import dataclass
from .pose_snapshot import PoseSnapshot


@dataclass
class AffineFrameState:
    """Affine Frame状态"""

    valid: bool
    t_meas: float
    leader_ids: list[int]
    leader_positions: dict
    condition_number: float
    diagnostics: dict


class AffineFrameEstimator:
    """从pose snapshot中提取leader状态"""

    def __init__(self, fleet_model):
        self.fleet = fleet_model
        self._leader_ids_cache = tuple(fleet_model.leader_ids())
        self._leader_idx = np.asarray(
            [fleet_model.id_to_index(lid) for lid in self._leader_ids_cache],
            dtype=int,
        )

    def estimate(
        self, snapshot: PoseSnapshot, leader_ids: list[int]
    ) -> AffineFrameState:
        """估计当前affine frame"""
        if tuple(leader_ids) == self._leader_ids_cache:
            idx_arr = self._leader_idx
            fresh = snapshot.fresh_mask[idx_arr]
            stale_leaders = [
                lid for lid, ok in zip(self._leader_ids_cache, fresh) if not ok
            ]
            leader_positions = {
                lid: snapshot.positions[idx_arr[i]]
                for i, lid in enumerate(self._leader_ids_cache)
            }
        else:
            leader_positions = {}
            stale_leaders = []
            for lid in leader_ids:
                idx = self.fleet.id_to_index(lid)
                if not snapshot.fresh_mask[idx]:
                    stale_leaders.append(lid)
                leader_positions[lid] = snapshot.positions[idx]

        diagnostics = {
            "stale_leaders": stale_leaders,
            "leader_count": len(leader_ids),
        }

        if stale_leaders:
            return AffineFrameState(
                valid=False,
                t_meas=snapshot.t_meas,
                leader_ids=leader_ids,
                leader_positions=leader_positions,
                condition_number=float("inf"),
                diagnostics=diagnostics,
            )

        # 计算几何条件数：一次 SVD 同时得到 rank 与 cond
        pos_array = np.array([leader_positions[lid] for lid in leader_ids])

        if len(pos_array) < 4:
            diagnostics["reason"] = "insufficient_leaders"
            return AffineFrameState(
                False, snapshot.t_meas, leader_ids, {}, float("inf"), diagnostics
            )

        p0 = pos_array[0]
        diff = pos_array[1:] - p0
        # svdvals 只算奇异值，比 matrix_rank+cond 各一次 SVD 快 ~2x
        singular_values = np.linalg.svd(diff, compute_uv=False)
        tolerance = singular_values.max() * max(diff.shape) * np.finfo(float).eps
        rank = int(np.sum(singular_values > tolerance))
        if rank == 3 and singular_values[-1] > 0:
            cond = float(singular_values[0] / singular_values[-1])
        else:
            cond = float("inf")
        diagnostics["rank"] = rank

        return AffineFrameState(
            valid=(rank == 3),
            t_meas=snapshot.t_meas,
            leader_ids=leader_ids,
            leader_positions=leader_positions,
            condition_number=cond,
            diagnostics=diagnostics,
        )
