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

    def estimate(
        self, snapshot: PoseSnapshot, leader_ids: list[int]
    ) -> AffineFrameState:
        """估计当前affine frame"""
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

        # 计算几何条件数
        pos_array = np.array([leader_positions[lid] for lid in leader_ids])

        if len(pos_array) < 4:
            diagnostics["reason"] = "insufficient_leaders"
            return AffineFrameState(
                False, snapshot.t_meas, leader_ids, {}, float("inf"), diagnostics
            )

        p0 = pos_array[0]
        diff = pos_array[1:] - p0
        rank = np.linalg.matrix_rank(diff)
        cond = np.linalg.cond(diff) if rank == 3 else float("inf")
        diagnostics["rank"] = int(rank)

        return AffineFrameState(
            valid=(rank == 3),
            t_meas=snapshot.t_meas,
            leader_ids=leader_ids,
            leader_positions=leader_positions,
            condition_number=cond,
            diagnostics=diagnostics,
        )
