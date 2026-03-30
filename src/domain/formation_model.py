"""编队模型"""

import numpy as np
from dataclasses import dataclass
from .fleet_model import FleetModel


@dataclass
class AffineSpanReport:
    valid: bool
    rank: int
    required_rank: int
    condition_number: float


class FormationModel:
    def __init__(
        self, nominal_positions: np.ndarray, leader_ids: list[int], fleet: FleetModel
    ):
        self.nominal_positions = nominal_positions  # (n, 3)
        self.leader_ids = leader_ids
        self.fleet = fleet

        # 提取leader和follower的nominal位置
        leader_indices = [fleet.id_to_index(lid) for lid in leader_ids]
        follower_ids = fleet.follower_ids()
        follower_indices = [fleet.id_to_index(fid) for fid in follower_ids]

        self.leader_nominal = nominal_positions[leader_indices]
        self.follower_nominal = nominal_positions[follower_indices]

    def nominal_position(self, drone_id: int) -> np.ndarray:
        idx = self.fleet.id_to_index(drone_id)
        return self.nominal_positions[idx]

    def check_affine_span(self, leader_ids: list[int]) -> AffineSpanReport:
        """检查leader是否affine span R^3"""
        leader_indices = [self.fleet.id_to_index(lid) for lid in leader_ids]
        leader_pos = self.nominal_positions[leader_indices]  # (n_l, 3)

        # 构建增广矩阵 [p1-p0, p2-p0, p3-p0]
        if len(leader_pos) < 4:
            return AffineSpanReport(False, 0, 3, float("inf"))

        p0 = leader_pos[0]
        diff_matrix = leader_pos[1:] - p0  # (3, 3)

        rank = np.linalg.matrix_rank(diff_matrix)
        cond = np.linalg.cond(diff_matrix) if rank == 3 else float("inf")

        return AffineSpanReport(
            valid=(rank == 3), rank=rank, required_rank=3, condition_number=cond
        )
