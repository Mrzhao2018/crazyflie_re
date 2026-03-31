"""Follower参考生成器 - 从leader实测位置恢复affine frame"""

import numpy as np
from dataclasses import dataclass


@dataclass
class FollowerReferenceSet:
    """Follower参考集合"""

    follower_ids: list[int]
    target_positions: dict  # {drone_id: np.ndarray}
    target_velocities: dict | None
    frame_condition_number: float
    valid: bool


class FollowerReferenceGenerator:
    """Follower参考生成器"""

    def __init__(self, formation_model, afc_model, max_condition_number=100.0):
        self.formation = formation_model
        self.afc = afc_model
        self.max_cond = max_condition_number

    def compute(self, leader_measurements: dict) -> FollowerReferenceSet:
        """从leader实测位置计算follower目标

        使用AFC稳态解：p_f* = -Omega_ff^{-1} Omega_fl p_l

        注意：leader 几何有效性/主条件数判断的所有权在 AffineFrameEstimator。
        这里保留的是轻量防御式检查，避免调用方误传不完整数据；
        正常运行路径应当先检查 frame.valid，再调用本函数。
        """
        leader_ids = self.formation.leader_ids
        if any(lid not in leader_measurements for lid in leader_ids):
            return FollowerReferenceSet([], {}, None, float("inf"), False)

        leader_pos_array = np.array([leader_measurements[lid] for lid in leader_ids])

        if len(leader_pos_array) < 4:
            return FollowerReferenceSet([], {}, None, float("inf"), False)

        # 轻量防御：如果调用方跳过了 frame estimator，这里仍避免 NaN/退化输入继续传播
        if np.isnan(leader_pos_array).any():
            return FollowerReferenceSet([], {}, None, float("inf"), False)

        p0 = leader_pos_array[0]
        diff_matrix = leader_pos_array[1:] - p0
        rank = np.linalg.matrix_rank(diff_matrix)
        cond = np.linalg.cond(diff_matrix) if rank == 3 else float("inf")

        if rank < 3 or cond > self.max_cond:
            return FollowerReferenceSet([], {}, None, cond, False)

        # 计算稳态
        target_positions = self.afc.steady_state(leader_measurements)

        return FollowerReferenceSet(
            follower_ids=list(target_positions.keys()),
            target_positions=target_positions,
            target_velocities=None,
            frame_condition_number=cond,
            valid=True,
        )
