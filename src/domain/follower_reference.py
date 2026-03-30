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
        """
        # 检查leader几何
        leader_ids = self.formation.leader_ids
        if any(lid not in leader_measurements for lid in leader_ids):
            return FollowerReferenceSet([], {}, None, float("inf"), False)

        leader_pos_array = np.array([leader_measurements[lid] for lid in leader_ids])

        if len(leader_pos_array) < 4:
            return FollowerReferenceSet([], {}, None, float("inf"), False)

        # 检查条件数
        p0 = leader_pos_array[0]
        diff_matrix = leader_pos_array[1:] - p0
        cond = np.linalg.cond(diff_matrix)

        if cond > self.max_cond:  # 使用配置的阈值
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
