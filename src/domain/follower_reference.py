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

    def __init__(
        self,
        formation_model,
        afc_model,
        max_condition_number=100.0,
        *,
        time_delay_compensation_enabled: bool = False,
        estimated_total_delay_ms: float = 0.0,
        delay_prediction_gain: float = 1.0,
    ):
        self.formation = formation_model
        self.afc = afc_model
        self.max_cond = max_condition_number
        self.time_delay_compensation_enabled = time_delay_compensation_enabled
        self.estimated_total_delay_s = max(0.0, float(estimated_total_delay_ms)) / 1000.0
        self.delay_prediction_gain = max(0.0, float(delay_prediction_gain))
        self._last_target_positions: dict[int, np.ndarray] | None = None
        self._last_t_meas: float | None = None

    def compute(
        self, leader_measurements: dict, t_meas: float | None = None
    ) -> FollowerReferenceSet:
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
        target_velocities = self._estimate_target_velocities(target_positions, t_meas)
        target_positions = self._apply_delay_compensation(
            target_positions,
            target_velocities,
        )

        return FollowerReferenceSet(
            follower_ids=list(target_positions.keys()),
            target_positions=target_positions,
            target_velocities=target_velocities,
            frame_condition_number=cond,
            valid=True,
        )

    def _estimate_target_velocities(
        self,
        target_positions: dict[int, np.ndarray],
        t_meas: float | None,
    ) -> dict[int, np.ndarray] | None:
        velocities = None
        if self._last_target_positions is not None and self._last_t_meas is not None:
            if t_meas is not None:
                dt = float(t_meas) - self._last_t_meas
            else:
                dt = 0.0
            if dt > 1e-9:
                velocities = {}
                for drone_id, target in target_positions.items():
                    previous = self._last_target_positions.get(drone_id)
                    if previous is None:
                        continue
                    velocities[drone_id] = (target - previous) / dt

        self._last_target_positions = {
            drone_id: np.array(target, dtype=float).copy()
            for drone_id, target in target_positions.items()
        }
        self._last_t_meas = float(t_meas) if t_meas is not None else None
        return velocities

    def _apply_delay_compensation(
        self,
        target_positions: dict[int, np.ndarray],
        target_velocities: dict[int, np.ndarray] | None,
    ) -> dict[int, np.ndarray]:
        if (
            not self.time_delay_compensation_enabled
            or target_velocities is None
            or self.estimated_total_delay_s <= 0.0
        ):
            return target_positions

        compensated = {}
        horizon = self.estimated_total_delay_s * self.delay_prediction_gain
        for drone_id, target in target_positions.items():
            velocity = target_velocities.get(drone_id)
            if velocity is None:
                compensated[drone_id] = target
                continue
            compensated[drone_id] = target + np.array(velocity, dtype=float) * horizon
        return compensated
