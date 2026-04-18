"""Follower参考生成器 - 从leader实测位置恢复affine frame"""

import numpy as np
from dataclasses import dataclass


@dataclass
class FollowerReferenceSet:
    """Follower参考集合"""

    follower_ids: list[int]
    target_positions: dict  # {drone_id: np.ndarray}
    target_velocities: dict | None
    target_accelerations: dict | None
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
        self._last_target_positions: np.ndarray | None = None
        self._last_target_ids: tuple[int, ...] | None = None
        self._last_target_velocities: np.ndarray | None = None
        self._last_target_velocity_ids: tuple[int, ...] | None = None
        self._last_t_meas: float | None = None

    def compute(
        self, leader_measurements: dict, t_meas: float | None = None
    ) -> FollowerReferenceSet:
        """从leader实测位置计算follower目标

        使用AFC稳态解：p_f* = -Omega_ff^{-1} Omega_fl p_l

        Rank / condition number 的权威判定在 AffineFrameEstimator；
        这里只保留 NaN 防御以避免 estimator 被误绕过。
        """
        leader_ids = self.formation.leader_ids
        if any(lid not in leader_measurements for lid in leader_ids):
            return FollowerReferenceSet([], {}, None, None, float("inf"), False)

        leader_pos_array = np.array([leader_measurements[lid] for lid in leader_ids])

        if len(leader_pos_array) < 4:
            return FollowerReferenceSet([], {}, None, None, float("inf"), False)

        if np.isnan(leader_pos_array).any():
            return FollowerReferenceSet([], {}, None, None, float("inf"), False)

        # 计算稳态（不再在此复算 rank/cond；调用方必须先过 frame.valid）
        target_positions = self.afc.steady_state(leader_measurements)
        target_velocities = self._estimate_target_velocities(target_positions, t_meas)
        target_accelerations = self._estimate_target_accelerations(
            target_velocities,
            t_meas,
        )
        target_positions = self._apply_delay_compensation(
            target_positions,
            target_velocities,
        )

        return FollowerReferenceSet(
            follower_ids=list(target_positions.keys()),
            target_positions=target_positions,
            target_velocities=target_velocities,
            target_accelerations=target_accelerations,
            frame_condition_number=float("nan"),
            valid=True,
        )

    def _estimate_target_velocities(
        self,
        target_positions: dict[int, np.ndarray],
        t_meas: float | None,
    ) -> dict[int, np.ndarray] | None:
        velocities: dict[int, np.ndarray] | None = None
        follower_ids = list(target_positions.keys())
        current = (
            np.stack(
                [np.asarray(target_positions[fid], dtype=float) for fid in follower_ids]
            )
            if follower_ids
            else np.zeros((0, 3))
        )

        if (
            self._last_target_positions is not None
            and self._last_target_ids == tuple(follower_ids)
            and self._last_t_meas is not None
            and t_meas is not None
        ):
            dt = float(t_meas) - self._last_t_meas
            if dt > 1e-9:
                vel_arr = (current - self._last_target_positions) / dt
                velocities = {
                    fid: vel_arr[row] for row, fid in enumerate(follower_ids)
                }

        # Freeze 当前 ids/positions 作为下次参考
        self._last_target_positions = current.copy()
        self._last_target_ids = tuple(follower_ids)
        return velocities

    def _estimate_target_accelerations(
        self,
        target_velocities: dict[int, np.ndarray] | None,
        t_meas: float | None,
    ) -> dict[int, np.ndarray] | None:
        accelerations: dict[int, np.ndarray] | None = None
        if target_velocities is None:
            self._last_target_velocities = None
            self._last_target_velocity_ids = None
            self._last_t_meas = float(t_meas) if t_meas is not None else None
            return None

        follower_ids = list(target_velocities.keys())
        current = (
            np.stack(
                [np.asarray(target_velocities[fid], dtype=float) for fid in follower_ids]
            )
            if follower_ids
            else np.zeros((0, 3))
        )

        if (
            self._last_target_velocities is not None
            and self._last_target_velocity_ids == tuple(follower_ids)
            and self._last_t_meas is not None
            and t_meas is not None
        ):
            dt = float(t_meas) - self._last_t_meas
            if dt > 1e-9:
                acc_arr = (current - self._last_target_velocities) / dt
                accelerations = {
                    fid: acc_arr[row] for row, fid in enumerate(follower_ids)
                }

        self._last_target_velocities = current.copy()
        self._last_target_velocity_ids = tuple(follower_ids)
        self._last_t_meas = float(t_meas) if t_meas is not None else None
        return accelerations

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
