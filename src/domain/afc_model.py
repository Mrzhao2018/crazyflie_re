"""AFC模型 - 稳态解、误差、收敛率计算"""

import numpy as np
from dataclasses import dataclass


@dataclass
class ConvergenceReport:
    """收敛性报告"""

    min_eigenvalue: float
    convergence_rate: float
    is_stable: bool


class AFCModel:
    """仿射编队控制数学模型"""

    def __init__(self, stress_result, fleet_model):
        self.Omega = stress_result.omega
        self.Omega_ff = stress_result.omega_ff
        self.Omega_fl = stress_result.omega_fl
        self.fleet = fleet_model

        # 预计算Omega_ff的逆
        try:
            self.Omega_ff_inv = np.linalg.inv(self.Omega_ff)
        except np.linalg.LinAlgError:
            raise ValueError("Omega_ff不可逆")

    def steady_state_array(
        self, leader_positions: dict
    ) -> tuple[np.ndarray, tuple[int, ...]]:
        """返回 (p_f_star[n_f, 3], follower_ids)。

        数值与 `steady_state` 一致，区别是不做字典化，便于调用方继续走向量运算。
        """
        leader_ids = self.fleet.leader_ids()
        p_l = np.array([leader_positions[lid] for lid in leader_ids])
        p_f_star = -self.Omega_ff_inv @ self.Omega_fl @ p_l
        return p_f_star, tuple(self.fleet.follower_ids())

    def steady_state(self, leader_positions: dict) -> dict:
        """计算follower稳态位置

        p_f* = -Omega_ff^{-1} Omega_fl p_l
        """
        p_f_star, follower_ids = self.steady_state_array(leader_positions)
        return {fid: p_f_star[i] for i, fid in enumerate(follower_ids)}

    def formation_error(self, current_positions: dict, leader_positions: dict) -> float:
        """计算编队误差"""
        steady = self.steady_state(leader_positions)

        error = 0.0
        for fid, p_target in steady.items():
            if fid in current_positions:
                error += np.linalg.norm(current_positions[fid] - p_target) ** 2

        return np.sqrt(error)

    def convergence_report(self) -> ConvergenceReport:
        """计算收敛性报告"""
        eigs = np.linalg.eigvalsh(self.Omega_ff)
        min_eig = eigs[0]

        return ConvergenceReport(
            min_eigenvalue=min_eig, convergence_rate=min_eig, is_stable=(min_eig > 0)
        )
