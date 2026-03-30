"""应力矩阵求解器 - 从archive迁移"""

import importlib
import numpy as np
from scipy.linalg import null_space
from dataclasses import dataclass


@dataclass
class StressMatrixResult:
    """应力矩阵求解结果"""

    omega: np.ndarray  # 完整应力矩阵 (n, n)
    omega_ff: np.ndarray  # follower子矩阵
    omega_fl: np.ndarray  # follower-leader耦合矩阵
    min_eig_ff: float  # Omega_ff最小特征值
    condition_number: float | None
    null_dim: int
    n_edges: int


class StressMatrixSolver:
    """应力矩阵求解器"""

    def __init__(self, formation_model):
        self.formation = formation_model

    def solve_dense(self, leader_ids: list[int]) -> StressMatrixResult:
        """求解dense应力矩阵"""
        # 获取位置
        n = len(self.formation.nominal_positions)
        positions = self.formation.nominal_positions

        # 构建全连接邻接矩阵
        adj = np.ones((n, n)) - np.eye(n)

        # 提取边
        edges = self._get_edges(adj)

        # 构建约束矩阵
        C = self._build_constraint_matrix(positions, edges)

        # 求零空间
        N = null_space(C)
        null_dim = N.shape[1]

        if null_dim == 0:
            raise ValueError("不存在可行的应力矩阵")

        # 优化求解
        return self._solve_sdp(N, edges, n, leader_ids, null_dim)

    def _get_edges(self, adj_matrix):
        """提取无向边列表"""
        n = adj_matrix.shape[0]
        edges = []
        for i in range(n):
            for j in range(i + 1, n):
                if adj_matrix[i, j] > 0:
                    edges.append((i, j))
        return edges

    def _build_constraint_matrix(self, positions, edges):
        """构建约束矩阵 C"""
        n, d = positions.shape
        m = len(edges)
        C = np.zeros((n * d, m))

        for e_idx, (i, j) in enumerate(edges):
            for k in range(d):
                C[i * d + k, e_idx] = positions[j, k] - positions[i, k]
                C[j * d + k, e_idx] = positions[i, k] - positions[j, k]

        return C

    def _solve_sdp(self, N, edges, n, leader_ids, null_dim):
        """使用SDP优化求解"""
        cp = self._load_cvxpy()
        if cp is None:
            return self._solve_random(N, edges, n, leader_ids, null_dim)

        # 获取follower索引
        follower_indices = [
            i for i in range(n) if self.formation.fleet.index_to_id(i) not in leader_ids
        ]
        n_f = len(follower_indices)

        # 预计算每个零空间基向量对应的Omega_ff
        F_list = []
        for k in range(null_dim):
            Omega_k = self._weights_to_stress_matrix(N[:, k], edges, n)
            F_k = Omega_k[np.ix_(follower_indices, follower_indices)]
            F_list.append(F_k)

        # 定义优化变量
        c = cp.Variable(null_dim)
        t = cp.Variable()

        # 构建Omega_ff = sum(c_k * F_k)
        Omega_ff_expr = sum(c[k] * F_list[k] for k in range(null_dim))

        # 约束
        constraints = [
            Omega_ff_expr >> t * np.eye(n_f),  # 正定约束
            cp.norm(c) <= 1,
            t >= 0.01,
        ]

        # 目标：最大化最小特征值
        prob = cp.Problem(cp.Maximize(t), constraints)
        prob.solve(solver=cp.SCS, verbose=False)

        if prob.status not in ["optimal", "optimal_inaccurate"]:
            return self._solve_random(N, edges, n, leader_ids, null_dim)

        # 构建最终应力矩阵
        weights = N @ c.value
        Omega = self._weights_to_stress_matrix(weights, edges, n)

        return self._build_result(Omega, follower_indices, null_dim, len(edges))

    @staticmethod
    def _load_cvxpy():
        try:
            return importlib.import_module("cvxpy")
        except ImportError:
            return None

    def _solve_random(self, N, edges, n, leader_ids, null_dim):
        """随机搜索回退方案"""
        follower_indices = [
            i for i in range(n) if self.formation.fleet.index_to_id(i) not in leader_ids
        ]

        best_omega = None
        best_min_eig = -np.inf

        for _ in range(100):
            c = np.random.randn(null_dim)
            c = c / np.linalg.norm(c)
            weights = N @ c
            Omega = self._weights_to_stress_matrix(weights, edges, n)
            Omega_ff = Omega[np.ix_(follower_indices, follower_indices)]

            eigs = np.linalg.eigvalsh(Omega_ff)
            min_eig = eigs[0]

            if min_eig > best_min_eig:
                best_min_eig = min_eig
                best_omega = Omega

        return self._build_result(best_omega, follower_indices, null_dim, len(edges))

    def _weights_to_stress_matrix(self, weights, edges, n):
        """边权重转应力矩阵"""
        Omega = np.zeros((n, n))
        for w_val, (i, j) in zip(weights, edges):
            Omega[i, j] = w_val
            Omega[j, i] = w_val
        for i in range(n):
            Omega[i, i] = -np.sum(Omega[i, :])
        return Omega

    def _build_result(self, Omega, follower_indices, null_dim, n_edges):
        """构建结果对象"""
        n = Omega.shape[0]
        leader_indices = [i for i in range(n) if i not in follower_indices]

        Omega_ff = Omega[np.ix_(follower_indices, follower_indices)]
        Omega_fl = Omega[np.ix_(follower_indices, leader_indices)]

        eigs = np.linalg.eigvalsh(Omega_ff)
        cond = np.linalg.cond(Omega_ff)

        return StressMatrixResult(
            omega=Omega,
            omega_ff=Omega_ff,
            omega_fl=Omega_fl,
            min_eig_ff=eigs[0],
            condition_number=cond,
            null_dim=null_dim,
            n_edges=n_edges,
        )
