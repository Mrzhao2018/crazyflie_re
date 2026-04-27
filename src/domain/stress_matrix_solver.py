"""应力矩阵求解器 - 从archive迁移"""

import numpy as np
import importlib
from scipy.linalg import null_space
from dataclasses import dataclass, field


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
    metadata: dict = field(default_factory=dict)


class StressMatrixSolver:
    """应力矩阵求解器"""

    def __init__(self, formation_model):
        self.formation = formation_model

    def solve(self, leader_ids: list[int], mode: str = "dense_current") -> StressMatrixResult:
        if mode == "dense_current":
            result = self.solve_dense(leader_ids)
            result.metadata["mode"] = mode
            return result
        if mode == "sparse_convex":
            return self.solve_sparse_convex(leader_ids)
        if mode == "clustered":
            raise NotImplementedError("clustered stress solver mode is reserved")
        raise ValueError(f"unknown stress solver mode: {mode}")

    def solve_dense(self, leader_ids: list[int]) -> StressMatrixResult:
        """求解dense应力矩阵"""
        n = len(self.formation.nominal_positions)
        adj = np.ones((n, n)) - np.eye(n)
        result = self._solve_from_adjacency(adj, leader_ids)
        result.metadata["mode"] = result.metadata.get("mode", "dense_current")
        return result

    def solve_sparse_convex(self, leader_ids: list[int]) -> StressMatrixResult:
        """Sparse candidate mode using progressively denser k-nearest graphs.

        The mainline dense solver remains the default. This mode is intentionally
        conservative: it tries sparse graphs first and falls back to dense when a
        positive follower block cannot be certified.
        """
        n = len(self.formation.nominal_positions)
        dense_result = self.solve_dense(leader_ids)
        dense_edges = dense_result.n_edges
        for k in range(min(4, n - 1), n):
            adj = self._knn_adjacency(k)
            try:
                result = self._solve_from_adjacency(adj, leader_ids)
            except ValueError:
                continue
            if result.min_eig_ff > 0:
                result.metadata["mode"] = "sparse_convex"
                result.metadata["dense_edge_count"] = dense_edges
                return result
        dense_result.metadata["mode"] = "sparse_convex"
        dense_result.metadata["fallback"] = "dense_current"
        dense_result.metadata["dense_edge_count"] = dense_edges
        return dense_result

    def _solve_from_adjacency(
        self, adj: np.ndarray, leader_ids: list[int]
    ) -> StressMatrixResult:
        n = len(self.formation.nominal_positions)
        positions = self.formation.nominal_positions
        edges = self._get_edges(adj)
        C = self._build_constraint_matrix(positions, edges)
        N = null_space(C)
        null_dim = N.shape[1]

        if null_dim == 0:
            raise ValueError("不存在可行的应力矩阵")

        return self._solve_sdp(N, edges, n, leader_ids, null_dim)

    def _knn_adjacency(self, k: int) -> np.ndarray:
        positions = self.formation.nominal_positions
        n = len(positions)
        adj = np.zeros((n, n), dtype=float)
        for i in range(n):
            distances = [
                (float(np.linalg.norm(positions[i] - positions[j])), j)
                for j in range(n)
                if j != i
            ]
            for _dist, j in sorted(distances)[:k]:
                adj[i, j] = 1.0
                adj[j, i] = 1.0
        return adj

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

        possible_edges = n * (n - 1) / 2
        return StressMatrixResult(
            omega=Omega,
            omega_ff=Omega_ff,
            omega_fl=Omega_fl,
            min_eig_ff=eigs[0],
            condition_number=cond,
            null_dim=null_dim,
            n_edges=n_edges,
            metadata={
                "edge_count": n_edges,
                "min_eig_ff": float(eigs[0]),
                "condition_number": float(cond),
                "sparsity": 1.0 - (float(n_edges) / possible_edges),
            },
        )
