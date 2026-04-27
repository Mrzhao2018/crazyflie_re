"""Stress solver mode contracts."""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.domain.stress_matrix_solver import StressMatrixSolver


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
formation = FormationModel(
    np.array(config.mission.nominal_positions, dtype=float),
    fleet.leader_ids(),
    fleet,
)
solver = StressMatrixSolver(formation)

dense = solver.solve(fleet.leader_ids(), mode="dense_current")
sparse = solver.solve(fleet.leader_ids(), mode="sparse_convex")

assert dense.metadata["mode"] == "dense_current"
assert sparse.metadata["mode"] == "sparse_convex"
assert sparse.metadata["sparsity"] >= dense.metadata["sparsity"]
assert sparse.n_edges <= dense.n_edges
assert sparse.min_eig_ff > 0.0

try:
    solver.solve(fleet.leader_ids(), mode="clustered")
    raise AssertionError("clustered mode should be reserved, not silently run")
except NotImplementedError:
    pass

print("[OK] stress solver modes verified")
