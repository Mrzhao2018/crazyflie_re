"""AFCModel.steady_state_array: 数组版与 dict 版数值一致"""

import numpy as np
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.domain.stress_matrix_solver import StressMatrixSolver
from src.domain.afc_model import AFCModel


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
nominal = np.array(config.mission.nominal_positions, dtype=float)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)
afc = AFCModel(StressMatrixSolver(formation).solve_dense(fleet.leader_ids()), fleet)

leader_measurements = {lid: nominal[fleet.id_to_index(lid)] for lid in fleet.leader_ids()}

dict_result = afc.steady_state(leader_measurements)
array_result, follower_ids = afc.steady_state_array(leader_measurements)

assert list(follower_ids) == list(fleet.follower_ids())
for row, fid in enumerate(follower_ids):
    assert np.allclose(array_result[row], dict_result[fid])

print("[OK] AFCModel.steady_state_array parity verified")
