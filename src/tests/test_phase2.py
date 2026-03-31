"""测试第二阶段算法层"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.domain.stress_matrix_solver import StressMatrixSolver
from src.domain.afc_model import AFCModel
from src.domain.mission_profile import MissionProfile
from src.domain.leader_reference import LeaderReferenceGenerator
from src.domain.follower_reference import FollowerReferenceGenerator
from src.runtime.affine_frame_estimator import AffineFrameEstimator
from src.runtime.follower_controller import FollowerController
from src.runtime.pose_snapshot import PoseSnapshot

print("=== 加载配置 ===")
config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
nominal = np.array(config.mission.nominal_positions)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)
mission_profile = MissionProfile(config.mission)
print(f"[OK] Fleet: {len(fleet.all_ids())} drones")

print("\n=== 测试StressMatrixSolver ===")
solver = StressMatrixSolver(formation)
result = solver.solve_dense(fleet.leader_ids())
print(f"[OK] Omega_ff min_eig: {result.min_eig_ff:.4f}")
print(f"[OK] Condition number: {result.condition_number:.2f}")

print("\n=== 测试AFCModel ===")
afc = AFCModel(result, fleet)
conv = afc.convergence_report()
print(f"[OK] Stable: {conv.is_stable}, rate: {conv.convergence_rate:.4f}")

print("\n=== 测试LeaderReferenceGenerator ===")
leader_gen = LeaderReferenceGenerator(mission_profile, formation, fleet)
leader_ref = leader_gen.reference_at(0.0)
print(f"[OK] Generated {len(leader_ref.positions)} leader references")

print("\n=== 测试FollowerReferenceGenerator ===")
follower_gen = FollowerReferenceGenerator(formation, afc)
follower_ref = follower_gen.compute(leader_ref.positions)
print(
    f"[OK] Valid: {follower_ref.valid}, cond: {follower_ref.frame_condition_number:.2f}"
)
print(f"[OK] Generated {len(follower_ref.target_positions)} follower targets")

print("\n=== 测试AffineFrameEstimator ===")
snapshot = PoseSnapshot(0, 0.0, nominal, np.ones(len(nominal), dtype=bool), [])
estimator = AffineFrameEstimator(fleet)
frame = estimator.estimate(snapshot, fleet.leader_ids())
print(f"[OK] Frame valid: {frame.valid}, cond: {frame.condition_number:.2f}")
assert frame.valid == follower_ref.valid
assert abs(frame.condition_number - follower_ref.frame_condition_number) < 1e-6

print("\n=== 测试FollowerController ===")
controller = FollowerController(config.control)
commands = controller.compute(snapshot, follower_ref, fleet.follower_ids(), fleet)
print(f"[OK] Generated {len(commands.commands)} velocity commands")

print("\n[SUCCESS] 第二阶段算法层测试通过！")
