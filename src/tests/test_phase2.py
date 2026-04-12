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
config.mission.leader_motion.trajectory_enabled = False
config.control.gain_xy = 1.2
config.control.gain_z = 0.4
config.control.feedforward_gain_xy = 0.6
config.control.feedforward_gain_z = 0.2
config.control.max_feedforward_velocity_xy = 0.2
config.control.max_feedforward_velocity_z = 0.05
config.control.radial_gain_scale_xy = 0.2
config.control.radial_feedforward_scale_xy = 0.1
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
follower_ref = follower_gen.compute(leader_ref.positions, 0.0)
print(
    f"[OK] Valid: {follower_ref.valid}, cond: {follower_ref.frame_condition_number:.2f}"
)
print(f"[OK] Generated {len(follower_ref.target_positions)} follower targets")
follower_ref_next = follower_gen.compute(
    {lid: pos + np.array([0.1, 0.0, 0.0]) for lid, pos in leader_ref.positions.items()},
    0.1,
)
assert follower_ref_next.target_velocities is not None

delay_follower_gen = FollowerReferenceGenerator(
    formation,
    afc,
    config.safety.max_condition_number,
    time_delay_compensation_enabled=True,
    estimated_total_delay_ms=100.0,
    delay_prediction_gain=1.0,
)
delay_follower_gen.compute(leader_ref.positions, 0.0)
delay_follower_ref = delay_follower_gen.compute(
    {lid: pos + np.array([0.1, 0.0, 0.0]) for lid, pos in leader_ref.positions.items()},
    0.1,
)
assert delay_follower_ref.valid is True
assert delay_follower_ref.target_velocities is not None
sample_follower_id = next(iter(delay_follower_ref.target_positions))
baseline_follower_ref = follower_gen.compute(
    {lid: pos + np.array([0.1, 0.0, 0.0]) for lid, pos in leader_ref.positions.items()},
    0.2,
)
assert (
    delay_follower_ref.target_positions[sample_follower_id][0]
    >= baseline_follower_ref.target_positions[sample_follower_id][0]
)

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
commands_ff = controller.compute(
    snapshot, follower_ref_next, fleet.follower_ids(), fleet
)
assert commands_ff.diagnostics["feedforward_followers"]
assert commands_ff.diagnostics["radial_scaled_followers"]
sample_cmd = next(iter(commands_ff.commands.values()))
assert abs(sample_cmd[2]) <= config.control.max_velocity

print("\n[SUCCESS] 第二阶段算法层测试通过！")
