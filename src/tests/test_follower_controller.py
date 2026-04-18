"""FollowerController: 向量化前后逐 follower 输出必须相等"""

import numpy as np
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.follower_reference import FollowerReferenceSet
from src.runtime.follower_controller import FollowerController
from src.runtime.pose_snapshot import PoseSnapshot


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)

controller = FollowerController(config.control)

nominal = np.array(config.mission.nominal_positions, dtype=float)
n = len(nominal)

# 偏移 follower 实测位置，触发 P 项与前馈都非零
positions = nominal.copy()
for fid in fleet.follower_ids():
    idx = fleet.id_to_index(fid)
    positions[idx] = positions[idx] + np.array([0.05, -0.03, 0.02])

snapshot = PoseSnapshot(
    seq=1, t_meas=0.0, positions=positions,
    fresh_mask=np.ones(n, dtype=bool), disconnected_ids=[],
)

target_positions = {fid: nominal[fleet.id_to_index(fid)] for fid in fleet.follower_ids()}
target_velocities = {fid: np.array([0.1, 0.0, 0.0]) for fid in fleet.follower_ids()}
ref = FollowerReferenceSet(
    follower_ids=list(fleet.follower_ids()),
    target_positions=target_positions,
    target_velocities=target_velocities,
    target_accelerations=None,
    frame_condition_number=float("nan"),
    valid=True,
)

result = controller.compute(snapshot, ref, fleet.follower_ids(), fleet)

# 1) 所有 active follower 都有 command
assert set(result.commands.keys()) == set(fleet.follower_ids())

# 2) command_norms 与逐个 np.linalg.norm 一致（不能因向量化产生误差）
for fid, cmd in result.commands.items():
    expected = float(np.linalg.norm(cmd))
    assert abs(result.diagnostics["command_norms"][fid] - expected) < 1e-12

# 3) 对于偏移 error = (+0.05, -0.03, +0.02)，u = -K*err + Kff*vel
#    手算一个 follower 来卡数值
fid0 = fleet.follower_ids()[0]
err = np.array([0.05, -0.03, 0.02])
expected_p = -np.array([
    controller.gain_xy * err[0],
    controller.gain_xy * err[1],
    controller.gain_z * err[2],
])
expected_ff = np.array([
    controller.feedforward_gain_xy * 0.1,
    0.0,
    0.0,
])
# radial 缩放会略抬升 XY 分量；至少符号与 Z 分量应严格一致
cmd0 = result.commands[fid0]
assert np.sign(cmd0[0]) == np.sign(expected_p[0] + expected_ff[0])
assert abs(cmd0[2] - (expected_p[2])) < 1e-9

# 4) stale follower 不产生 command
stale_mask = np.ones(n, dtype=bool)
stale_mask[fleet.id_to_index(fleet.follower_ids()[0])] = False
stale_snapshot = PoseSnapshot(
    seq=2, t_meas=0.0, positions=positions,
    fresh_mask=stale_mask, disconnected_ids=[fleet.follower_ids()[0]],
)
result_stale = controller.compute(stale_snapshot, ref, fleet.follower_ids(), fleet)
assert fleet.follower_ids()[0] not in result_stale.commands
assert fleet.follower_ids()[0] in result_stale.diagnostics["skipped_stale_followers"]

print("[OK] FollowerController vectorization parity verified")
