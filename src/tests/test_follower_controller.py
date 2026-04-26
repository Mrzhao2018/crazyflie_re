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

# 3) 对于偏移 error = (+0.05, -0.03, +0.02)，u = -K*err + Kff*vel，
#    且 XY 会叠加径向缩放；手算一个 follower 来卡数值
fid0 = fleet.follower_ids()[0]
err = np.array([0.05, -0.03, 0.02])
target_radii = {
    fid: float(np.linalg.norm(np.asarray(ref.target_positions[fid], dtype=float)[:2]))
    for fid in fleet.follower_ids()
}
max_radius = max(target_radii.values())
radius_ratio = target_radii[fid0] / max_radius if max_radius > 1e-9 else 0.0
gain_scale_xy = 1.0 + controller.radial_gain_scale_xy * radius_ratio
ff_scale_xy = 1.0 + controller.radial_feedforward_scale_xy * radius_ratio
expected_cmd0 = np.array([
    -(controller.gain_xy * gain_scale_xy) * err[0]
    + controller.feedforward_gain_xy * ff_scale_xy * 0.1,
    -(controller.gain_xy * gain_scale_xy) * err[1],
    -controller.gain_z * err[2],
])
cmd0 = result.commands[fid0]
assert np.allclose(cmd0, expected_cmd0, atol=1e-12)

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
