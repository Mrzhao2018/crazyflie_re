"""AffineFrameEstimator: 合并 SVD 后 rank/cond 与旧实现数值一致"""

import numpy as np
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.affine_frame_estimator import AffineFrameEstimator


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
estimator = AffineFrameEstimator(fleet)

# 1) 正常几何：valid + condition_number 有限
nominal = np.array(config.mission.nominal_positions, dtype=float)
snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
frame = estimator.estimate(snapshot, fleet.leader_ids())
assert frame.valid
assert np.isfinite(frame.condition_number)
assert frame.diagnostics["rank"] == 3

# 2) 退化几何：四个 leader 共面，rank=2 时 valid=False, cond=inf
leader_ids = fleet.leader_ids()
degenerate = nominal.copy()
for lid in leader_ids:
    idx = fleet.id_to_index(lid)
    degenerate[idx, 2] = 0.0  # 全部 z=0 → 3 维 diff 落在 2D 平面
snap_deg = PoseSnapshot(
    seq=2, t_meas=0.0, positions=degenerate,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
frame_deg = estimator.estimate(snap_deg, fleet.leader_ids())
assert not frame_deg.valid
assert frame_deg.diagnostics["rank"] < 3
assert frame_deg.condition_number == float("inf")

# 3) 稳定性：重复调用结果完全相同
frame_a = estimator.estimate(snapshot, fleet.leader_ids())
frame_b = estimator.estimate(snapshot, fleet.leader_ids())
assert frame_a.condition_number == frame_b.condition_number
assert frame_a.diagnostics["rank"] == frame_b.diagnostics["rank"]

print("[OK] AffineFrameEstimator SVD fusion contracts verified")
