"""Affine frame estimator tests"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.affine_frame_estimator import AffineFrameEstimator


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
estimator = AffineFrameEstimator(fleet)
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

stale_mask = np.ones(len(nominal), dtype=bool)
stale_mask[0] = False
stale_snapshot = PoseSnapshot(
    seq=2,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=stale_mask,
    disconnected_ids=[1],
)
stale_frame = estimator.estimate(stale_snapshot, fleet.leader_ids())
assert not stale_frame.valid

print("[OK] Affine frame validity contracts verified")
