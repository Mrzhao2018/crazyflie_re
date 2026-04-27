"""Inter-drone separation safety hook contracts."""

import numpy as np

from src.config.schema import SafetyConfig
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyManager
from src.tests.run_real_fixtures import FakeFleet


positions = np.array(
    [
        [0.0, 0.0, 0.8],
        [0.05, 0.0, 0.8],
        [1.0, 0.0, 0.8],
        [0.0, 1.0, 0.8],
        [0.0, 0.0, 1.2],
        [0.5, 0.5, 0.8],
    ],
    dtype=float,
)
snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=positions,
    fresh_mask=np.ones(6, dtype=bool),
    disconnected_ids=[],
)

cfg = SafetyConfig(
    boundary_min=[-2.0, -2.0, -0.5],
    boundary_max=[2.0, 2.0, 2.5],
    pose_timeout=1.0,
    max_condition_number=100.0,
    min_inter_drone_distance=0.2,
    inter_drone_separation_action="telemetry",
)
decision = SafetyManager(cfg, FakeFleet()).evaluate(snapshot)
assert decision.action == "EXECUTE"
assert "INTER_DRONE_SEPARATION" in decision.reason_codes

cfg_hold = SafetyConfig(
    boundary_min=[-2.0, -2.0, -0.5],
    boundary_max=[2.0, 2.0, 2.5],
    pose_timeout=1.0,
    max_condition_number=100.0,
    min_inter_drone_distance=0.2,
    inter_drone_separation_action="hold",
)
hold_decision = SafetyManager(cfg_hold, FakeFleet()).evaluate(snapshot)
assert hold_decision.action == "HOLD"
assert hold_decision.structured_reasons[0].details["pair"] == [1, 2]

print("[OK] inter-drone separation safety hooks verified")
