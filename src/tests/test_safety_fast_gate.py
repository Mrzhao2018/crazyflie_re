"""SafetyManager.fast_gate: 轻量 pre-check 只看 disconnected + boundary"""

import numpy as np
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyManager


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
safety = SafetyManager(config.safety, fleet)

n = len(fleet.all_ids())
positions = np.zeros((n, 3))
positions[:, 2] = 0.5

# 1) 正常：不拦截
snapshot = PoseSnapshot(
    seq=1, t_meas=0.0, positions=positions,
    fresh_mask=np.ones(n, dtype=bool), disconnected_ids=[],
)
blocked, reasons = safety.fast_gate(snapshot)
assert not blocked
assert reasons == []

# 2) 断连：拦截
snap_disc = PoseSnapshot(
    seq=2, t_meas=0.0, positions=positions,
    fresh_mask=np.ones(n, dtype=bool),
    disconnected_ids=[fleet.all_ids()[0]],
)
blocked, reasons = safety.fast_gate(snap_disc)
assert blocked
assert any("DISCONNECTED" in r for r in reasons)

# 3) 越界：拦截
oob_positions = positions.copy()
oob_positions[0] = np.array([999.0, 999.0, 999.0])
snap_oob = PoseSnapshot(
    seq=3, t_meas=0.0, positions=oob_positions,
    fresh_mask=np.ones(n, dtype=bool), disconnected_ids=[],
)
blocked, reasons = safety.fast_gate(snap_oob)
assert blocked
assert any("OUT_OF_BOUNDS" in r for r in reasons)

print("[OK] SafetyManager.fast_gate contracts verified")
