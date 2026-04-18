"""PoseBus: 连续多次 latest() 不应互相污染"""

import time
import numpy as np
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.pose_bus import PoseBus


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
pose_bus = PoseBus(fleet, pose_timeout=1.0)

drone_ids = list(fleet.all_ids())

# 1) 首次 update + latest
for d in drone_ids:
    pose_bus.update_agent(d, np.array([1.0, 2.0, 3.0]), time.time())
snap_a = pose_bus.latest()
assert snap_a is not None
positions_a_copy = snap_a.positions.copy()

# 2) 改一个 agent，再次 latest：旧 snapshot 的 positions 不应被改到
idx0 = fleet.id_to_index(drone_ids[0])
pose_bus.update_agent(drone_ids[0], np.array([9.0, 9.0, 9.0]), time.time())
snap_b = pose_bus.latest()
assert snap_b is not None

assert np.allclose(snap_a.positions, positions_a_copy), (
    "旧 snapshot 的 positions 在下一次 latest() 之后被改动了，说明 buffer 未隔离"
)
assert np.allclose(snap_b.positions[idx0], np.array([9.0, 9.0, 9.0]))
assert not np.allclose(snap_b.positions[idx0], snap_a.positions[idx0])

# 3) fresh_mask 过期行为
old_pose_bus = PoseBus(fleet, pose_timeout=0.001)
for d in drone_ids:
    old_pose_bus.update_agent(d, np.array([0.0, 0.0, 0.0]), time.time() - 10.0)
snap_c = old_pose_bus.latest()
assert snap_c is not None
assert not snap_c.fresh_mask.any()

print("[OK] PoseBus buffer isolation verified")
