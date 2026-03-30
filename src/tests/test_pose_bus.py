"""PoseBus contract tests"""

import time
import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.pose_bus import PoseBus


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
pose_bus = PoseBus(fleet, pose_timeout=0.01)

pose_bus.update_agent(1, np.array([0.0, 0.0, 0.0]), time.time())
snapshot1 = pose_bus.latest()
assert snapshot1 is not None
assert snapshot1.seq == 1
assert pose_bus.has_newer_than(0)

time.sleep(0.02)
snapshot2 = pose_bus.latest()
assert snapshot2 is not None
assert snapshot2.seq == 1
assert 1 in snapshot2.disconnected_ids

print("[OK] PoseBus freshness contracts verified")
