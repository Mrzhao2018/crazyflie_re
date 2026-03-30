"""SafetyManager decision tests"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.health_bus import HealthSample
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyManager
from src.runtime.follower_controller import FollowerCommandSet


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
safety = SafetyManager(config.safety, fleet)
nominal = np.array(config.mission.nominal_positions, dtype=float)

snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)

decision = safety.evaluate(snapshot)
assert decision.action == "EXECUTE"
assert decision.reason_codes == []

bad_snapshot = PoseSnapshot(
    seq=2,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[1],
)
decision = safety.evaluate(bad_snapshot)
assert decision.action == "ABORT"
assert "DISCONNECTED" in decision.reason_codes

commands = FollowerCommandSet(commands={5: np.array([3.0, 0.0, 0.0])}, diagnostics={})
decision = safety.evaluate(snapshot, commands=commands)
assert decision.action == "HOLD"
assert "COMMAND_SATURATED" in decision.reason_codes

health = {5: HealthSample(t_meas=0.0, values={"pm.vbat": 3.0})}
decision = safety.evaluate(snapshot, health=health)
assert decision.action == "ABORT"
assert "LOW_BATTERY" in decision.reason_codes

print("[OK] SafetyManager decision contracts verified")
