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

health = {5: HealthSample(t_meas=0.0, values={"pm.vbat": 4.0})}
decision = safety.evaluate(snapshot, health=health)
assert decision.action == "EXECUTE"
assert "LOW_BATTERY" not in decision.reason_codes

transient_low = {5: HealthSample(t_meas=1.0, values={"pm.vbat": 2.95})}
decision = safety.evaluate(snapshot, health=transient_low)
assert decision.action == "EXECUTE"
assert "LOW_BATTERY" not in decision.reason_codes

decision = safety.evaluate(
    snapshot,
    health={5: HealthSample(t_meas=2.0, values={"pm.vbat": 2.95})},
    health_window={
        5: [
            HealthSample(t_meas=0.0, values={"pm.vbat": 4.0}),
            HealthSample(t_meas=0.5, values={"pm.vbat": 4.0}),
            HealthSample(t_meas=1.0, values={"pm.vbat": 4.0}),
            HealthSample(t_meas=1.5, values={"pm.vbat": 2.95}),
            HealthSample(t_meas=2.0, values={"pm.vbat": 2.95}),
        ]
    },
)
assert decision.action == "EXECUTE"
assert "LOW_BATTERY" not in decision.reason_codes

decision = safety.evaluate(
    snapshot,
    health={5: HealthSample(t_meas=3.0, values={"pm.vbat": 2.95})},
    health_window={
        5: [
            HealthSample(t_meas=float(i), values={"pm.vbat": 2.95})
            for i in range(config.safety.min_vbat_abort_samples)
        ]
    },
)
assert decision.action == "ABORT"
assert "LOW_BATTERY" in decision.reason_codes
reason = next(r for r in decision.structured_reasons if r.code == "LOW_BATTERY")
assert reason.details["window_median"] == 2.95
assert reason.details["window_sample_count"] == config.safety.min_vbat_abort_samples

critical_safety = SafetyManager(config.safety, fleet)
decision = critical_safety.evaluate(
    snapshot,
    health={5: HealthSample(t_meas=4.0, values={"pm.vbat": config.safety.min_vbat_critical})},
)
assert decision.action == "ABORT"
assert "LOW_BATTERY" in decision.reason_codes

assert config.safety.hold_auto_land_timeout > 0

print("[OK] SafetyManager decision contracts verified")
