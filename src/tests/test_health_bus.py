"""HealthBus contract tests"""

from src.runtime.health_bus import HealthBus


bus = HealthBus()
assert bus.latest() == {}

bus.update(1, {"pm.vbat": 4.1}, 1.0)
latest = bus.latest()
assert 1 in latest
assert latest[1].values["pm.vbat"] == 4.1
assert latest[1].safety_t_meas == 1.0

bus.update(2, {"pm.vbat": 3.9}, 2.0)
latest = bus.latest()
assert latest[2].t_meas == 2.0

bus.update(1, {"stateEstimate.roll": 3.0}, 3.0)
latest = bus.latest()
assert latest[1].t_meas == 3.0
assert latest[1].safety_t_meas == 1.0

print("[OK] HealthBus contracts verified")
