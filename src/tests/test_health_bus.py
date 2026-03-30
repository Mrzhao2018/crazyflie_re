"""HealthBus contract tests"""

from src.runtime.health_bus import HealthBus


bus = HealthBus()
assert bus.latest() == {}

bus.update(1, {"pm.vbat": 4.1}, 1.0)
latest = bus.latest()
assert 1 in latest
assert latest[1].values["pm.vbat"] == 4.1

print("[OK] HealthBus contracts verified")
