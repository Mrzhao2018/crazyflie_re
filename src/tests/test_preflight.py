"""Preflight structured report tests"""

import numpy as np

from src.app.preflight import PreflightRunner
from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.runtime.health_bus import HealthBus
from src.runtime.pose_snapshot import PoseSnapshot


class FakePoseBus:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def latest(self):
        return self._snapshot


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
nominal = np.array(config.mission.nominal_positions, dtype=float)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)

snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)

runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in fleet.leader_ids()
            }
        },
    }
)
for drone_id in fleet.all_ids():
    runner.comp["health_bus"].update(drone_id, {"pm.vbat": 4.0}, 0.0)
report = runner.run()
assert report.ok is True
assert report.failed_codes == []
leader_count_check = next(check for check in report.checks if check.code == "LEADER_COUNT")
assert leader_count_check.passed is True
assert any(check.code == "AFFINE_SPAN" for check in report.checks)
assert any(check.code == "TRAJECTORY_READY" for check in report.checks)

bad_snapshot = PoseSnapshot(
    seq=2,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.zeros(len(nominal), dtype=bool),
    disconnected_ids=[1],
)
bad_runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(bad_snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {},
    }
)
bad_report = bad_runner.run()
assert bad_report.ok is False
assert (
    "POSE_FRESH" in bad_report.failed_codes or "DISCONNECTED" in bad_report.failed_codes
)
assert any("missing" in reason or "outside" in reason for reason in bad_report.reasons)

low_bat_runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in fleet.leader_ids()
            }
        },
    }
)
for drone_id in fleet.all_ids():
    low_bat_runner.comp["health_bus"].update(drone_id, {"pm.vbat": 3.0}, 0.0)
low_bat_report = low_bat_runner.run()
assert low_bat_report.ok is True
assert not any(code.startswith("VBAT_DRONE_") for code in low_bat_report.failed_codes)
assert low_bat_report.reasons == []

stale_health_runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in fleet.leader_ids()
            }
        },
    }
)
for drone_id in fleet.all_ids():
    stale_health_runner.comp["health_bus"].update(drone_id, {"pm.vbat": 4.0}, -10.0)
stale_health_report = stale_health_runner.run()
assert stale_health_report.ok is False
assert any(
    code.startswith("HEALTH_FRESH_DRONE_") for code in stale_health_report.failed_codes
)
assert any("stale" in reason for reason in stale_health_report.reasons)

health_split_snapshot = PoseSnapshot(
    seq=3,
    t_meas=10.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
split_health_runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(health_split_snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in fleet.leader_ids()
            }
        },
    }
)
for drone_id in fleet.all_ids():
    split_health_runner.comp["health_bus"].update(drone_id, {"pm.vbat": 4.0}, 0.0)
    split_health_runner.comp["health_bus"].update(
        drone_id, {"stateEstimate.roll": 1.0}, 10.0
    )
split_health_report = split_health_runner.run()
assert split_health_report.ok is False
assert any(
    code.startswith("HEALTH_FRESH_DRONE_") for code in split_health_report.failed_codes
)
assert any("stale" in reason for reason in split_health_report.reasons)

bad_traj_runner = PreflightRunner(
    {
        "fleet": fleet,
        "formation": formation,
        "pose_bus": FakePoseBus(snapshot),
        "health_bus": HealthBus(),
        "config": config,
        "readiness_report": {
            "trajectory_prepare": {
                1: {"uploaded": False, "defined": False, "fits_memory": False}
            }
        },
    }
)
for drone_id in fleet.all_ids():
    bad_traj_runner.comp["health_bus"].update(drone_id, {"pm.vbat": 4.0}, 0.0)
bad_traj_report = bad_traj_runner.run()
assert bad_traj_report.ok is False
assert "TRAJECTORY_READY" in bad_traj_report.failed_codes

print("[OK] Preflight structured report contracts verified")
