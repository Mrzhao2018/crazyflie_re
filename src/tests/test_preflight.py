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


class WindowPoseBus(FakePoseBus):
    def __init__(self, snapshot, samples):
        super().__init__(snapshot)
        self._samples = samples

    def recent_samples(self, window_s):
        return self._samples


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
nominal = np.array(config.mission.nominal_positions, dtype=float)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)


def healthy_values(vbat=4.0, variance=0.0005):
    return {
        "pm.vbat": vbat,
        "kalman.varPX": variance,
        "kalman.varPY": variance,
        "kalman.varPZ": variance,
    }

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
    runner.comp["health_bus"].update(drone_id, healthy_values(), 0.0)
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
low_bat_runner.comp["config"].safety.min_vbat = 0.0
for drone_id in fleet.all_ids():
    low_bat_runner.comp["health_bus"].update(drone_id, healthy_values(vbat=3.0), 0.0)
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
    stale_health_runner.comp["health_bus"].update(drone_id, healthy_values(), -10.0)
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
    split_health_runner.comp["health_bus"].update(drone_id, healthy_values(), 0.0)
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
    bad_traj_runner.comp["health_bus"].update(drone_id, healthy_values(), 0.0)
bad_traj_report = bad_traj_runner.run()
assert bad_traj_report.ok is False
assert "TRAJECTORY_READY" in bad_traj_report.failed_codes

spacing_config = ConfigLoader.load("config")
spacing_config.safety.min_inter_drone_distance = 0.2
spacing_fleet = FleetModel(spacing_config.fleet)
spacing_nominal = np.array(spacing_config.mission.nominal_positions, dtype=float)
spacing_nominal[1] = spacing_nominal[0] + np.array([0.05, 0.0, 0.0])
spacing_formation = FormationModel(spacing_nominal, spacing_fleet.leader_ids(), spacing_fleet)
spacing_snapshot = PoseSnapshot(
    seq=5,
    t_meas=0.0,
    positions=spacing_nominal,
    fresh_mask=np.ones(len(spacing_nominal), dtype=bool),
    disconnected_ids=[],
)
spacing_runner = PreflightRunner(
    {
        "fleet": spacing_fleet,
        "formation": spacing_formation,
        "pose_bus": FakePoseBus(spacing_snapshot),
        "health_bus": HealthBus(),
        "config": spacing_config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in spacing_fleet.leader_ids()
            }
        },
    }
)
for drone_id in spacing_fleet.all_ids():
    spacing_runner.comp["health_bus"].update(drone_id, healthy_values(), 0.0)
spacing_report = spacing_runner.run()
assert spacing_report.ok is False
assert "FORMATION_SPACING" in spacing_report.failed_codes

high_var_runner = PreflightRunner(
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
    high_var_runner.comp["health_bus"].update(drone_id, healthy_values(), 0.0)
high_var_runner.comp["health_bus"].update(
    fleet.all_ids()[0],
    healthy_values(variance=config.safety.estimator_variance_threshold * 2.0),
    0.0,
)
high_var_report = high_var_runner.run()
assert high_var_report.ok is False
assert f"ESTIMATOR_VARIANCE_DRONE_{fleet.all_ids()[0]}" in high_var_report.failed_codes
assert any("variance" in reason for reason in high_var_report.reasons)

window_config = ConfigLoader.load("config")
window_config.safety.pose_jitter_threshold = 0.01
window_config.safety.estimator_variance_window_s = 2.0
window_config.safety.lighthouse_required_method = 8
window_fleet = FleetModel(window_config.fleet)
window_nominal = np.array(window_config.mission.nominal_positions, dtype=float)
window_formation = FormationModel(window_nominal, window_fleet.leader_ids(), window_fleet)
window_snapshot = PoseSnapshot(
    seq=4,
    t_meas=1.0,
    positions=window_nominal,
    fresh_mask=np.ones(len(window_nominal), dtype=bool),
    disconnected_ids=[],
)
window_health = HealthBus()
for drone_id in window_fleet.all_ids():
    window_health.update(
        drone_id,
        {**healthy_values(variance=0.0002), "lighthouse.method": 8},
        0.0,
    )
window_health.update(
    window_fleet.all_ids()[0],
    healthy_values(variance=window_config.safety.estimator_variance_threshold * 2.0),
    0.5,
)
window_health.update(
    window_fleet.all_ids()[0],
    {**healthy_values(variance=0.0002), "lighthouse.method": 8},
    1.0,
)
pose_samples = {
    drone_id: [
        (0.0, window_nominal[window_fleet.id_to_index(drone_id)]),
        (0.5, window_nominal[window_fleet.id_to_index(drone_id)]),
        (1.0, window_nominal[window_fleet.id_to_index(drone_id)]),
    ]
    for drone_id in window_fleet.all_ids()
}
pose_samples[window_fleet.all_ids()[0]][1] = (
    0.5,
    window_nominal[window_fleet.id_to_index(window_fleet.all_ids()[0])]
    + np.array([0.03, 0.0, 0.0]),
)
window_report = PreflightRunner(
    {
        "fleet": window_fleet,
        "formation": window_formation,
        "pose_bus": WindowPoseBus(window_snapshot, pose_samples),
        "health_bus": window_health,
        "config": window_config,
        "readiness_report": {
            "trajectory_prepare": {
                drone_id: {"uploaded": True, "defined": True, "fits_memory": True}
                for drone_id in window_fleet.leader_ids()
            }
        },
    }
).run()
assert window_report.ok is False
assert f"POSE_JITTER_DRONE_{window_fleet.all_ids()[0]}" in window_report.failed_codes
assert (
    f"ESTIMATOR_VARIANCE_WINDOW_DRONE_{window_fleet.all_ids()[0]}"
    in window_report.failed_codes
)
assert not any(code.startswith("LIGHTHOUSE_METHOD_DRONE_") for code in window_report.failed_codes)

print("[OK] Preflight structured report contracts verified")
