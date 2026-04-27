"""Generate Crazyswarm2 configuration from the project fleet/mission config."""

from __future__ import annotations

from pathlib import Path

import yaml

from ..config.loader import ConfigLoader

DEFAULT_CRAZYSWARM_YAML = Path("artifacts/crazyswarm2/generated_crazyflies.yaml")


def _sim_uri_for_id(drone_id: int, *, channel: int = 90) -> str:
    if not 0 <= int(drone_id) <= 255:
        raise ValueError(f"Crazyswarm2 legacy id must fit one URI byte: {drone_id}")
    return f"radio://0/{channel}/2M/E7E7E7E7{int(drone_id):02X}"


def _initial_position(nominal_position: list[float]) -> list[float]:
    if len(nominal_position) < 2:
        raise ValueError(f"nominal_position must contain at least x/y: {nominal_position!r}")
    return [float(nominal_position[0]), float(nominal_position[1]), 0.0]


def build_crazyswarm2_yaml_data(config_dir: str, *, channel: int = 90) -> dict:
    config = ConfigLoader.load(config_dir)
    robots = {}
    for idx, drone in enumerate(config.fleet.drones):
        nominal = config.mission.nominal_positions[idx]
        robots[f"cf{drone.id}"] = {
            "enabled": True,
            "uri": _sim_uri_for_id(drone.id, channel=channel),
            "initial_position": _initial_position(nominal),
            "type": "cf21",
        }

    return {
        "fileversion": 3,
        "robots": robots,
        "robot_types": {
            "cf21": {
                "motion_capture": {
                    "tracking": "librigidbodytracker",
                    "marker": "default_single_marker",
                    "dynamics": "default",
                },
                "big_quad": False,
                "battery": {
                    "voltage_warning": 3.8,
                    "voltage_critical": 3.7,
                },
            }
        },
        "all": {
            "firmware_logging": {
                "enabled": True,
                "default_topics": {
                    "pose": {"frequency": 10},
                    "status": {"frequency": 1},
                },
            },
            "firmware_params": {
                "commander": {"enHighLevel": 1},
                "stabilizer": {
                    "estimator": 2,
                    "controller": 2,
                },
                "locSrv": {
                    "extPosStdDev": 1e-3,
                    "extQuatStdDev": 0.5e-1,
                },
            },
            "reference_frame": "world",
            "broadcasts": {
                "num_repeats": 15,
                "delay_between_repeats_ms": 1,
            },
        },
    }


def generate_crazyswarm2_yaml(
    config_dir: str,
    output_path: str | Path = DEFAULT_CRAZYSWARM_YAML,
    *,
    channel: int = 90,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = build_crazyswarm2_yaml_data(config_dir, channel=channel)
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )
    return path
