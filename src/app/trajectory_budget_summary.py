"""Dry-run trajectory budget summary without connecting drones."""

from __future__ import annotations

import json

from .bootstrap import build_core_app
from ..adapters.trajectory_common import (
    TRAJECTORY_MAX_PIECES,
    TRAJECTORY_MEMORY_BYTES,
    estimate_trajectory_bytes,
)


def build_trajectory_budget_summary(config_dir: str = "config") -> dict:
    components = build_core_app(config_dir)
    leader_ref = components["leader_ref_gen"].reference_at(0.0)
    mission = components["config"].mission
    motion = mission.leader_motion
    summary_base = {
        "config_dir": components.get("config_dir"),
        "startup_mode": components.get("startup_mode"),
        "mission_duration": components["mission_profile"].total_time(),
        "trajectory_sample_dt": motion.trajectory_sample_dt,
        "trajectory_time_scale": motion.trajectory_time_scale,
        "angular_rate": motion.angular_rate,
        "phase_schedule": [
            {
                "name": phase.name,
                "t_start": phase.t_start,
                "t_end": phase.t_end,
                "mode": phase.mode,
            }
            for phase in mission.phases
        ],
    }
    if leader_ref.mode != "trajectory" or leader_ref.trajectory is None:
        return {
            **summary_base,
            "mode": leader_ref.mode,
            "trajectory_enabled": False,
            "leaders": {},
        }

    per_leader = leader_ref.trajectory.get("per_leader", {})
    leaders = {}
    for drone_id, spec in per_leader.items():
        pieces = spec.get("pieces", [])
        start_addr = spec.get("start_addr", 0)
        trajectory_type = spec.get("trajectory_type", "poly4d")
        estimated_bytes = estimate_trajectory_bytes(pieces, trajectory_type)
        leaders[int(drone_id)] = {
            "trajectory_id": spec.get("trajectory_id", 1),
            "trajectory_type": trajectory_type,
            "pieces": len(pieces),
            "estimated_bytes": estimated_bytes,
            "start_addr": start_addr,
            "fits_memory": start_addr + estimated_bytes <= TRAJECTORY_MEMORY_BYTES,
            "fits_piece_count": len(pieces) <= TRAJECTORY_MAX_PIECES,
            "max_pieces": TRAJECTORY_MAX_PIECES,
            "memory_capacity": TRAJECTORY_MEMORY_BYTES,
            "nominal_position": spec.get("nominal_position"),
        }

    return {
        **summary_base,
        "mode": leader_ref.mode,
        "trajectory_enabled": True,
        "leaders": leaders,
    }


def print_trajectory_budget_summary(config_dir: str = "config") -> int:
    summary = build_trajectory_budget_summary(config_dir)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0
