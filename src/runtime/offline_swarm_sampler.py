"""Offline mission/reference sampler for full-swarm visualization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..app.bootstrap import build_core_app
from .pose_snapshot import PoseSnapshot


@dataclass
class OfflineSwarmReplay:
    times: list[float]
    drone_ids: list[int]
    roles: dict[int, str]
    phase_labels: list[str]
    positions: dict[int, list[list[float] | None]]
    leader_positions: dict[int, list[list[float]]]
    follower_positions: dict[int, list[list[float] | None]]
    follower_valid: list[bool]
    frame_valid: list[bool]
    frame_condition_numbers: list[float]
    leader_modes: list[str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "times": self.times,
            "drone_ids": self.drone_ids,
            "roles": self.roles,
            "phase_labels": self.phase_labels,
            "positions": self.positions,
            "leader_positions": self.leader_positions,
            "follower_positions": self.follower_positions,
            "follower_valid": self.follower_valid,
            "frame_valid": self.frame_valid,
            "frame_condition_numbers": self.frame_condition_numbers,
            "leader_modes": self.leader_modes,
        }

    def drone_positions(self, drone_id: int) -> list[list[float] | None]:
        return self.positions[drone_id]


def _sample_times(total_time: float, dt: float) -> list[float]:
    if dt <= 0:
        raise ValueError("dt must be > 0")

    times = np.arange(0.0, total_time + 1e-9, dt, dtype=float).tolist()
    if not times or abs(times[-1] - total_time) > 1e-9:
        times.append(float(total_time))
    return [float(t) for t in times]


def _poly_value(coeffs: list[float], t_local: float) -> float:
    return float(
        sum(float(coeff) * (t_local**power) for power, coeff in enumerate(coeffs))
    )


def _evaluate_trajectory_piece(piece, t_local: float) -> np.ndarray:
    return np.array(
        [
            _poly_value(piece.x, t_local),
            _poly_value(piece.y, t_local),
            _poly_value(piece.z, t_local),
        ],
        dtype=float,
    )


def _evaluate_trajectory_spec(spec: dict, t: float) -> np.ndarray:
    pieces = spec.get("pieces", [])
    if not pieces:
        return np.zeros(3, dtype=float)

    elapsed = 0.0
    for piece in pieces:
        duration = float(piece.duration)
        if t <= elapsed + duration + 1e-9:
            local_t = min(max(t - elapsed, 0.0), duration)
            return _evaluate_trajectory_piece(piece, local_t)
        elapsed += duration

    return _evaluate_trajectory_piece(pieces[-1], float(pieces[-1].duration))


def _leader_positions_at(
    components: dict, t: float
) -> tuple[dict[int, np.ndarray], str]:
    leader_ref = components["leader_ref_gen"].reference_at(t)
    if leader_ref.mode != "trajectory":
        return {
            lid: np.array(pos, dtype=float) for lid, pos in leader_ref.positions.items()
        }, leader_ref.mode

    per_leader = (leader_ref.trajectory or {}).get("per_leader", {})
    t0 = components["mission_profile"].trajectory_start_time()
    if t < t0:
        positions = {
            drone_id: _evaluate_trajectory_spec(spec, 0.0)
            for drone_id, spec in per_leader.items()
        }
        return positions, leader_ref.mode

    t_traj = t - t0
    positions = {
        drone_id: _evaluate_trajectory_spec(spec, t_traj)
        for drone_id, spec in per_leader.items()
    }
    return positions, leader_ref.mode


def evaluate_offline_swarm_at_time(components: dict, t: float) -> dict[str, Any]:
    fleet = components["fleet"]
    frame_estimator = components["frame_estimator"]
    follower_ref_gen = components["follower_ref_gen"]

    leader_positions, leader_mode = _leader_positions_at(components, t)
    drone_ids = fleet.all_ids()
    ordered_positions = []
    for drone_id in drone_ids:
        if drone_id in leader_positions:
            position = np.array(leader_positions[drone_id], dtype=float)
        else:
            position = np.zeros(3, dtype=float)
        ordered_positions.append(position)

    snapshot = PoseSnapshot(
        seq=0,
        t_meas=t,
        positions=np.array(ordered_positions, dtype=float),
        fresh_mask=np.ones(len(drone_ids), dtype=bool),
        disconnected_ids=[],
    )
    frame = frame_estimator.estimate(snapshot, fleet.leader_ids())
    follower_ref = (
        follower_ref_gen.compute(frame.leader_positions, t) if frame.valid else None
    )

    return {
        "time": float(t),
        "phase_label": components["mission_profile"].phase_at(t).name,
        "leader_mode": leader_mode,
        "leader_reference_positions": {
            drone_id: np.array(position, dtype=float).round(9).tolist()
            for drone_id, position in leader_positions.items()
        },
        "follower_reference_positions": {
            drone_id: np.array(position, dtype=float).round(9).tolist()
            for drone_id, position in (
                follower_ref.target_positions.items()
                if follower_ref is not None and follower_ref.valid
                else []
            )
        },
        "follower_reference_valid": bool(follower_ref.valid)
        if follower_ref is not None
        else False,
        "frame_valid": bool(frame.valid),
        "frame_condition_number": float(frame.condition_number),
    }


def sample_offline_swarm(
    components: dict,
    dt: float = 0.25,
    total_time: float | None = None,
) -> OfflineSwarmReplay:
    mission = components["mission_profile"]
    fleet = components["fleet"]
    frame_estimator = components["frame_estimator"]
    follower_ref_gen = components["follower_ref_gen"]

    total = mission.total_time() if total_time is None else float(total_time)
    times = _sample_times(total, dt)
    leader_ids = fleet.leader_ids()
    follower_ids = fleet.follower_ids()
    drone_ids = fleet.all_ids()
    roles = {
        drone_id: ("leader" if fleet.is_leader(drone_id) else "follower")
        for drone_id in drone_ids
    }

    leader_positions: dict[int, list[list[float]]] = {
        drone_id: [] for drone_id in leader_ids
    }
    follower_positions: dict[int, list[list[float] | None]] = {
        drone_id: [] for drone_id in follower_ids
    }
    positions: dict[int, list[list[float] | None]] = {
        drone_id: [] for drone_id in drone_ids
    }
    phase_labels: list[str] = []
    follower_valid: list[bool] = []
    frame_valid: list[bool] = []
    frame_condition_numbers: list[float] = []
    leader_modes: list[str] = []

    for seq, t in enumerate(times):
        phase_labels.append(mission.phase_at(t).name)
        sampled_leader_positions, leader_mode = _leader_positions_at(components, t)
        leader_modes.append(leader_mode)

        ordered_positions = []
        for drone_id in drone_ids:
            if drone_id in sampled_leader_positions:
                position = np.array(sampled_leader_positions[drone_id], dtype=float)
            else:
                position = np.zeros(3, dtype=float)
            ordered_positions.append(position)

        snapshot = PoseSnapshot(
            seq=seq,
            t_meas=t,
            positions=np.array(ordered_positions, dtype=float),
            fresh_mask=np.ones(len(drone_ids), dtype=bool),
            disconnected_ids=[],
        )

        frame = frame_estimator.estimate(snapshot, leader_ids)
        follower_ref = (
            follower_ref_gen.compute(frame.leader_positions, t) if frame.valid else None
        )

        for drone_id in leader_ids:
            point = (
                np.array(sampled_leader_positions[drone_id], dtype=float)
                .round(9)
                .tolist()
            )
            leader_positions[drone_id].append(point)
            positions[drone_id].append(point)

        for drone_id in follower_ids:
            if follower_ref is not None and follower_ref.valid:
                point = (
                    np.array(follower_ref.target_positions[drone_id], dtype=float)
                    .round(9)
                    .tolist()
                )
                follower_positions[drone_id].append(point)
                positions[drone_id].append(point)
            else:
                follower_positions[drone_id].append(None)
                positions[drone_id].append(None)

        frame_valid.append(bool(frame.valid))
        follower_valid.append(
            bool(follower_ref.valid) if follower_ref is not None else False
        )
        frame_condition_numbers.append(float(frame.condition_number))

    return OfflineSwarmReplay(
        times=times,
        drone_ids=drone_ids,
        roles=roles,
        phase_labels=phase_labels,
        positions=positions,
        leader_positions=leader_positions,
        follower_positions=follower_positions,
        follower_valid=follower_valid,
        frame_valid=frame_valid,
        frame_condition_numbers=frame_condition_numbers,
        leader_modes=leader_modes,
    )


def sample_offline_swarm_from_config(
    config_dir: str = "config",
    dt: float = 0.25,
    total_time: float | None = None,
) -> OfflineSwarmReplay:
    return sample_offline_swarm(
        build_core_app(config_dir), dt=dt, total_time=total_time
    )
