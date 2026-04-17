"""Offline mission/reference sampler for full-swarm visualization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..app.bootstrap import build_core_app
from .pose_snapshot import PoseSnapshot
from .telemetry_replay import analyze_records


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

    def as_telemetry_records(self) -> list[dict[str, Any]]:
        records = []
        for index, t_meas in enumerate(self.times):
            measured_positions = {
                str(drone_id): position
                for drone_id, positions in self.positions.items()
                for position in [positions[index]]
                if position is not None
            }
            leader_reference_positions = {
                str(drone_id): self.leader_positions[drone_id][index]
                for drone_id in self.leader_positions
                if self.leader_positions[drone_id][index] is not None
            }
            follower_reference_positions = {
                str(drone_id): self.follower_positions[drone_id][index]
                for drone_id in self.follower_positions
                if self.follower_positions[drone_id][index] is not None
            }
            fresh_mask = {str(drone_id): True for drone_id in self.drone_ids}
            records.append(
                {
                    "mission_state": "RUN",
                    "mission_elapsed": float(t_meas),
                    "snapshot_seq": index,
                    "snapshot_t_meas": float(t_meas),
                    "measured_positions": measured_positions,
                    "fresh_mask": fresh_mask,
                    "disconnected_ids": [],
                    "phase_label": self.phase_labels[index],
                    "leader_mode": self.leader_modes[index],
                    "leader_reference_positions": leader_reference_positions,
                    "follower_reference_positions": follower_reference_positions,
                    "safety_action": "EXECUTE",
                    "scheduler_reason": "offline_sample",
                    "frame_valid": self.frame_valid[index],
                    "frame_condition_number": self.frame_condition_numbers[index],
                    "follower_command_norms": {},
                }
            )
        return records

    def performance_summary(self) -> dict[str, Any]:
        return analyze_records(self.as_telemetry_records())


@dataclass
class OfflineClosedLoopReplay:
    times: list[float]
    phase_labels: list[str]
    leader_modes: list[str]
    frame_valid: list[bool]
    follower_valid: list[bool]
    frame_condition_numbers: list[float]
    records: list[dict[str, Any]]
    condition_penalty_summary: dict[str, Any] | None = None

    def performance_summary(self) -> dict[str, Any]:
        summary = analyze_records(self.records)
        summary["trajectory_quality_summary"] = self.condition_penalty_summary or {}
        return summary


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


def simulate_offline_closed_loop(
    components: dict,
    dt: float = 0.25,
    total_time: float | None = None,
) -> OfflineClosedLoopReplay:
    mission = components["mission_profile"]
    fleet = components["fleet"]
    frame_estimator = components["frame_estimator"]
    follower_ref_gen = components["follower_ref_gen"]
    follower_controller = components["follower_controller"]
    safety_manager = components["safety"]
    afc = components["afc"]
    config = components["config"]

    total = mission.total_time() if total_time is None else float(total_time)
    pose_period = 1.0 / config.comm.pose_log_freq
    tx_period = 1.0 / config.comm.follower_tx_freq
    internal_dt = min(float(dt), pose_period, tx_period) / 4.0
    times = _sample_times(total, internal_dt)

    drone_ids = fleet.all_ids()
    leader_ids = fleet.leader_ids()
    follower_ids = fleet.follower_ids()
    positions = np.array(config.mission.nominal_positions, dtype=float)
    follower_velocities = {
        drone_id: np.zeros(3, dtype=float) for drone_id in follower_ids
    }
    applied_commands = {drone_id: np.zeros(3, dtype=float) for drone_id in follower_ids}
    applied_accelerations = {
        drone_id: np.zeros(3, dtype=float) for drone_id in follower_ids
    }
    last_sent_commands: dict[int, np.ndarray] = {}
    phase_labels: list[str] = []
    leader_modes: list[str] = []
    frame_valid: list[bool] = []
    follower_valid: list[bool] = []
    frame_condition_numbers: list[float] = []
    records: list[dict[str, Any]] = []

    next_pose_time = 0.0
    next_tx_time = 0.0
    latest_frame = None
    latest_follower_ref = None
    latest_commands = None
    latest_safety = None

    for seq, t in enumerate(times):
        leader_positions, leader_mode = _leader_positions_at(components, t)
        leader_modes.append(leader_mode)
        phase_labels.append(mission.phase_at(t).name)

        for drone_id, leader_position in leader_positions.items():
            positions[fleet.id_to_index(drone_id)] = np.array(leader_position, dtype=float)

        if t + 1e-9 >= next_pose_time:
            snapshot = PoseSnapshot(
                seq=seq,
                t_meas=t,
                positions=positions.copy(),
                fresh_mask=np.ones(len(drone_ids), dtype=bool),
                disconnected_ids=[],
            )
            latest_frame = frame_estimator.estimate(snapshot, leader_ids)
            latest_follower_ref = (
                follower_ref_gen.compute(latest_frame.leader_positions, t)
                if latest_frame.valid
                else None
            )
            latest_commands = (
                follower_controller.compute(
                    snapshot,
                    latest_follower_ref,
                    follower_ids,
                    fleet,
                )
                if latest_follower_ref is not None and latest_follower_ref.valid
                else None
            )
            latest_safety = safety_manager.evaluate(
                snapshot,
                latest_frame,
                latest_commands,
                latest_follower_ref,
            )
            next_pose_time += pose_period

        if t + 1e-9 >= next_tx_time:
            if (
                latest_commands is not None
                and latest_safety is not None
                and latest_safety.action == "EXECUTE"
            ):
                acceleration_commands = {
                    int(drone_id): np.array(command, dtype=float)
                    for drone_id, command in (
                        latest_commands.diagnostics.get("commanded_accelerations") or {}
                    ).items()
                }
                for drone_id, velocity in latest_commands.commands.items():
                    previous = last_sent_commands.get(drone_id)
                    if previous is not None:
                        delta = float(np.linalg.norm(np.array(velocity) - previous))
                        if delta < config.comm.follower_cmd_deadband:
                            continue
                    applied_commands[drone_id] = np.array(velocity, dtype=float)
                    last_sent_commands[drone_id] = np.array(velocity, dtype=float)
                    applied_accelerations[drone_id] = acceleration_commands.get(
                        drone_id, np.zeros(3, dtype=float)
                    )
            elif latest_safety is not None and latest_safety.action != "EXECUTE":
                for drone_id in follower_ids:
                    applied_commands[drone_id] = np.zeros(3, dtype=float)
                    applied_accelerations[drone_id] = np.zeros(3, dtype=float)
                    last_sent_commands[drone_id] = np.zeros(3, dtype=float)
            next_tx_time += tx_period

        truth_follower_positions = (
            afc.steady_state(leader_positions)
            if latest_frame is not None and latest_frame.valid
            else {}
        )
        follower_command_norms = {
            str(drone_id): float(np.linalg.norm(command))
            for drone_id, command in applied_commands.items()
        }
        records.append(
            {
                "mission_state": "RUN",
                "mission_elapsed": float(t),
                "snapshot_seq": seq,
                "snapshot_t_meas": float(t),
                "measured_positions": {
                    str(drone_id): positions[fleet.id_to_index(drone_id)].round(9).tolist()
                    for drone_id in drone_ids
                },
                "fresh_mask": {str(drone_id): True for drone_id in drone_ids},
                "disconnected_ids": [],
                "phase_events": [],
                "phase_label": phase_labels[-1],
                "leader_mode": leader_mode,
                "leader_reference_positions": {
                    str(drone_id): np.array(position, dtype=float).round(9).tolist()
                    for drone_id, position in leader_positions.items()
                },
                "follower_reference_positions": {
                    str(drone_id): np.array(position, dtype=float).round(9).tolist()
                    for drone_id, position in truth_follower_positions.items()
                },
                "safety_action": latest_safety.action if latest_safety is not None else "EXECUTE",
                "scheduler_reason": "offline_closed_loop",
                "frame_valid": bool(latest_frame.valid) if latest_frame is not None else False,
                "frame_condition_number": float(latest_frame.condition_number)
                if latest_frame is not None
                else float("inf"),
                "follower_command_norms": follower_command_norms,
            }
        )
        frame_valid.append(bool(latest_frame.valid) if latest_frame is not None else False)
        follower_valid.append(
            bool(latest_follower_ref.valid)
            if latest_follower_ref is not None
            else False
        )
        frame_condition_numbers.append(
            float(latest_frame.condition_number)
            if latest_frame is not None
            else float("inf")
        )

        if t >= total:
            continue

        for drone_id in follower_ids:
            idx = fleet.id_to_index(drone_id)
            if config.control.dynamics_model_order == 2:
                follower_velocities[drone_id] = (
                    follower_velocities[drone_id]
                    + applied_accelerations[drone_id] * internal_dt
                    - config.control.damping_coeff * follower_velocities[drone_id] * internal_dt
                )
                speed = np.linalg.norm(follower_velocities[drone_id])
                if speed > config.control.max_velocity:
                    follower_velocities[drone_id] = (
                        follower_velocities[drone_id] / speed * config.control.max_velocity
                    )
                positions[idx] = positions[idx] + follower_velocities[drone_id] * internal_dt
            else:
                positions[idx] = positions[idx] + applied_commands[drone_id] * internal_dt

    return OfflineClosedLoopReplay(
        times=times,
        phase_labels=phase_labels,
        leader_modes=leader_modes,
        frame_valid=frame_valid,
        follower_valid=follower_valid,
        frame_condition_numbers=frame_condition_numbers,
        records=records,
        condition_penalty_summary=mission.trajectory_quality_summary(),
    )


def sample_offline_swarm_from_config(
    config_dir: str = "config",
    dt: float = 0.25,
    total_time: float | None = None,
) -> OfflineSwarmReplay:
    return sample_offline_swarm(
        build_core_app(config_dir), dt=dt, total_time=total_time
    )
