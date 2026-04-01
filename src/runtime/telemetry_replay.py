"""Offline telemetry replay analysis helpers."""

from __future__ import annotations

import json
from collections import Counter


def iter_records(path: str):
    with open(path, encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"Invalid telemetry JSON on line {line_no}: {exc}"
                ) from exc


def load_records(path: str) -> list[dict]:
    return list(iter_records(path))


def analyze_records(records: list[dict]) -> dict:
    deduped = []
    seen = set()
    for record in records:
        for event in record.get("phase_events", []):
            key = (
                event.get("event"),
                json.dumps(event.get("details", {}), sort_keys=True),
            )
            if key not in seen:
                seen.add(key)
                deduped.append(event)

    event_counts = Counter(event.get("event") for event in deduped)
    safety_counts = Counter(record.get("safety_action") for record in records)
    scheduler_reason_counts = Counter(
        record.get("scheduler_reason")
        for record in records
        if record.get("scheduler_reason")
    )

    max_command_norm_per_drone = {}
    valid_frame_count = 0
    condition_numbers = []
    tracking_errors_per_drone = {}
    role_tracking_errors = {"leader": [], "follower": []}
    phase_tracking_errors: dict[str, list[float]] = {}
    phase_role_tracking_errors: dict[str, dict[str, list[float]]] = {}
    formation_errors = []
    fresh_sample_count = 0
    total_sample_count = 0
    for record in records:
        if record.get("frame_valid"):
            valid_frame_count += 1
        if record.get("frame_condition_number") is not None:
            condition_numbers.append(record["frame_condition_number"])
        measured_positions = record.get("measured_positions", {}) or {}
        leader_reference_positions = record.get("leader_reference_positions", {}) or {}
        follower_reference_positions = (
            record.get("follower_reference_positions", {}) or {}
        )
        fresh_mask = record.get("fresh_mask", {}) or {}
        phase_label = record.get("phase_label") or "unknown"

        record_errors = []
        for drone_id, is_fresh in fresh_mask.items():
            if is_fresh:
                fresh_sample_count += 1
            total_sample_count += 1

        reference_positions = {
            **leader_reference_positions,
            **follower_reference_positions,
        }
        for drone_id, ref_pos in reference_positions.items():
            measured = measured_positions.get(
                str(drone_id), measured_positions.get(drone_id)
            )
            if measured is None:
                continue
            error = _euclidean_error(measured, ref_pos)
            tracking_errors_per_drone.setdefault(str(drone_id), []).append(error)
            record_errors.append(error)
            role = (
                "leader"
                if drone_id in leader_reference_positions
                or str(drone_id) in leader_reference_positions
                else "follower"
            )
            role_tracking_errors[role].append(error)
            phase_tracking_errors.setdefault(phase_label, []).append(error)
            phase_role_tracking_errors.setdefault(
                phase_label, {"leader": [], "follower": []}
            )[role].append(error)

        if record_errors:
            formation_errors.append(sum(record_errors) / len(record_errors))

        for drone_id, norm in record.get("follower_command_norms", {}).items():
            current = max_command_norm_per_drone.get(drone_id, 0.0)
            max_command_norm_per_drone[drone_id] = max(current, float(norm))

    per_drone_error_stats = {
        drone_id: _series_stats(values)
        for drone_id, values in tracking_errors_per_drone.items()
    }
    role_error_stats = {
        role: _series_stats(values) for role, values in role_tracking_errors.items()
    }
    phase_error_stats = {
        phase: _series_stats(values) for phase, values in phase_tracking_errors.items()
    }
    phase_role_error_stats = {
        phase: {role: _series_stats(values) for role, values in role_map.items()}
        for phase, role_map in phase_role_tracking_errors.items()
    }
    formation_error_stats = _series_stats(formation_errors)

    return {
        "record_count": len(records),
        "phase_counts": {
            phase: len(values) for phase, values in phase_tracking_errors.items()
        },
        "event_counts": dict(event_counts),
        "safety_counts": dict(safety_counts),
        "scheduler_reason_counts": dict(scheduler_reason_counts),
        "first_mission_state": records[0].get("mission_state") if records else None,
        "last_mission_state": records[-1].get("mission_state") if records else None,
        "valid_frame_count": valid_frame_count,
        "frame_valid_rate": (valid_frame_count / len(records)) if records else 0.0,
        "frame_condition_min": min(condition_numbers) if condition_numbers else None,
        "frame_condition_max": max(condition_numbers) if condition_numbers else None,
        "fresh_sample_rate": (
            fresh_sample_count / total_sample_count if total_sample_count else 0.0
        ),
        "effective_update_rate_hz": _effective_update_rate_hz(records),
        "max_command_norm_per_drone": max_command_norm_per_drone,
        "per_drone_tracking_error": per_drone_error_stats,
        "role_tracking_error": role_error_stats,
        "phase_tracking_error": phase_error_stats,
        "phase_role_tracking_error": phase_role_error_stats,
        "formation_error": formation_error_stats,
    }


def build_replay(records: list[dict]) -> dict:
    return {"records": records, "summary": analyze_records(records)}


def _euclidean_error(actual: list[float], target: list[float]) -> float:
    return sum((float(a) - float(b)) ** 2 for a, b in zip(actual, target)) ** 0.5


def _series_stats(values: list[float]) -> dict:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "rmse": None,
            "p95": None,
            "max": None,
        }
    mean = sum(values) / len(values)
    rmse = (sum(v * v for v in values) / len(values)) ** 0.5
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, max(0, int(round(0.95 * (len(ordered) - 1)))))
    return {
        "count": len(values),
        "mean": mean,
        "rmse": rmse,
        "p95": ordered[p95_index],
        "max": max(values),
    }


def _effective_update_rate_hz(records: list[dict]) -> float | None:
    seen = set()
    times = []
    for record in records:
        seq = record.get("snapshot_seq")
        t_meas = record.get("snapshot_t_meas")
        if seq is None or t_meas is None or seq in seen:
            continue
        seen.add(seq)
        times.append(float(t_meas))

    if len(times) < 2:
        return None
    duration = times[-1] - times[0]
    if duration <= 0:
        return None
    return (len(times) - 1) / duration
