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
    for record in records:
        if record.get("frame_valid"):
            valid_frame_count += 1
        if record.get("frame_condition_number") is not None:
            condition_numbers.append(record["frame_condition_number"])
        for drone_id, norm in record.get("follower_command_norms", {}).items():
            current = max_command_norm_per_drone.get(drone_id, 0.0)
            max_command_norm_per_drone[drone_id] = max(current, float(norm))

    return {
        "record_count": len(records),
        "event_counts": dict(event_counts),
        "safety_counts": dict(safety_counts),
        "scheduler_reason_counts": dict(scheduler_reason_counts),
        "first_mission_state": records[0].get("mission_state") if records else None,
        "last_mission_state": records[-1].get("mission_state") if records else None,
        "valid_frame_count": valid_frame_count,
        "frame_valid_rate": (valid_frame_count / len(records)) if records else 0.0,
        "frame_condition_min": min(condition_numbers) if condition_numbers else None,
        "frame_condition_max": max(condition_numbers) if condition_numbers else None,
        "max_command_norm_per_drone": max_command_norm_per_drone,
    }


def build_replay(records: list[dict]) -> dict:
    return {"records": records, "summary": analyze_records(records)}
