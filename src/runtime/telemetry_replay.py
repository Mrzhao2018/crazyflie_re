"""Offline telemetry replay analysis helpers.

支持 schema_version=2 的 JSONL：``kind=header`` / ``kind=event`` / ``kind=record``。
对旧格式（记录内嵌 ``phase_events`` 列表、不带 ``kind`` 字段）保留兼容路径。
"""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass


WATCHDOG_EVENT_CODES = {
    "RUNTIME_VELOCITY_STREAM_WATCHDOG": "telemetry",
    "RUNTIME_WATCHDOG_HOLD": "hold",
    "RUNTIME_WATCHDOG_DEGRADE": "degrade",
    "RUNTIME_WATCHDOG_DEGRADE_RECOVERED": "degrade_recovered",
}

EXECUTOR_EVENT_CODES = {
    "RUNTIME_EXECUTOR_GROUP_DEGRADE": "degrade",
    "RUNTIME_EXECUTOR_GROUP_HOLD": "hold",
}


@dataclass
class TelemetryStream:
    header: dict | None
    events: list[dict]
    records: list[dict]


def _watchdog_summary(events: list[dict]) -> dict:
    code_counts = Counter()
    mode_counts = Counter()
    event_counts = Counter()

    for event in events:
        event_name = event.get("event")
        details = event.get("details", {}) or {}
        code = details.get("code")
        if code not in WATCHDOG_EVENT_CODES:
            continue
        code_counts[code] += 1
        mode_counts[WATCHDOG_EVENT_CODES[code]] += 1
        if event_name:
            event_counts[event_name] += 1

    return {
        "total": sum(code_counts.values()),
        "by_code": dict(code_counts),
        "by_mode": dict(mode_counts),
        "by_event": dict(event_counts),
    }


def _executor_failure_summary(events: list[dict]) -> dict:
    code_counts = Counter()
    action_counts = Counter()
    event_counts = Counter()
    group_counts = Counter()
    failure_category_counts = Counter()
    retryable_counts = Counter()

    for event in events:
        event_name = event.get("event")
        details = event.get("details", {}) or {}
        code = details.get("code")
        if code not in EXECUTOR_EVENT_CODES:
            continue

        code_counts[code] += 1
        action_counts[EXECUTOR_EVENT_CODES[code]] += 1
        if event_name:
            event_counts[event_name] += 1

        for group_item in details.get("triggered_groups", []) or []:
            if not isinstance(group_item, dict):
                continue
            group_id = group_item.get("group_id")
            if group_id is not None:
                group_counts[str(group_id)] += 1
            for failure in group_item.get("failures", []) or []:
                if not isinstance(failure, dict):
                    continue
                category = failure.get("failure_category")
                if category is not None:
                    failure_category_counts[str(category)] += 1
                retryable = failure.get("retryable")
                if retryable is True:
                    retryable_counts["retryable"] += 1
                elif retryable is False:
                    retryable_counts["non_retryable"] += 1

    return {
        "total": sum(code_counts.values()),
        "by_code": dict(code_counts),
        "by_action": dict(action_counts),
        "by_event": dict(event_counts),
        "by_group": dict(group_counts),
        "failure_categories": dict(failure_category_counts),
        "retryable_counts": dict(retryable_counts),
    }


def _iter_lines(path: str):
    with open(path, encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield line_no, json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"Invalid telemetry JSON on line {line_no}: {exc}"
                ) from exc


def iter_telemetry(path: str) -> TelemetryStream:
    """Parse a JSONL telemetry file into (header, events, records).

    Recognises schema_version=2 lines tagged with ``kind`` as well as legacy
    records that inline ``phase_events`` lists.
    """

    header: dict | None = None
    events: list[dict] = []
    records: list[dict] = []
    legacy_mode = False

    for _, entry in _iter_lines(path):
        if not isinstance(entry, dict):
            continue

        kind = entry.get("kind")
        if kind == "header":
            header = entry
            continue
        if kind == "event":
            events.append(_event_from_line(entry))
            continue
        if kind == "record":
            record = {k: v for k, v in entry.items() if k != "kind"}
            records.append(record)
            continue

        # Legacy shape: bare record dict with phase_events inline.
        legacy_mode = True
        records.append(entry)

    if legacy_mode and not events:
        seen = set()
        for record in records:
            for event in record.get("phase_events", []) or []:
                key = (
                    event.get("event"),
                    json.dumps(event.get("details", {}), sort_keys=True),
                )
                if key in seen:
                    continue
                seen.add(key)
                events.append(event)

    return TelemetryStream(header=header, events=events, records=records)


def _event_from_line(entry: dict) -> dict:
    event = {
        "event": entry.get("event"),
        "details": entry.get("details", {}) or {},
    }
    if "t_wall" in entry:
        event["t_wall"] = entry["t_wall"]
    return event


def iter_records(path: str):
    """Legacy helper that yields only record dicts."""

    stream = iter_telemetry(path)
    for record in stream.records:
        yield record


def load_records(path: str) -> list[dict]:
    return list(iter_telemetry(path).records)


def load_telemetry(path: str) -> TelemetryStream:
    return iter_telemetry(path)


def _events_from_records(records: list[dict]) -> list[dict]:
    deduped = []
    seen = set()
    for record in records:
        for event in record.get("phase_events", []) or []:
            key = (
                event.get("event"),
                json.dumps(event.get("details", {}), sort_keys=True),
            )
            if key in seen:
                continue
            seen.add(key)
            deduped.append(event)
    return deduped


def analyze_records(
    records: list[dict],
    events: list[dict] | None = None,
    *,
    header: dict | None = None,
) -> dict:
    if events is None:
        events = _events_from_records(records)

    event_counts = Counter(event.get("event") for event in events)
    safety_counts = Counter(record.get("safety_action") for record in records)
    scheduler_reason_counts = Counter(
        record.get("scheduler_reason")
        for record in records
        if record.get("scheduler_reason")
    )

    if header and isinstance(header, dict) and header.get("config_fingerprint"):
        config_fingerprint = header.get("config_fingerprint")
    else:
        config_fingerprint = (
            records[0].get("config_fingerprint") if records else None
        )
    watchdog_summary = _watchdog_summary(events)
    executor_failure_summary = _executor_failure_summary(events)

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
        "config_fingerprint": config_fingerprint,
        "phase_counts": {
            phase: len(values) for phase, values in phase_tracking_errors.items()
        },
        "event_counts": dict(event_counts),
        "watchdog_summary": watchdog_summary,
        "executor_failure_summary": executor_failure_summary,
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


def build_replay(records: list[dict], events: list[dict] | None = None) -> dict:
    return {
        "records": records,
        "events": events if events is not None else _events_from_records(records),
        "summary": analyze_records(records, events=events),
    }


def analyze_telemetry(stream: TelemetryStream) -> dict:
    return analyze_records(stream.records, stream.events, header=stream.header)


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
