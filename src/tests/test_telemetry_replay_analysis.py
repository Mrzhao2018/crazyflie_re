"""Offline telemetry replay analysis tests."""

import json
from pathlib import Path

from src.runtime.telemetry_replay import (
    analyze_records,
    analyze_telemetry,
    build_replay,
    iter_telemetry,
    load_records,
)


tmp_path = Path("telemetry") / "test_replay.jsonl"
tmp_path.parent.mkdir(parents=True, exist_ok=True)


header_line = {
    "kind": "header",
    "schema_version": 2,
    "t_wall": 0.0,
    "config_fingerprint": {"config_sha256": "abc123", "startup_mode": "auto"},
    "readiness": {"pose_ready": True},
    "fleet": {"drone_count": 2},
}

events_lines = [
    {
        "kind": "event",
        "t_wall": 0.01,
        "event": "wait_for_params",
        "details": {"drone_id": 1},
    },
    {
        "kind": "event",
        "t_wall": 0.02,
        "event": "preflight",
        "details": {"ok": True},
    },
    {
        "kind": "event",
        "t_wall": 0.03,
        "event": "velocity_stream_watchdog",
        "details": {
            "code": "RUNTIME_VELOCITY_STREAM_WATCHDOG",
            "category": "runtime",
            "stage": "velocity_stream_watchdog",
        },
    },
    {
        "kind": "event",
        "t_wall": 0.04,
        "event": "watchdog_degrade",
        "details": {
            "code": "RUNTIME_WATCHDOG_DEGRADE",
            "category": "runtime",
            "stage": "watchdog_degrade",
        },
    },
    {
        "kind": "event",
        "t_wall": 0.05,
        "event": "executor_group_degrade",
        "details": {
            "code": "RUNTIME_EXECUTOR_GROUP_DEGRADE",
            "category": "runtime",
            "stage": "executor_group_degrade",
            "triggered_groups": [
                {
                    "group_id": 2,
                    "failures": [
                        {
                            "failure_category": "timeout",
                            "retryable": True,
                        }
                    ],
                }
            ],
        },
    },
    {
        "kind": "event",
        "t_wall": 0.06,
        "event": "emergency_land",
        "details": {"ok": True},
    },
    {
        "kind": "event",
        "t_wall": 0.07,
        "event": "hold_entered",
        "details": {
            "code": "RUNTIME_WATCHDOG_HOLD",
            "category": "runtime",
            "stage": "watchdog_hold",
        },
    },
    {
        "kind": "event",
        "t_wall": 0.08,
        "event": "watchdog_degrade_recovered",
        "details": {
            "code": "RUNTIME_WATCHDOG_DEGRADE_RECOVERED",
            "category": "runtime",
            "stage": "watchdog_degrade_recovered",
        },
    },
    {
        "kind": "event",
        "t_wall": 0.09,
        "event": "executor_group_hold",
        "details": {
            "code": "RUNTIME_EXECUTOR_GROUP_HOLD",
            "category": "runtime",
            "stage": "executor_group_hold",
            "triggered_groups": [
                {
                    "group_id": 2,
                    "failures": [
                        {
                            "failure_category": "link_lookup",
                            "retryable": False,
                        }
                    ],
                }
            ],
        },
    },
]

record_lines = [
    {
        "kind": "record",
        "mission_state": "RUN",
        "mission_elapsed": 0.0,
        "snapshot_seq": 1,
        "snapshot_t_meas": 0.0,
        "measured_positions": {"1": [0.0, 0.0, 0.5], "5": [0.2, 0.0, 0.5]},
        "fresh_mask": {"1": True, "5": True},
        "disconnected_ids": [],
        "phase_label": "formation_run",
        "leader_mode": "batch_goto",
        "leader_reference_positions": {"1": [0.1, 0.0, 0.5]},
        "follower_reference_positions": {"5": [0.1, 0.0, 0.5]},
        "safety_action": "EXECUTE",
        "scheduler_reason": "execute",
        "frame_valid": True,
        "frame_condition_number": 1.0,
        "follower_command_norms": {"5": 0.2},
    },
    {
        "kind": "record",
        "mission_state": "ABORT",
        "mission_elapsed": 0.1,
        "snapshot_seq": 2,
        "snapshot_t_meas": 0.1,
        "measured_positions": {"1": [0.2, 0.0, 0.5], "5": [0.5, 0.0, 0.5]},
        "fresh_mask": {"1": True, "5": False},
        "disconnected_ids": [5],
        "phase_label": "formation_run",
        "leader_mode": "batch_goto",
        "leader_reference_positions": {"1": [0.1, 0.0, 0.5]},
        "follower_reference_positions": {"5": [0.1, 0.0, 0.5]},
        "safety_action": "ABORT",
        "scheduler_reason": "hold",
        "frame_valid": False,
        "frame_condition_number": 5.0,
        "follower_command_norms": {"5": 0.5},
    },
]

with open(tmp_path, "w", encoding="utf-8") as fh:
    fh.write(json.dumps(header_line, ensure_ascii=False) + "\n")
    for entry in events_lines:
        fh.write(json.dumps(entry, ensure_ascii=False) + "\n")
    for entry in record_lines:
        fh.write(json.dumps(entry, ensure_ascii=False) + "\n")


stream = iter_telemetry(str(tmp_path))
assert stream.header is not None
assert stream.header["schema_version"] == 2
assert stream.header["config_fingerprint"]["config_sha256"] == "abc123"
assert len(stream.records) == 2
assert len(stream.events) == 9
assert "kind" not in stream.records[0]


loaded = load_records(str(tmp_path))
assert len(loaded) == 2
assert "kind" not in loaded[0]


summary = analyze_telemetry(stream)
assert summary["record_count"] == 2
assert summary["config_fingerprint"]["config_sha256"] == "abc123"
assert summary["event_counts"]["wait_for_params"] == 1
assert summary["watchdog_summary"]["total"] == 4
assert summary["watchdog_summary"]["by_mode"]["telemetry"] == 1
assert summary["watchdog_summary"]["by_mode"]["hold"] == 1
assert summary["watchdog_summary"]["by_mode"]["degrade"] == 1
assert summary["watchdog_summary"]["by_mode"]["degrade_recovered"] == 1
assert summary["executor_failure_summary"]["total"] == 2
assert summary["executor_failure_summary"]["by_action"]["degrade"] == 1
assert summary["executor_failure_summary"]["by_action"]["hold"] == 1
assert summary["executor_failure_summary"]["by_group"]["2"] == 2
assert summary["executor_failure_summary"]["failure_categories"]["timeout"] == 1
assert summary["executor_failure_summary"]["failure_categories"]["link_lookup"] == 1
assert summary["executor_failure_summary"]["retryable_counts"]["retryable"] == 1
assert summary["executor_failure_summary"]["retryable_counts"]["non_retryable"] == 1
assert summary["safety_counts"]["EXECUTE"] == 1
assert summary["safety_counts"]["ABORT"] == 1
assert summary["max_command_norm_per_drone"]["5"] == 0.5
assert summary["fresh_sample_rate"] == 0.75
assert summary["effective_update_rate_hz"] == 10.0
assert summary["phase_counts"]["formation_run"] == 4
assert summary["per_drone_tracking_error"]["1"]["count"] == 2
assert summary["per_drone_tracking_error"]["5"]["count"] == 2
assert summary["per_drone_tracking_error"]["1"]["p95"] is not None
assert summary["role_tracking_error"]["leader"]["count"] == 2
assert summary["role_tracking_error"]["follower"]["count"] == 2
assert summary["phase_tracking_error"]["formation_run"]["count"] == 4
assert summary["phase_role_tracking_error"]["formation_run"]["leader"]["count"] == 2
assert summary["formation_error"]["count"] == 2
assert summary["formation_error"]["p95"] is not None
assert summary["formation_error"]["max"] is not None

replay = build_replay(loaded, stream.events)
assert replay["summary"]["last_mission_state"] == "ABORT"
assert replay["summary"]["watchdog_summary"]["by_event"]["hold_entered"] == 1
assert replay["summary"]["executor_failure_summary"]["by_event"]["executor_group_hold"] == 1


# Legacy-format fixture (records inline ``phase_events``, no ``kind`` tag) still works.
legacy_path = Path("telemetry") / "test_replay_legacy.jsonl"
legacy_records = [
    {
        "mission_state": "RUN",
        "mission_elapsed": 0.0,
        "config_fingerprint": {"config_sha256": "legacy"},
        "snapshot_seq": 1,
        "snapshot_t_meas": 0.0,
        "measured_positions": {"1": [0.0, 0.0, 0.5], "5": [0.2, 0.0, 0.5]},
        "fresh_mask": {"1": True, "5": True},
        "disconnected_ids": [],
        "phase_events": [
            {"event": "wait_for_params", "details": {"drone_id": 1}},
            {
                "event": "velocity_stream_watchdog",
                "details": {
                    "code": "RUNTIME_VELOCITY_STREAM_WATCHDOG",
                    "category": "runtime",
                    "stage": "velocity_stream_watchdog",
                },
            },
        ],
        "phase_label": "formation_run",
        "leader_mode": "batch_goto",
        "leader_reference_positions": {"1": [0.1, 0.0, 0.5]},
        "follower_reference_positions": {"5": [0.1, 0.0, 0.5]},
        "safety_action": "EXECUTE",
        "scheduler_reason": "execute",
        "frame_valid": True,
        "frame_condition_number": 1.0,
        "follower_command_norms": {"5": 0.2},
    }
]
with open(legacy_path, "w", encoding="utf-8") as fh:
    for record in legacy_records:
        fh.write(json.dumps(record, ensure_ascii=False) + "\n")

legacy_stream = iter_telemetry(str(legacy_path))
assert legacy_stream.header is None
assert len(legacy_stream.records) == 1
assert len(legacy_stream.events) == 2
legacy_summary = analyze_telemetry(legacy_stream)
assert legacy_summary["config_fingerprint"]["config_sha256"] == "legacy"
assert legacy_summary["event_counts"]["wait_for_params"] == 1
legacy_direct_summary = analyze_records(legacy_stream.records)
assert legacy_direct_summary["event_counts"]["wait_for_params"] == 1

print("[OK] Telemetry replay analysis verified")
