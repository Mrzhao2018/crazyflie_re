"""Offline telemetry replay analysis tests."""

import json
from pathlib import Path

from src.runtime.telemetry_replay import load_records, analyze_records, build_replay


tmp_path = Path("telemetry") / "test_replay.jsonl"
tmp_path.parent.mkdir(parents=True, exist_ok=True)

records = [
    {
        "mission_state": "RUN",
        "mission_elapsed": 0.0,
        "snapshot_seq": 1,
        "snapshot_t_meas": 0.0,
        "measured_positions": {"1": [0.0, 0.0, 0.5], "5": [0.2, 0.0, 0.5]},
        "fresh_mask": {"1": True, "5": True},
        "disconnected_ids": [],
        "phase_events": [
            {"event": "wait_for_params", "details": {"drone_id": 1}},
            {"event": "preflight", "details": {"ok": True}},
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
    },
    {
        "mission_state": "ABORT",
        "mission_elapsed": 0.1,
        "snapshot_seq": 2,
        "snapshot_t_meas": 0.1,
        "measured_positions": {"1": [0.2, 0.0, 0.5], "5": [0.5, 0.0, 0.5]},
        "fresh_mask": {"1": True, "5": False},
        "disconnected_ids": [5],
        "phase_events": [
            {"event": "wait_for_params", "details": {"drone_id": 1}},
            {"event": "emergency_land", "details": {"ok": True}},
        ],
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
    for record in records:
        fh.write(json.dumps(record, ensure_ascii=False) + "\n")

loaded = load_records(str(tmp_path))
assert len(loaded) == 2

summary = analyze_records(loaded)
assert summary["record_count"] == 2
assert summary["event_counts"]["wait_for_params"] == 1
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

replay = build_replay(loaded)
assert replay["summary"]["last_mission_state"] == "ABORT"

print("[OK] Telemetry replay analysis verified")
