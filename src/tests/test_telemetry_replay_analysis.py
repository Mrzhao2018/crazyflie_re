"""Offline telemetry replay analysis tests."""

import json
from pathlib import Path

from src.runtime.telemetry_replay import load_records, analyze_records, build_replay


tmp_path = Path("telemetry") / "test_replay.jsonl"
tmp_path.parent.mkdir(parents=True, exist_ok=True)

records = [
    {
        "mission_state": "RUN",
        "phase_events": [
            {"event": "wait_for_params", "details": {"drone_id": 1}},
            {"event": "preflight", "details": {"ok": True}},
        ],
        "safety_action": "EXECUTE",
        "scheduler_reason": "execute",
        "frame_valid": True,
        "frame_condition_number": 1.0,
        "follower_command_norms": {"5": 0.2},
    },
    {
        "mission_state": "ABORT",
        "phase_events": [
            {"event": "wait_for_params", "details": {"drone_id": 1}},
            {"event": "emergency_land", "details": {"ok": True}},
        ],
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

replay = build_replay(loaded)
assert replay["summary"]["last_mission_state"] == "ABORT"

print("[OK] Telemetry replay analysis verified")
