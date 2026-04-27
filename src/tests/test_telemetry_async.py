"""TelemetryRecorder: 后台 writer drain + close 契约"""

import json
import tempfile
from pathlib import Path

from src.runtime.telemetry import TelemetryRecorder, TelemetryRecord


def _make_record(seq: int) -> TelemetryRecord:
    return TelemetryRecord(
        t_wall=0.0,
        mission_state="RUN",
        startup_mode="auto",
        mission_elapsed=0.1 * seq,
        trajectory_state="running",
        trajectory_terminal_reason=None,
        snapshot_seq=seq,
        snapshot_t_meas=0.0,
        measured_positions={1: [0.0, 0.0, 0.5]},
        fresh_mask={1: True},
        disconnected_ids=[],
        health={1: {"pm.vbat": 4.0}},
        frame_valid=True,
        frame_condition_number=1.0,
        phase_label="formation_run",
        leader_mode="batch_goto",
        leader_reference_positions={1: [0.1, 0.0, 0.5]},
        follower_reference_positions={5: [0.0, 0.2, 0.5]},
        safety_action="EXECUTE",
        safety_reasons=[],
        safety_reason_codes=[],
        scheduler_reason="execute",
        scheduler_diagnostics={"fresh": True},
        leader_reference_source="L",
        manual_axis=None,
        manual_input_age=None,
        leader_action_count=1,
        follower_action_count=2,
        follower_command_norms={5: 0.1},
    )


with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "t.jsonl"
    rec = TelemetryRecorder()
    rec.open(str(path))
    rec.write_header(
        config_fingerprint={"x": 1}, readiness={"pose_ready": True}, fleet_meta={"n": 1}
    )
    rec.record_event("hello", ok=True)
    for i in range(100):
        rec.log(_make_record(i))
    rec.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    kinds = [json.loads(line)["kind"] for line in lines]
    assert kinds[0] == "header"
    assert kinds.count("event") == 1
    assert kinds.count("record") == 100

# 不 open 的情况下 log/record_event 仍然不崩，并且 summary 准确
rec2 = TelemetryRecorder()
rec2.record_event("in_memory_only", ok=True)
rec2.log(_make_record(0))
summary = rec2.summary()
assert summary["record_count"] == 1
assert summary["event_counts"]["in_memory_only"] == 1
rec2.close()

with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "late_open.jsonl"
    rec3 = TelemetryRecorder()
    rec3.write_header(config_fingerprint={"late": True})
    rec3.record_event("before_open", ok=True)
    rec3.open(str(path))
    rec3.record_event("after_open", ok=True)
    rec3.flush()
    rec3.close()

    entries = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert entries[0]["kind"] == "header"
    assert entries[0]["config_fingerprint"] == {"late": True}
    assert [entry["event"] for entry in entries if entry["kind"] == "event"] == [
        "before_open",
        "after_open",
    ]

print("[OK] TelemetryRecorder async drain & in-memory modes verified")
