"""Telemetry review helper tests"""

import json
import tempfile
from pathlib import Path

import numpy as np

from src.runtime.telemetry import TelemetryRecorder, TelemetryRecord, SCHEMA_VERSION


telemetry = TelemetryRecorder()
telemetry.record_event("wait_for_params", ok=True)
telemetry.record_event("preflight", ok=True)
telemetry.log(
    TelemetryRecord(
        t_wall=0.0,
        mission_state="RUN",
        startup_mode="auto",
        mission_elapsed=1.25,
        trajectory_state="running",
        trajectory_terminal_reason=None,
        snapshot_seq=1,
        snapshot_t_meas=0.0,
        measured_positions={1: [0.0, 0.0, 0.5]},
        fresh_mask={1: np.bool_(True)},
        disconnected_ids=[],
        health={1: {"pm.vbat": np.float64(4.0)}},
        frame_valid=np.bool_(True),
        frame_condition_number=np.float64(1.0),
        phase_label="formation_run",
        leader_mode="batch_goto",
        leader_reference_positions={1: [0.1, 0.0, 0.5]},
        follower_reference_positions={5: [0.0, 0.2, 0.5]},
        safety_action="EXECUTE",
        safety_reasons=[],
        safety_reason_codes=[],
        scheduler_reason="execute",
        scheduler_diagnostics={"fresh": np.bool_(True)},
        leader_reference_source="LeaderReferenceGenerator",
        manual_axis=None,
        manual_input_age=None,
        leader_action_count=1,
        follower_action_count=2,
        follower_command_norms={5: np.float64(0.1), 6: np.float64(0.2)},
    )
)

summary = telemetry.summary()
assert summary["event_counts"]["wait_for_params"] == 1
assert summary["safety_counts"]["EXECUTE"] == 1
assert summary["scheduler_reason_counts"]["execute"] == 1
assert summary["record_count"] == 1
assert summary["last_mission_state"] == "RUN"
assert len(telemetry.phase_events()) == 2


with tempfile.TemporaryDirectory() as tmp_dir:
    path = Path(tmp_dir) / "telemetry.jsonl"
    writer = TelemetryRecorder()
    writer.open(str(path))
    writer.write_header(
        config_fingerprint={"config_sha256": "abc123", "startup_mode": "auto"},
        readiness={"pose_ready": True},
        fleet_meta={"drone_count": 2},
    )
    writer.record_event("wait_for_params", ok=True)
    writer.log(
        TelemetryRecord(
            t_wall=1.0,
            mission_state="RUN",
            startup_mode="auto",
            mission_elapsed=0.5,
            trajectory_state="running",
            trajectory_terminal_reason=None,
            snapshot_seq=1,
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
            leader_reference_source="LeaderReferenceGenerator",
            manual_axis=None,
            manual_input_age=None,
            leader_action_count=1,
            follower_action_count=2,
            follower_command_norms={5: 0.1, 6: 0.2},
        )
    )
    writer.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    parsed = [json.loads(line) for line in lines if line.strip()]
    assert parsed[0]["kind"] == "header"
    assert parsed[0]["schema_version"] == SCHEMA_VERSION
    assert parsed[0]["config_fingerprint"]["config_sha256"] == "abc123"
    assert parsed[0]["readiness"] == {"pose_ready": True}
    assert parsed[0]["fleet"] == {"drone_count": 2}
    kinds = [entry.get("kind") for entry in parsed]
    assert kinds.count("event") >= 1
    assert kinds.count("record") == 1
    record_line = next(entry for entry in parsed if entry.get("kind") == "record")
    assert record_line["mission_state"] == "RUN"
    assert record_line["measured_positions"]["1"] == [0.0, 0.0, 0.5]
    assert "phase_events" not in record_line
    assert "config_fingerprint" not in record_line
    assert "readiness" not in record_line


# write_header is idempotent; the second call must not emit another header.
with tempfile.TemporaryDirectory() as tmp_dir:
    path = Path(tmp_dir) / "telemetry.jsonl"
    writer = TelemetryRecorder()
    writer.open(str(path))
    writer.write_header(config_fingerprint={"config_sha256": "x"})
    writer.write_header(config_fingerprint={"config_sha256": "y"})
    writer.close()
    entries = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert len(entries) == 1
    assert entries[0]["config_fingerprint"]["config_sha256"] == "x"


print("[OK] Telemetry review helpers verified")
