"""Telemetry review helper tests"""

from src.runtime.telemetry import TelemetryRecorder, TelemetryRecord


telemetry = TelemetryRecorder()
telemetry.record_event("wait_for_params", ok=True)
telemetry.record_event("preflight", ok=True)
telemetry.log(
    TelemetryRecord(
        t_wall=0.0,
        mission_state="RUN",
        readiness={"pose_ready": True},
        phase_events=telemetry.phase_events(),
        snapshot_seq=1,
        snapshot_t_meas=0.0,
        health={1: {"pm.vbat": 4.0}},
        frame_valid=True,
        frame_condition_number=1.0,
        safety_action="EXECUTE",
        safety_reasons=[],
        safety_reason_codes=[],
        scheduler_reason="execute",
        scheduler_diagnostics={},
        leader_action_count=1,
        follower_action_count=2,
        follower_command_norms={5: 0.1, 6: 0.2},
    )
)

summary = telemetry.summary()
assert summary["event_counts"]["wait_for_params"] == 1
assert summary["safety_counts"]["EXECUTE"] == 1
assert summary["scheduler_reason_counts"]["execute"] == 1

replay = telemetry.export_replay()
assert len(replay["phase_events"]) == 2
assert len(replay["records"]) == 1

print("[OK] Telemetry review helpers verified")
