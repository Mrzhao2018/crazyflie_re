"""Telemetry review helper tests"""

from src.runtime.telemetry import TelemetryRecorder, TelemetryRecord
import numpy as np


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
        readiness={"pose_ready": np.bool_(True)},
        phase_events=telemetry.phase_events(),
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

replay = telemetry.export_replay()
assert len(replay["phase_events"]) == 2
assert len(replay["records"]) == 1
assert replay["records"][0]["mission_elapsed"] == 1.25
assert replay["records"][0]["trajectory_state"] == "running"
assert replay["records"][0]["measured_positions"][1] == [0.0, 0.0, 0.5]
assert replay["records"][0]["leader_reference_positions"][1] == [0.1, 0.0, 0.5]

print("[OK] Telemetry review helpers verified")
