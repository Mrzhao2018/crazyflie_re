"""Thesis-grade trajectory comparison smoke test."""

import json
import tempfile
from pathlib import Path

from src.app.trajectory_comparison import generate_thesis_analysis


with tempfile.TemporaryDirectory() as tmp_dir:
    tmp_root = Path(tmp_dir)
    telemetry_path = tmp_root / "run_real.jsonl"
    output_dir = tmp_root / "comparison"

    records = [
        {
            "mission_state": "RUN",
            "mission_elapsed": 0.0,
            "snapshot_seq": 1,
            "snapshot_t_meas": 0.0,
            "measured_positions": {"1": [0.0, 0.0, 0.5], "5": [0.2, 0.0, 0.5]},
            "fresh_mask": {"1": True, "5": True},
            "disconnected_ids": [],
            "phase_events": [],
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
            "mission_state": "RUN",
            "mission_elapsed": 0.5,
            "snapshot_seq": 2,
            "snapshot_t_meas": 0.5,
            "measured_positions": {"1": [0.1, 0.0, 0.5], "5": [0.15, 0.0, 0.5]},
            "fresh_mask": {"1": True, "5": True},
            "disconnected_ids": [],
            "phase_events": [],
            "phase_label": "formation_run",
            "leader_mode": "batch_goto",
            "leader_reference_positions": {"1": [0.1, 0.0, 0.5]},
            "follower_reference_positions": {"5": [0.1, 0.0, 0.5]},
            "safety_action": "EXECUTE",
            "scheduler_reason": "execute",
            "frame_valid": True,
            "frame_condition_number": 1.0,
            "follower_command_norms": {"5": 0.1},
        },
    ]

    with open(telemetry_path, "w", encoding="utf-8") as fh:
        for record in records:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    outputs = generate_thesis_analysis(
        telemetry_path=str(telemetry_path),
        config_dir="config",
    )

    assert outputs.summary_path.parent.name == telemetry_path.stem
    assert outputs.summary_path.exists()
    assert outputs.overlay_png_path.exists()
    assert outputs.error_png_path.exists()
    assert outputs.summary_path.stat().st_size > 0
    assert outputs.overlay_png_path.stat().st_size > 0
    assert outputs.error_png_path.stat().st_size > 0
    assert outputs.summary["default_phase_scope"] == "formation_run"
    assert outputs.summary["alignment_time_base"] == "mission_elapsed"
    assert outputs.summary["formation_run_summary"]["effective_update_rate_hz"] == 2.0
    assert (
        outputs.summary["formation_run_summary"]["phase_counts"]["formation_run"] == 4
    )
    assert (
        outputs.summary["formation_run_summary"]["role_tracking_error"]["leader"][
            "count"
        ]
        == 2
    )
    assert (
        outputs.summary["full_mission_summary"]["phase_tracking_error"][
            "formation_run"
        ]["count"]
        == 4
    )

print("[OK] Trajectory comparison smoke path verified")


with tempfile.TemporaryDirectory() as tmp_dir:
    tmp_root = Path(tmp_dir)
    telemetry_path = tmp_root / "run_real_traj.jsonl"
    output_dir = tmp_root / "comparison_traj"

    records = [
        {
            "mission_state": "RUN",
            "mission_elapsed": 3.5,
            "snapshot_seq": 1,
            "snapshot_t_meas": 0.0,
            "measured_positions": {"1": [1.0, 0.0, 0.35], "5": [0.0, 0.72, 0.82]},
            "fresh_mask": {"1": True, "5": True},
            "disconnected_ids": [],
            "phase_events": [],
            "phase_label": "formation_run",
            "leader_mode": "trajectory",
            "leader_reference_positions": {},
            "follower_reference_positions": {},
            "safety_action": "EXECUTE",
            "scheduler_reason": "execute",
            "frame_valid": True,
            "frame_condition_number": 1.0,
            "follower_command_norms": {"5": 0.1},
        }
    ]

    with open(telemetry_path, "w", encoding="utf-8") as fh:
        for record in records:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    outputs = generate_thesis_analysis(
        telemetry_path=str(telemetry_path),
        config_dir="config",
    )

    summary = json.loads(outputs.summary_path.read_text(encoding="utf-8"))
    assert summary["alignment_time_base"] == "mission_elapsed"
    assert outputs.summary_path.parent.name == telemetry_path.stem
    assert summary["formation_run_summary"]["record_count"] == 1
    assert summary["full_mission_summary"]["record_count"] == 1
    assert "formation_run" in summary["full_mission_summary"]["phase_tracking_error"]
    assert outputs.overlay_png_path.exists()
    assert outputs.error_png_path.exists()
    telemetry_records = [
        json.loads(line)
        for line in telemetry_path.read_text(encoding="utf-8").splitlines()
    ]
    assert telemetry_records[0]["leader_reference_positions"] == {}


with tempfile.TemporaryDirectory() as tmp_dir:
    tmp_root = Path(tmp_dir)
    telemetry_path = tmp_root / "run_real_all.jsonl"
    output_dir = tmp_root / "comparison_all"

    records = [
        {
            "mission_state": "LAND",
            "mission_elapsed": 0.0,
            "snapshot_seq": 1,
            "snapshot_t_meas": 0.0,
            "measured_positions": {"1": [0.0, 0.0, 0.5]},
            "fresh_mask": {"1": True},
            "disconnected_ids": [],
            "phase_events": [],
            "phase_label": "settle",
            "leader_mode": "batch_goto",
            "leader_reference_positions": {"1": [0.0, 0.0, 0.5]},
            "follower_reference_positions": {},
            "safety_action": "LAND",
            "scheduler_reason": "land",
            "frame_valid": True,
            "frame_condition_number": 1.0,
            "follower_command_norms": {},
        },
        {
            "mission_state": "RUN",
            "mission_elapsed": 1.0,
            "snapshot_seq": 2,
            "snapshot_t_meas": 1.0,
            "measured_positions": {"1": [0.1, 0.0, 0.5]},
            "fresh_mask": {"1": True},
            "disconnected_ids": [],
            "phase_events": [],
            "phase_label": "formation_run",
            "leader_mode": "batch_goto",
            "leader_reference_positions": {"1": [0.1, 0.0, 0.5]},
            "follower_reference_positions": {},
            "safety_action": "EXECUTE",
            "scheduler_reason": "execute",
            "frame_valid": True,
            "frame_condition_number": 1.0,
            "follower_command_norms": {},
        },
    ]

    with open(telemetry_path, "w", encoding="utf-8") as fh:
        for record in records:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    outputs = generate_thesis_analysis(
        telemetry_path=str(telemetry_path),
        output_dir=output_dir,
        config_dir="config",
        include_all_phases=True,
    )

    assert outputs.summary["default_phase_scope"] == "full_mission"
    assert outputs.summary_path.parent == output_dir
    assert outputs.summary["formation_run_summary"]["record_count"] == 1
    assert outputs.summary["full_mission_summary"]["record_count"] == 2

print("[OK] Trajectory-mode offline alignment verified")
