"""Trajectory budget dry-run summary tests."""

from src.app.trajectory_budget_summary import build_trajectory_budget_summary


summary = build_trajectory_budget_summary("config")
assert "trajectory_enabled" in summary
assert "mission_duration" in summary
assert "trajectory_sample_dt" in summary
assert "trajectory_time_scale" in summary
assert "angular_rate" in summary
assert "phase_schedule" in summary
if summary["trajectory_enabled"]:
    assert summary["mode"] == "trajectory"
    assert summary["leaders"]
    for drone_id, item in summary["leaders"].items():
        assert item["pieces"] >= 1
        assert item["estimated_bytes"] >= 132
        assert item["memory_capacity"] == 4096

print("[OK] Trajectory budget dry-run summary verified")
