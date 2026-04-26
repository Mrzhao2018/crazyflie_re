"""Offline smoke runner contracts."""

from src.tests.slow_guard import skip_when_fast_marker_requested

skip_when_fast_marker_requested()

import json
import subprocess
import sys
import tempfile
from pathlib import Path
import yaml

from src.app.run_sim import build_offline_smoke_summary


summary = build_offline_smoke_summary("config", dt=5.0, total_time=10.0)
assert summary["sample_count"] >= 3
assert summary["all_frame_valid"] is True
assert summary["all_follower_valid"] is True
assert summary["formation_rmse"] is not None
assert summary["leader_rmse"] is not None
assert summary["follower_rmse"] is not None
assert summary["formation_rmse"] > 0.0
assert summary["follower_rmse"] > 0.0
assert summary["frame_valid_rate"] == 1.0
assert summary["trajectory_quality_summary"] is not None
assert summary["leader_ids"] == [1, 4, 7, 8]
assert summary["follower_ids"] == [2, 3, 5, 6, 9, 10]
assert "trajectory" in summary["leader_modes"]

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "offline_smoke.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "src.app.run_sim",
            "--config-dir",
            "config",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
        ],
        check=True,
    )
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["all_frame_valid"] is True
    assert rendered["sample_count"] == summary["sample_count"]

with tempfile.TemporaryDirectory() as tmp_dir:
    config_dir = Path(tmp_dir) / "config"
    config_dir.mkdir(parents=True)
    for name in ("fleet.yaml", "mission.yaml", "comm.yaml", "safety.yaml", "startup.yaml"):
        source = Path("config") / name
        target = config_dir / name
        target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")

    fleet_path = config_dir / "fleet.yaml"
    fleet_data = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    fleet_data["control"]["dynamics_model_order"] = 2
    fleet_data["control"]["velocity_feedback_gain"] = 0.8
    fleet_data["control"]["acceleration_feedforward_gain"] = 1.0
    fleet_data["control"]["max_acceleration"] = 2.0
    fleet_path.write_text(
        yaml.safe_dump(fleet_data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    second_order_summary = build_offline_smoke_summary(
        str(config_dir), dt=5.0, total_time=10.0
    )
    assert second_order_summary["all_frame_valid"] is True
    assert second_order_summary["all_follower_valid"] is True
    assert second_order_summary["formation_rmse"] is not None
    assert second_order_summary["follower_rmse"] is not None

print("[OK] Offline smoke runner verified")
