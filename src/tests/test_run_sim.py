"""Offline smoke runner contracts."""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

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

print("[OK] Offline smoke runner verified")
