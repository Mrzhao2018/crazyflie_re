"""Multi-run trajectory comparison smoke test."""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from src.app.trajectory_compare_runs import compare_run_summaries


with tempfile.TemporaryDirectory() as tmp_dir:
    root = Path(tmp_dir)
    run_a = root / "run_a"
    run_b = root / "run_b"
    run_a.mkdir()
    run_b.mkdir()

    summary_a = {
        "alignment_time_base": "mission_elapsed",
        "formation_run_summary": {
            "record_count": 100,
            "frame_valid_rate": 0.8,
            "formation_error": {"mean": 0.12, "rmse": 0.13, "p95": 0.2, "max": 0.25},
            "role_tracking_error": {
                "leader": {"rmse": 0.11, "p95": 0.18},
                "follower": {"rmse": 0.14, "p95": 0.22},
            },
        },
    }
    summary_b = {
        "alignment_time_base": "mission_elapsed",
        "formation_run_summary": {
            "record_count": 120,
            "frame_valid_rate": 0.85,
            "formation_error": {"mean": 0.09, "rmse": 0.1, "p95": 0.15, "max": 0.2},
            "role_tracking_error": {
                "leader": {"rmse": 0.1, "p95": 0.16},
                "follower": {"rmse": 0.11, "p95": 0.17},
            },
        },
    }

    (run_a / "trajectory_comparison_summary.json").write_text(
        json.dumps(summary_a, ensure_ascii=False), encoding="utf-8"
    )
    (run_b / "trajectory_comparison_summary.json").write_text(
        json.dumps(summary_b, ensure_ascii=False), encoding="utf-8"
    )

    result = compare_run_summaries([str(run_a), str(run_b)])
    assert result["run_count"] == 2
    assert result["best_formation_rmse_run"] == "run_b"
    assert result["runs"][0]["formation_rmse"] is not None

    output_json = root / "compare_runs.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "src.app.trajectory_compare_runs",
            str(run_a),
            str(run_b),
            "--output",
            str(output_json),
        ],
        check=True,
    )
    assert output_json.exists()
    assert (root / "compare_overview.png").exists()
    assert (root / "compare_roles.png").exists()

print("[OK] Multi-run trajectory comparison verified")
