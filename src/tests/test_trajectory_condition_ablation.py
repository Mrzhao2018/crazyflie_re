"""Phase 1C trajectory condition ablation helper tests."""

import json
import tempfile
from pathlib import Path

from scripts.generate_baseline_sweep import run_sweep
from scripts.generate_trajectory_condition_ablation import run_condition_ablation


with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "trajectory_condition_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    baseline = run_sweep(
        config_dir="config",
        output_path=str(baseline_path),
        dt=5.0,
        total_time=10.0,
        grid={"gain_xy": [1.4], "gain_z": [0.6], "max_velocity": [0.55]},
    )
    result = run_condition_ablation(
        config_dir="config",
        output_path=str(output_path),
        dt=5.0,
        total_time=10.0,
        condition_soft_limit=3.05,
        condition_penalty_scale=1.0,
        condition_stress_enabled=True,
        condition_stress_min_scale=0.15,
        baseline_results_path=str(baseline_path),
    )
    assert len(result["trials"]) == 2
    assert result["baseline_parameters"] == baseline["best_trial"]["parameters"]
    assert result["trials"][0]["summary"]["trajectory_quality_summary"] is not None
    assert result["trials"][1]["summary"]["trajectory_quality_summary"] is not None
    assert result["trials"][1]["summary"]["trajectory_quality_summary"]["condition_penalty_enabled"] is True
    assert result["trials"][1]["summary"]["trajectory_quality_summary"]["penalty_active"] is True
    assert result["trials"][1]["summary"]["trajectory_quality_summary"]["penalized_samples"] > 0
    assert result["trials"][1]["summary"]["trajectory_quality_summary"]["raw_condition_number_max"] > result["trials"][1]["summary"]["trajectory_quality_summary"]["condition_soft_limit"]
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["comparison"]["penalized_samples_delta"] is not None

print("[OK] Trajectory condition ablation helper verified")