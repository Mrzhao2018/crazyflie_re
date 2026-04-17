"""Phase 1B delay compensation ablation helper tests."""

import json
import tempfile
from pathlib import Path

from scripts.generate_baseline_sweep import run_sweep
from scripts.generate_delay_compensation_ablation import run_delay_ablation


with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "delay_compensation_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    baseline = run_sweep(
        config_dir="config",
        output_path=str(baseline_path),
        dt=5.0,
        total_time=10.0,
        grid={"gain_xy": [1.0], "gain_z": [0.8], "max_velocity": [0.7]},
    )
    result = run_delay_ablation(
        config_dir="config",
        output_path=str(output_path),
        dt=5.0,
        total_time=10.0,
        estimated_total_delay_ms=50.0,
        delay_prediction_gain=1.0,
        baseline_results_path=str(baseline_path),
    )
    assert len(result["trials"]) == 2
    assert result["baseline_parameters"] == baseline["best_trial"]["parameters"]
    assert result["trials"][0]["summary"]["formation_rmse"] is not None
    assert result["trials"][1]["summary"]["formation_rmse"] is not None
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["comparison"]["formation_rmse_delta"] is not None

print("[OK] Delay compensation ablation helper verified")