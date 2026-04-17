"""Phase 1A baseline sweep helper tests."""

import json
import tempfile
from pathlib import Path

from scripts.generate_baseline_sweep import run_sweep


with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "baseline_results.json"
    result = run_sweep(
        config_dir="config",
        output_path=str(output_path),
        dt=5.0,
        total_time=10.0,
        formation_rmse_threshold=1.0,
        grid={
            "gain_xy": [1.0, 1.2],
            "feedforward_gain_xy": [0.5],
        },
    )
    assert result["trial_count"] == 2
    assert result["best_trial"] is not None
    assert result["best_trial"]["summary"]["formation_rmse"] is not None
    assert result["best_trial"]["summary"]["formation_rmse"] > 0.0
    assert result["failing_trials"] == []
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["trial_count"] == 2
    assert len(rendered["results"]) == 2

print("[OK] Baseline sweep helper verified")