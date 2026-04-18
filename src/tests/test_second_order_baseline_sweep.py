"""Second-order baseline sweep helper tests."""

import json
import tempfile
from pathlib import Path

from scripts.generate_second_order_baseline_sweep import run_second_order_sweep


with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "second_order_baseline_results.json"
    result = run_second_order_sweep(
        config_dir="config",
        output_path=str(output_path),
        dt=5.0,
        total_time=10.0,
        grid={
            "gain_xy": [1.0],
            "gain_z": [0.6],
            "max_velocity": [0.55],
            "velocity_feedback_gain": [0.8],
            "acceleration_feedforward_gain": [1.0],
            "damping_coeff": [0.15],
        },
    )
    assert result["trial_count"] == 1
    assert result["best_trial"] is not None
    assert result["best_trial"]["summary"]["formation_rmse"] is not None
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["trial_count"] == 1

print("[OK] Second-order baseline sweep helper verified")