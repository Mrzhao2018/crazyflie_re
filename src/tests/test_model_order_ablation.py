"""Phase 2B1 model-order ablation helper tests."""

import json
import tempfile
from pathlib import Path

from generate_baseline_sweep import run_sweep
from generate_model_order_ablation import run_model_order_ablation
from generate_second_order_baseline_sweep import run_second_order_sweep


with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "model_order_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    baseline = run_sweep(
        config_dir="config",
        output_path=str(baseline_path),
        dt=5.0,
        total_time=10.0,
        grid={"gain_xy": [1.4], "gain_z": [0.6], "max_velocity": [0.55]},
    )
    second_order_path = Path(tmp_dir) / "second_order_baseline_results.json"
    second_order = run_second_order_sweep(
        config_dir="config",
        output_path=str(second_order_path),
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
    result = run_model_order_ablation(
        config_dir="config",
        output_path=str(output_path),
        dt=5.0,
        total_time=10.0,
        baseline_results_path=str(baseline_path),
        second_order_baseline_results_path=str(second_order_path),
    )
    assert len(result["trials"]) == 2
    assert result["baseline_parameters"] == baseline["best_trial"]["parameters"]
    assert result["second_order_baseline_parameters"] == second_order["best_trial"]["parameters"]
    assert {trial["model_order"] for trial in result["trials"]} == {1, 2}
    assert result["trials"][0]["summary"]["formation_rmse"] is not None
    assert result["trials"][1]["summary"]["formation_rmse"] is not None
    second_order_control = result["trials"][1].get("applied_control") or {}
    assert (
        second_order_control.get("velocity_feedback_gain")
        == second_order["best_trial"]["parameters"]["velocity_feedback_gain"]
    )
    assert (
        second_order_control.get("acceleration_feedforward_gain")
        == second_order["best_trial"]["parameters"]["acceleration_feedforward_gain"]
    )
    assert second_order_control.get("dynamics_model_order") == 2.0
    assert output_path.exists()
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["comparison"]["formation_rmse_delta"] is not None

print("[OK] Model-order ablation helper verified")