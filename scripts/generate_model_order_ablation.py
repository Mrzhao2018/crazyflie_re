"""Model-order offline ablation helper for Phase 2B1."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


def _load_baseline_parameters(path: Path | None) -> dict[str, float]:
    if path is None:
        return {}
    baseline = json.loads(path.read_text(encoding="utf-8"))
    best_trial = baseline.get("best_trial") or {}
    parameters = best_trial.get("parameters") or {}
    return {
        str(key): float(value)
        for key, value in parameters.items()
        if isinstance(value, (int, float))
    }


def _load_baseline_payload(path: Path | None) -> dict:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _write_trial_config(
    src_config_dir: Path,
    dst_config_dir: Path,
    *,
    model_order: int,
    velocity_feedback_gain: float,
    acceleration_feedforward_gain: float,
    max_acceleration: float,
    baseline_parameters: dict[str, float] | None = None,
) -> dict[str, float]:
    shutil.copytree(src_config_dir, dst_config_dir)

    fleet_path = dst_config_dir / "fleet.yaml"
    fleet_data = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    control = dict(fleet_data.get("control", {}))
    if baseline_parameters:
        control.update(baseline_parameters)
    control.setdefault("velocity_feedback_gain", velocity_feedback_gain)
    control.setdefault(
        "acceleration_feedforward_gain", acceleration_feedforward_gain
    )
    control.setdefault("max_acceleration", max_acceleration)
    control["dynamics_model_order"] = model_order
    fleet_data["control"] = control
    fleet_path.write_text(
        yaml.safe_dump(fleet_data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return {
        str(key): float(value)
        for key, value in control.items()
        if isinstance(value, (int, float))
    }


def _run_trial(
    config_dir: Path,
    output_path: Path,
    *,
    dt: float,
    total_time: float | None,
) -> dict:
    command = [
        sys.executable,
        "-m",
        "src.app.run_sim",
        "--config-dir",
        str(config_dir),
        "--dt",
        str(dt),
        "--output",
        str(output_path),
    ]
    if total_time is not None:
        command.extend(["--total-time", str(total_time)])
    subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(output_path.read_text(encoding="utf-8"))


def run_model_order_ablation(
    *,
    config_dir: str = "config",
    output_path: str = "artifacts/model_order_ablation.json",
    dt: float = 0.25,
    total_time: float | None = None,
    velocity_feedback_gain: float = 0.8,
    acceleration_feedforward_gain: float = 1.0,
    max_acceleration: float = 2.0,
    baseline_results_path: str | None = None,
    second_order_baseline_results_path: str | None = None,
) -> dict:
    config_root = Path(config_dir)
    trials = []
    baseline_parameters = _load_baseline_parameters(
        Path(baseline_results_path) if baseline_results_path is not None else None
    )
    second_order_baseline_parameters = _load_baseline_parameters(
        Path(second_order_baseline_results_path)
        if second_order_baseline_results_path is not None
        else None
    )

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_root = Path(tmp_dir)
        for model_order in (1, 2):
            trial_name = f"model_order_{model_order}"
            trial_config_dir = tmp_root / trial_name / "config"
            trial_output_path = tmp_root / trial_name / "smoke_summary.json"
            applied_control = _write_trial_config(
                config_root,
                trial_config_dir,
                model_order=model_order,
                velocity_feedback_gain=velocity_feedback_gain,
                acceleration_feedforward_gain=acceleration_feedforward_gain,
                max_acceleration=max_acceleration,
                baseline_parameters=(
                    second_order_baseline_parameters
                    if model_order == 2 and second_order_baseline_parameters
                    else baseline_parameters
                ),
            )
            summary = _run_trial(
                trial_config_dir,
                trial_output_path,
                dt=dt,
                total_time=total_time,
            )
            trials.append(
                {
                    "trial": trial_name,
                    "model_order": model_order,
                    "applied_control": applied_control,
                    "summary": summary,
                }
            )

    first_order = next(item for item in trials if item["model_order"] == 1)["summary"]
    second_order = next(item for item in trials if item["model_order"] == 2)["summary"]
    rendered = {
        "dt": dt,
        "total_time": total_time,
        "velocity_feedback_gain": velocity_feedback_gain,
        "acceleration_feedforward_gain": acceleration_feedforward_gain,
        "max_acceleration": max_acceleration,
        "baseline_parameters": baseline_parameters,
        "baseline_results_path": baseline_results_path,
        "second_order_baseline_parameters": second_order_baseline_parameters,
        "second_order_baseline_results_path": second_order_baseline_results_path,
        "trials": trials,
        "comparison": {
            "formation_rmse_delta": (second_order.get("formation_rmse") or 0.0)
            - (first_order.get("formation_rmse") or 0.0),
            "follower_rmse_delta": (second_order.get("follower_rmse") or 0.0)
            - (first_order.get("follower_rmse") or 0.0),
            "frame_valid_rate_delta": (second_order.get("frame_valid_rate") or 0.0)
            - (first_order.get("frame_valid_rate") or 0.0),
        },
    }
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(
        json.dumps(rendered, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return rendered


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare offline closed-loop performance between first-order and second-order follower models."
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--output", default="artifacts/model_order_ablation.json")
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--total-time", type=float, default=None)
    parser.add_argument("--velocity-feedback-gain", type=float, default=0.8)
    parser.add_argument("--acceleration-feedforward-gain", type=float, default=1.0)
    parser.add_argument("--max-acceleration", type=float, default=2.0)
    parser.add_argument(
        "--baseline-results",
        default=None,
        help="可选：从 baseline_results.json 读取最佳参数并应用到 model_order=1/2 两组对比。",
    )
    parser.add_argument(
        "--second-order-baseline-results",
        default=None,
        help="可选：从 second_order_baseline_results.json 读取最佳二阶参数，仅应用到 model_order=2 对比。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rendered = run_model_order_ablation(
        config_dir=args.config_dir,
        output_path=args.output,
        dt=args.dt,
        total_time=args.total_time,
        velocity_feedback_gain=args.velocity_feedback_gain,
        acceleration_feedforward_gain=args.acceleration_feedforward_gain,
        max_acceleration=args.max_acceleration,
        baseline_results_path=args.baseline_results,
        second_order_baseline_results_path=args.second_order_baseline_results,
    )
    print(json.dumps(rendered, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())