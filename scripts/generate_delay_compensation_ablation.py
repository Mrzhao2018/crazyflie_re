"""Delay compensation offline ablation helper for Phase 1B."""

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


def _write_trial_config(
    src_config_dir: Path,
    dst_config_dir: Path,
    *,
    enabled: bool,
    estimated_total_delay_ms: float,
    delay_prediction_gain: float,
    baseline_parameters: dict[str, float] | None = None,
) -> None:
    shutil.copytree(src_config_dir, dst_config_dir)
    fleet_path = dst_config_dir / "fleet.yaml"
    fleet_data = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    control = dict(fleet_data.get("control", {}))
    if baseline_parameters:
        control.update(baseline_parameters)
    control["time_delay_compensation_enabled"] = enabled
    control["estimated_total_delay_ms"] = estimated_total_delay_ms
    control["delay_prediction_gain"] = delay_prediction_gain
    fleet_data["control"] = control
    fleet_path.write_text(
        yaml.safe_dump(fleet_data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def _run_trial(config_dir: Path, output_path: Path, *, dt: float, total_time: float | None) -> dict:
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


def run_delay_ablation(
    *,
    config_dir: str = "config",
    output_path: str = "artifacts/delay_compensation_ablation.json",
    dt: float = 0.25,
    total_time: float | None = None,
    estimated_total_delay_ms: float = 50.0,
    delay_prediction_gain: float = 1.0,
    baseline_results_path: str | None = None,
) -> dict:
    config_root = Path(config_dir)
    trials = []
    baseline_parameters = _load_baseline_parameters(
        Path(baseline_results_path) if baseline_results_path is not None else None
    )

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_root = Path(tmp_dir)
        for enabled in (False, True):
            trial_name = "delay_on" if enabled else "delay_off"
            trial_config_dir = tmp_root / trial_name / "config"
            trial_output_path = tmp_root / trial_name / "smoke_summary.json"
            _write_trial_config(
                config_root,
                trial_config_dir,
                enabled=enabled,
                estimated_total_delay_ms=estimated_total_delay_ms,
                delay_prediction_gain=delay_prediction_gain,
                baseline_parameters=baseline_parameters,
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
                    "enabled": enabled,
                    "estimated_total_delay_ms": estimated_total_delay_ms,
                    "delay_prediction_gain": delay_prediction_gain,
                    "summary": summary,
                }
            )

    off_summary = next(item for item in trials if not item["enabled"])["summary"]
    on_summary = next(item for item in trials if item["enabled"])["summary"]
    rendered = {
        "dt": dt,
        "total_time": total_time,
        "estimated_total_delay_ms": estimated_total_delay_ms,
        "delay_prediction_gain": delay_prediction_gain,
        "baseline_parameters": baseline_parameters,
        "baseline_results_path": baseline_results_path,
        "trials": trials,
        "comparison": {
            "formation_rmse_delta": (on_summary.get("formation_rmse") or 0.0)
            - (off_summary.get("formation_rmse") or 0.0),
            "follower_rmse_delta": (on_summary.get("follower_rmse") or 0.0)
            - (off_summary.get("follower_rmse") or 0.0),
            "frame_valid_rate_delta": (on_summary.get("frame_valid_rate") or 0.0)
            - (off_summary.get("frame_valid_rate") or 0.0),
        },
    }
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(rendered, ensure_ascii=False, indent=2), encoding="utf-8")
    return rendered


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare offline closed-loop performance with and without delay compensation."
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--output", default="artifacts/delay_compensation_ablation.json")
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--total-time", type=float, default=None)
    parser.add_argument("--estimated-total-delay-ms", type=float, default=50.0)
    parser.add_argument("--delay-prediction-gain", type=float, default=1.0)
    parser.add_argument(
        "--baseline-results",
        default=None,
        help="可选：从 baseline_results.json 读取最佳参数并应用到 delay on/off 两组对比。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rendered = run_delay_ablation(
        config_dir=args.config_dir,
        output_path=args.output,
        dt=args.dt,
        total_time=args.total_time,
        estimated_total_delay_ms=args.estimated_total_delay_ms,
        delay_prediction_gain=args.delay_prediction_gain,
        baseline_results_path=args.baseline_results,
    )
    print(json.dumps(rendered, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())