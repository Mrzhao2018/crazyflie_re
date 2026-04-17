"""Grid-search helper for Phase 1A baseline tuning."""

from __future__ import annotations

import itertools
import json
import shutil
import subprocess
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
import argparse

import yaml


DEFAULT_GRID = {
    "gain_xy": [1.0, 1.2, 1.4],
    "gain_z": [0.8, 0.9, 1.0],
    "max_velocity": [0.6, 0.7, 0.8],
    "feedforward_gain_xy": [0.5, 0.7, 0.9],
    "feedforward_gain_z": [0.25, 0.35, 0.45],
}

QUICK_GRID = {
    "gain_xy": [1.0, 1.2],
    "gain_z": [0.8],
    "max_velocity": [0.7],
    "feedforward_gain_xy": [0.5, 0.7],
    "feedforward_gain_z": [0.35],
}

P_LIMIT_GRID = {
    "gain_xy": [0.9, 1.0, 1.1, 1.2, 1.3, 1.4],
    "gain_z": [0.6, 0.7, 0.8, 0.9, 1.0],
    "max_velocity": [0.55, 0.65, 0.75, 0.85],
}


def _write_trial_config(src_config_dir: Path, dst_config_dir: Path, overrides: dict[str, float]) -> None:
    shutil.copytree(src_config_dir, dst_config_dir)
    fleet_path = dst_config_dir / "fleet.yaml"
    fleet_data = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    control = deepcopy(fleet_data.get("control", {}))
    control.update(overrides)
    fleet_data["control"] = control
    fleet_path.write_text(yaml.safe_dump(fleet_data, allow_unicode=True, sort_keys=False), encoding="utf-8")


def _trial_name(index: int, overrides: dict[str, float]) -> str:
    pieces = [f"{key}={value}" for key, value in sorted(overrides.items())]
    return f"trial_{index:03d}__" + "__".join(pieces)


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


def run_sweep(
    *,
    config_dir: str = "config",
    output_path: str = "artifacts/baseline_results.json",
    dt: float = 0.25,
    total_time: float | None = None,
    grid: dict[str, list[float]] | None = None,
    formation_rmse_threshold: float | None = None,
    limit_trials: int | None = None,
) -> dict:
    search_grid = DEFAULT_GRID if grid is None else grid
    config_root = Path(config_dir)
    results = []

    keys = sorted(search_grid)
    combinations = list(itertools.product(*(search_grid[key] for key in keys)))
    if limit_trials is not None:
        combinations = combinations[: max(0, int(limit_trials))]

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_root = Path(tmp_dir)
        for index, values in enumerate(combinations, start=1):
            overrides = {key: value for key, value in zip(keys, values)}
            trial_name = _trial_name(index, overrides)
            trial_config_dir = tmp_root / trial_name / "config"
            trial_output_path = tmp_root / trial_name / "smoke_summary.json"
            _write_trial_config(config_root, trial_config_dir, overrides)
            summary = _run_trial(
                trial_config_dir,
                trial_output_path,
                dt=dt,
                total_time=total_time,
            )
            results.append(
                {
                    "trial": trial_name,
                    "parameters": overrides,
                    "summary": summary,
                    "passed": (
                        summary.get("formation_rmse") is not None
                        and (
                            formation_rmse_threshold is None
                            or summary["formation_rmse"] <= formation_rmse_threshold
                        )
                    ),
                }
            )

    ranked = sorted(
        results,
        key=lambda item: (
            float("inf") if item["summary"].get("formation_rmse") is None else item["summary"]["formation_rmse"],
            -(item["summary"].get("frame_valid_rate") or 0.0),
        ),
    )
    best = ranked[0] if ranked else None
    rendered = {
        "trial_count": len(results),
        "search_keys": keys,
        "dt": dt,
        "total_time": total_time,
        "formation_rmse_threshold": formation_rmse_threshold,
        "failing_trials": [item["trial"] for item in ranked if not item["passed"]],
        "best_trial": best,
        "results": ranked,
    }
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(rendered, ensure_ascii=False, indent=2), encoding="utf-8")
    return rendered


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a Phase 1A baseline parameter sweep using offline closed-loop simulation."
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--output", default="artifacts/baseline_results.json")
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--total-time", type=float, default=None)
    parser.add_argument("--formation-rmse-threshold", type=float, default=None)
    parser.add_argument(
        "--grid",
        choices=["default", "quick", "p-limit"],
        default="p-limit",
        help="选择参数网格；quick 用于快速试跑。",
    )
    parser.add_argument(
        "--limit-trials",
        type=int,
        default=None,
        help="可选：仅运行前 N 个 trial，用于快速调试。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.grid == "default":
        grid = DEFAULT_GRID
    elif args.grid == "quick":
        grid = QUICK_GRID
    else:
        grid = P_LIMIT_GRID
    rendered = run_sweep(
        config_dir=args.config_dir,
        output_path=args.output,
        dt=args.dt,
        total_time=args.total_time,
        formation_rmse_threshold=args.formation_rmse_threshold,
        grid=grid,
        limit_trials=args.limit_trials,
    )
    print(json.dumps(rendered, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())