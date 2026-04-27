"""Generate velocity/PID vs full_state/Mellinger experiment matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


CONTROL_PROFILES = {
    "velocity_pid_safe": {
        "output_mode": "velocity",
        "onboard_controller": "pid",
        "dynamics_model_order": 2,
        "acceleration_feedforward_gain": 0.5,
    },
    "full_state_mellinger": {
        "output_mode": "full_state",
        "onboard_controller": "mellinger",
        "dynamics_model_order": 2,
        "acceleration_feedforward_gain": 0.5,
    },
}

STANDARD_METRICS = [
    "formation_error",
    "per_drone_tracking_error",
    "role_tracking_error",
    "max_command_norm_per_drone",
    "radio_link_summary",
    "watchdog_summary",
    "executor_failure_summary",
    "safety_counts",
]


def build_experiment_matrix(*, repeats: int = 3) -> dict:
    experiments = []
    for repeat in range(1, max(1, int(repeats)) + 1):
        for profile_name, overrides in CONTROL_PROFILES.items():
            experiments.append(
                {
                    "name": f"{profile_name}_repeat_{repeat:02d}",
                    "profile": profile_name,
                    "repeat": repeat,
                    "control_overrides": dict(overrides),
                    "offline_command": (
                        "python -m src.app.run_sim --config-dir <generated-config> "
                        f"--output artifacts/full_state_ablation/{profile_name}_{repeat:02d}.json"
                    ),
                    "real_hover_command": (
                        "python -m src.app.run_real --config-dir <generated-config>"
                    ),
                }
            )
    return {
        "description": "Standard A/B matrix for velocity/PID vs full_state/Mellinger.",
        "repeat_count": max(1, int(repeats)),
        "profiles": CONTROL_PROFILES,
        "metrics": STANDARD_METRICS,
        "promotion_gate": (
            "Keep velocity_pid_safe as default until full_state_mellinger wins "
            "at least three repeat runs without additional safety events."
        ),
        "experiments": experiments,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate the full-state/Mellinger A/B experiment matrix."
    )
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--output",
        default="artifacts/full_state_ablation/experiment_matrix.json",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    matrix = build_experiment_matrix(repeats=args.repeats)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(matrix, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(matrix, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
