"""Generate AFC stress/delay/event-triggered ablation matrix."""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path


SOLVER_MODES = ["dense_current", "sparse_convex"]
DELAY_COMPENSATION = [
    {"enabled": False, "estimated_total_delay_ms": 0.0},
    {"enabled": True, "estimated_total_delay_ms": 80.0},
]
EVENT_TRIGGERED_TX = [
    {"enabled": False, "reference_delta": 0.0, "tracking_error": 0.0},
    {"enabled": True, "reference_delta": 0.03, "tracking_error": 0.08},
]
METRICS = [
    "formation_error",
    "per_drone_tracking_error",
    "edge_count",
    "stress_condition_number",
    "radio_link_summary",
    "follower_action_count",
    "safety_counts",
]


def build_afc_ablation_matrix() -> dict:
    trials = []
    for index, (solver_mode, delay, event_tx) in enumerate(
        itertools.product(SOLVER_MODES, DELAY_COMPENSATION, EVENT_TRIGGERED_TX),
        start=1,
    ):
        trials.append(
            {
                "name": f"afc_lab_{index:02d}_{solver_mode}",
                "solver_mode": solver_mode,
                "delay_compensation": dict(delay),
                "event_triggered_tx": dict(event_tx),
            }
        )
    return {
        "solver_modes": SOLVER_MODES,
        "axes": {
            "delay_compensation": DELAY_COMPENSATION,
            "event_triggered_tx": EVENT_TRIGGERED_TX,
        },
        "metrics": METRICS,
        "trials": trials,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate AFC solver/delay/event-triggered ablation matrix."
    )
    parser.add_argument(
        "--output", default="artifacts/afc_solver_lab/ablation_matrix.json"
    )
    args = parser.parse_args(argv)
    matrix = build_afc_ablation_matrix()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(matrix, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(matrix, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
