"""Minimal offline smoke runner for the control/reference pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .bootstrap import build_core_app
from ..runtime.offline_swarm_sampler import sample_offline_swarm


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a minimal offline control/reference smoke test."
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--total-time", type=float, default=None)
    parser.add_argument(
        "--output",
        default=None,
        help="optional output json path; default prints to stdout only",
    )
    return parser


def build_offline_smoke_summary(
    config_dir: str = "config",
    dt: float = 0.25,
    total_time: float | None = None,
) -> dict:
    components = build_core_app(config_dir)
    replay = sample_offline_swarm(components, dt=dt, total_time=total_time)
    return {
        "config_dir": components.get("config_dir"),
        "startup_mode": components.get("startup_mode"),
        "sample_count": len(replay.times),
        "drone_ids": replay.drone_ids,
        "leader_ids": components["fleet"].leader_ids(),
        "follower_ids": components["fleet"].follower_ids(),
        "phase_labels": sorted(set(replay.phase_labels)),
        "all_frame_valid": all(replay.frame_valid),
        "all_follower_valid": all(replay.follower_valid),
        "frame_condition_min": min(replay.frame_condition_numbers),
        "frame_condition_max": max(replay.frame_condition_numbers),
        "leader_modes": sorted(set(replay.leader_modes)),
        "first_time": replay.times[0] if replay.times else None,
        "last_time": replay.times[-1] if replay.times else None,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary = build_offline_smoke_summary(
        config_dir=args.config_dir,
        dt=args.dt,
        total_time=args.total_time,
    )
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered, encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
