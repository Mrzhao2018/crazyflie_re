"""Compare multiple trajectory comparison summaries across runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


def _resolve_summary_path(path_like: str) -> Path:
    path = Path(path_like)
    if path.is_dir():
        path = path / "trajectory_comparison_summary.json"
    if not path.exists():
        raise FileNotFoundError(f"Summary not found: {path}")
    return path


def _load_summary(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _extract_metrics(summary: dict, source_path: Path) -> dict:
    formation = summary.get("formation_run_summary", {})
    role = formation.get("role_tracking_error", {})
    leader = role.get("leader", {})
    follower = role.get("follower", {})
    return {
        "run": source_path.parent.name,
        "summary_path": str(source_path),
        "alignment_time_base": summary.get("alignment_time_base"),
        "record_count": formation.get("record_count"),
        "frame_valid_rate": formation.get("frame_valid_rate"),
        "formation_mean": (formation.get("formation_error") or {}).get("mean"),
        "formation_rmse": (formation.get("formation_error") or {}).get("rmse"),
        "formation_p95": (formation.get("formation_error") or {}).get("p95"),
        "formation_max": (formation.get("formation_error") or {}).get("max"),
        "leader_rmse": leader.get("rmse"),
        "follower_rmse": follower.get("rmse"),
        "leader_p95": leader.get("p95"),
        "follower_p95": follower.get("p95"),
    }


def compare_run_summaries(paths: list[str]) -> dict:
    resolved = [_resolve_summary_path(path) for path in paths]
    rows = [_extract_metrics(_load_summary(path), path) for path in resolved]
    best_by_rmse = None
    if rows:
        valid_rows = [row for row in rows if row.get("formation_rmse") is not None]
        if valid_rows:
            best_by_rmse = min(valid_rows, key=lambda row: row["formation_rmse"])["run"]
    return {
        "run_count": len(rows),
        "best_formation_rmse_run": best_by_rmse,
        "runs": rows,
    }


def _render_overview_plot(rows: list[dict], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    labels = [row["run"] for row in rows]
    rmse = [row["formation_rmse"] for row in rows]
    p95 = [row["formation_p95"] for row in rows]
    frame_valid = [row["frame_valid_rate"] for row in rows]

    fig, ax1 = plt.subplots(figsize=(12, 5))
    ax1.plot(labels, rmse, marker="o", label="formation RMSE")
    ax1.plot(labels, p95, marker="s", label="formation P95")
    ax1.set_ylabel("error [m]")
    ax1.set_xlabel("run")
    ax1.tick_params(axis="x", rotation=45)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(
        labels, frame_valid, marker="^", color="tab:green", label="frame_valid_rate"
    )
    ax2.set_ylabel("frame valid rate")

    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(handles1 + handles2, labels1 + labels2, loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _render_role_plot(rows: list[dict], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    labels = [row["run"] for row in rows]
    leader = [row["leader_rmse"] for row in rows]
    follower = [row["follower_rmse"] for row in rows]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(labels, leader, marker="o", label="leader RMSE")
    ax.plot(labels, follower, marker="s", label="follower RMSE")
    ax.set_ylabel("rmse [m]")
    ax.set_xlabel("run")
    ax.tick_params(axis="x", rotation=45)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare multiple trajectory comparison summaries across runs."
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="summary json path or run directory containing trajectory_comparison_summary.json",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="optional output json path; default prints to stdout only",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = compare_run_summaries(args.paths)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered, encoding="utf-8")
        _render_overview_plot(
            result["runs"], output_path.with_name("compare_overview.png")
        )
        _render_role_plot(result["runs"], output_path.with_name("compare_roles.png"))
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
