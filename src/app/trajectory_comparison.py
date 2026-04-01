"""Thesis-grade real-vs-ideal trajectory comparison outputs."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from ..app.bootstrap import build_core_app
from ..runtime.offline_swarm_sampler import evaluate_offline_swarm_at_time
from ..runtime.telemetry_replay import load_records, analyze_records


@dataclass
class ComparisonArtifacts:
    summary_path: Path
    overlay_png_path: Path
    error_png_path: Path
    summary: dict


def _filter_records_by_phase(records: list[dict], phase_label: str) -> list[dict]:
    return [record for record in records if record.get("phase_label") == phase_label]


def _default_output_dir_for_telemetry(telemetry_path: str | Path) -> Path:
    telemetry_file = Path(telemetry_path)
    return Path("artifacts") / telemetry_file.stem


def resolve_telemetry_path(telemetry_path: str | None) -> Path:
    if telemetry_path:
        return Path(telemetry_path)

    telemetry_dir = Path("telemetry")
    candidates = sorted(
        telemetry_dir.glob("run_real_*.jsonl"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(
            "No telemetry file provided and no telemetry/run_real_*.jsonl files found"
        )
    return candidates[0]


def _sample_time_from_record(record: dict) -> float:
    if record.get("mission_elapsed") is not None:
        return float(record["mission_elapsed"])
    if record.get("snapshot_t_meas") is not None:
        return float(record["snapshot_t_meas"])
    return 0.0


def _aligned_reference_records(records: list[dict], config_dir: str) -> list[dict]:
    if not records:
        return []

    uses_trajectory = any(
        record.get("leader_mode") == "trajectory" for record in records
    )
    if not uses_trajectory:
        return records

    components = build_core_app(config_dir)

    aligned = []
    for record in records:
        t_meas = _sample_time_from_record(record)
        evaluated = evaluate_offline_swarm_at_time(components, t_meas)
        aligned_record = dict(record)
        aligned_record["reference_alignment"] = {
            "mode": "offline_exact_time",
            "reference_time": evaluated["time"],
            "time_delta": 0.0,
            "time_base": (
                "mission_elapsed"
                if record.get("mission_elapsed") is not None
                else "snapshot_t_meas"
            ),
        }
        aligned_record["telemetry_phase_label"] = record.get("phase_label")
        aligned_record["aligned_phase_label"] = evaluated["phase_label"]
        aligned_record["phase_label"] = evaluated["phase_label"]
        aligned_record["leader_mode"] = evaluated["leader_mode"]
        aligned_record["leader_reference_positions"] = evaluated[
            "leader_reference_positions"
        ]
        aligned_record["follower_reference_positions"] = evaluated[
            "follower_reference_positions"
        ]
        aligned.append(aligned_record)
    return aligned


def _series_for_drone(
    records: list[dict], drone_id: int
) -> tuple[list[float], list[list[float]], list[list[float]]]:
    times = []
    measured = []
    reference = []
    key = str(drone_id)
    for record in records:
        measured_positions = record.get("measured_positions", {}) or {}
        ref_positions = {
            **(record.get("leader_reference_positions", {}) or {}),
            **(record.get("follower_reference_positions", {}) or {}),
        }
        measured_pos = measured_positions.get(key, measured_positions.get(drone_id))
        reference_pos = ref_positions.get(key, ref_positions.get(drone_id))
        if measured_pos is None or reference_pos is None:
            continue
        times.append(float(record.get("snapshot_t_meas", 0.0)))
        measured.append([float(v) for v in measured_pos])
        reference.append([float(v) for v in reference_pos])
    return times, measured, reference


def _render_overlay_plot(records: list[dict], output_path: Path, title: str) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    drone_ids = sorted(
        {
            int(drone_id)
            for record in records
            for drone_id in (record.get("measured_positions", {}) or {}).keys()
        }
    )
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    for drone_id in drone_ids:
        _, measured, reference = _series_for_drone(records, drone_id)
        if not measured or not reference:
            continue
        measured_arr = np.array(measured, dtype=float)
        reference_arr = np.array(reference, dtype=float)
        ax.plot(
            reference_arr[:, 0],
            reference_arr[:, 1],
            reference_arr[:, 2],
            linestyle="--",
            label=f"ref {drone_id}",
        )
        ax.plot(
            measured_arr[:, 0],
            measured_arr[:, 1],
            measured_arr[:, 2],
            label=f"meas {drone_id}",
        )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(title)
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _render_error_plot(records: list[dict], output_path: Path, title: str) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    drone_ids = sorted(
        {
            int(drone_id)
            for record in records
            for drone_id in (record.get("measured_positions", {}) or {}).keys()
        }
    )
    fig, ax = plt.subplots(figsize=(10, 5))
    for drone_id in drone_ids:
        times, measured, reference = _series_for_drone(records, drone_id)
        if not measured or not reference:
            continue
        measured_arr = np.array(measured, dtype=float)
        reference_arr = np.array(reference, dtype=float)
        errors = np.linalg.norm(measured_arr - reference_arr, axis=1)
        ax.plot(times, errors, label=f"drone {drone_id}")
    ax.set_xlabel("t_meas [s]")
    ax.set_ylabel("tracking error [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if labels:
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def build_thesis_summary(records: list[dict]) -> dict:
    summary = analyze_records(records)
    return {
        "record_count": summary.get("record_count"),
        "phase_counts": summary.get("phase_counts"),
        "effective_update_rate_hz": summary.get("effective_update_rate_hz"),
        "fresh_sample_rate": summary.get("fresh_sample_rate"),
        "frame_valid_rate": summary.get("frame_valid_rate"),
        "role_tracking_error": summary.get("role_tracking_error"),
        "phase_tracking_error": summary.get("phase_tracking_error"),
        "phase_role_tracking_error": summary.get("phase_role_tracking_error"),
        "per_drone_tracking_error": summary.get("per_drone_tracking_error"),
        "formation_error": summary.get("formation_error"),
    }


def generate_thesis_analysis(
    telemetry_path: str | None = None,
    output_dir: str | Path | None = None,
    config_dir: str = "config",
    include_all_phases: bool = False,
) -> ComparisonArtifacts:
    resolved_telemetry_path = resolve_telemetry_path(telemetry_path)
    records = _aligned_reference_records(
        load_records(str(resolved_telemetry_path)), config_dir
    )
    formation_run_records = _filter_records_by_phase(records, "formation_run")
    active_records = records if include_all_phases else formation_run_records

    summary = {
        "default_phase_scope": (
            "full_mission" if include_all_phases else "formation_run"
        ),
        "alignment_time_base": (
            "mission_elapsed"
            if any(record.get("mission_elapsed") is not None for record in records)
            else "snapshot_t_meas"
        ),
        "formation_run_summary": build_thesis_summary(formation_run_records),
        "full_mission_summary": build_thesis_summary(records),
    }

    output_root = (
        Path(output_dir)
        if output_dir is not None
        else _default_output_dir_for_telemetry(resolved_telemetry_path)
    )
    output_root.mkdir(parents=True, exist_ok=True)
    summary_path = output_root / "trajectory_comparison_summary.json"
    overlay_path = output_root / "trajectory_overlay.png"
    error_path = output_root / "tracking_error.png"

    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    scope_label = "full mission" if include_all_phases else "formation_run"
    _render_overlay_plot(
        active_records,
        overlay_path,
        title=f"Measured vs reference trajectories ({scope_label})",
    )
    _render_error_plot(
        active_records,
        error_path,
        title=f"Tracking error vs time ({scope_label})",
    )

    return ComparisonArtifacts(
        summary_path=summary_path,
        overlay_png_path=overlay_path,
        error_png_path=error_path,
        summary=summary,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate thesis-grade trajectory comparison outputs."
    )
    parser.add_argument(
        "telemetry_path",
        nargs="?",
        default=None,
        help="默认自动选择 telemetry/ 下最新的 run_real_*.jsonl",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="默认按 telemetry 文件名生成独立目录；传此参数可手动指定输出目录",
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument(
        "--include-all-phases",
        action="store_true",
        help="默认只统计 formation_run；加上此参数后图表默认使用全任务记录",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outputs = generate_thesis_analysis(
        telemetry_path=args.telemetry_path,
        output_dir=args.output_dir,
        config_dir=args.config_dir,
        include_all_phases=args.include_all_phases,
    )
    selected = resolve_telemetry_path(args.telemetry_path)
    print(f"Telemetry source: {selected}")
    print(f"Summary saved to {outputs.summary_path}")
    print(f"Overlay plot saved to {outputs.overlay_png_path}")
    print(f"Error plot saved to {outputs.error_png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
