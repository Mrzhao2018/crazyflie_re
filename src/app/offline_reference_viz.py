"""Offline full-swarm reference visualization CLI."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.artist import Artist
from matplotlib.lines import Line2D
from matplotlib.text import Text

from ..runtime.offline_swarm_sampler import (
    OfflineSwarmReplay,
    sample_offline_swarm_from_config,
)


ROLE_STYLE = {
    "leader": {"color": "#1f77b4", "marker": "o", "label": "Leaders"},
    "follower": {"color": "#d62728", "marker": "^", "label": "Followers"},
}


@dataclass
class VisualizationArtifacts:
    replay: OfflineSwarmReplay
    png_path: Path
    gif_path: Path


def _positions_array(replay: OfflineSwarmReplay, drone_id: int) -> np.ndarray:
    raw = replay.drone_positions(drone_id)
    return np.array(
        [[np.nan, np.nan, np.nan] if point is None else point for point in raw],
        dtype=float,
    )


def _axis_limits(
    replay: OfflineSwarmReplay,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    arrays = [_positions_array(replay, drone_id) for drone_id in replay.drone_ids]
    stacked = np.vstack(arrays)
    mins = np.nanmin(stacked, axis=0)
    maxs = np.nanmax(stacked, axis=0)
    spans = np.maximum(maxs - mins, 0.5)
    padding = np.maximum(spans * 0.15, 0.2)
    return (
        (float(mins[0] - padding[0]), float(maxs[0] + padding[0])),
        (float(mins[1] - padding[1]), float(maxs[1] + padding[1])),
        (float(max(mins[2] - padding[2], 0.0)), float(maxs[2] + padding[2])),
    )


def _format_condition(value: float) -> str:
    if np.isfinite(value):
        return f"{value:.2f}"
    return "inf"


def _configure_axes(ax, replay: OfflineSwarmReplay) -> None:
    xlim, ylim, zlim = _axis_limits(replay)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Offline full-swarm reference trajectories")
    ax.view_init(elev=24, azim=38)
    ax.grid(True, alpha=0.3)


def _legend_handles() -> list[Line2D]:
    return [
        Line2D(
            [0],
            [0],
            color=ROLE_STYLE["leader"]["color"],
            marker=ROLE_STYLE["leader"]["marker"],
            linewidth=2.0,
            label="Leaders",
        ),
        Line2D(
            [0],
            [0],
            color=ROLE_STYLE["follower"]["color"],
            marker=ROLE_STYLE["follower"]["marker"],
            linewidth=1.7,
            label="Followers",
        ),
        Line2D([0], [0], color="black", marker="X", linewidth=0.0, label="End"),
    ]


def render_static_plot(replay: OfflineSwarmReplay, output_path: str | Path) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    _configure_axes(ax, replay)

    for drone_id in replay.drone_ids:
        role = replay.roles[drone_id]
        style = ROLE_STYLE[role]
        positions = _positions_array(replay, drone_id)
        ax.plot(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            color=style["color"],
            linewidth=2.0 if role == "leader" else 1.7,
            alpha=0.9,
        )
        ax.scatter(
            positions[0, 0],
            positions[0, 1],
            positions[0, 2],
            color=style["color"],
            marker=style["marker"],
            s=50,
            edgecolors="black",
            linewidths=0.5,
        )
        ax.scatter(
            positions[-1, 0],
            positions[-1, 1],
            positions[-1, 2],
            color=style["color"],
            marker="X",
            s=70,
            edgecolors="black",
            linewidths=0.6,
        )
        end = positions[-1]
        ax.text(end[0], end[1], end[2], f"{role[0].upper()}{drone_id}", fontsize=8)

    ax.legend(handles=_legend_handles(), loc="upper left")
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def render_animation(
    replay: OfflineSwarmReplay,
    output_path: str | Path,
    fps: int = 10,
    trail: int = 20,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    _configure_axes(ax, replay)

    artists: dict[int, tuple[Any, Any]] = {}
    for drone_id in replay.drone_ids:
        role = replay.roles[drone_id]
        style = ROLE_STYLE[role]
        (line,) = ax.plot(
            [],
            [],
            [],
            color=style["color"],
            linewidth=2.0 if role == "leader" else 1.7,
            alpha=0.85,
        )
        (point,) = ax.plot(
            [np.nan],
            [np.nan],
            [np.nan],
            color=style["color"],
            marker=style["marker"],
            linestyle="None",
            markersize=8,
            markeredgecolor="black",
            markeredgewidth=0.5,
        )
        artists[drone_id] = (line, point)

    info = cast(Text, ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top"))
    ax.legend(handles=_legend_handles(), loc="upper left")

    def update(frame_idx: int) -> list[Artist]:
        for drone_id in replay.drone_ids:
            positions = _positions_array(replay, drone_id)
            start_idx = max(0, frame_idx - trail)
            window = positions[start_idx : frame_idx + 1]
            line, point = artists[drone_id]
            line.set_data(window[:, 0], window[:, 1])
            line.set_3d_properties(window[:, 2])
            point.set_data([positions[frame_idx, 0]], [positions[frame_idx, 1]])
            point.set_3d_properties([positions[frame_idx, 2]])

        info.set_text(
            "\n".join(
                [
                    f"t = {replay.times[frame_idx]:.2f}s",
                    f"phase = {replay.phase_labels[frame_idx]}",
                    f"follower_valid = {replay.follower_valid[frame_idx]}",
                    f"frame_cond = {_format_condition(replay.frame_condition_numbers[frame_idx])}",
                ]
            )
        )
        return cast(
            list[Artist],
            [info] + [artist for pair in artists.values() for artist in pair],
        )

    animation = FuncAnimation(
        fig, update, frames=len(replay.times), interval=1000 / max(fps, 1), blit=False
    )
    animation.save(output, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return output


def generate_reference_visualizations(
    config_dir: str = "config",
    output_dir: str | Path = "artifacts/offline_reference_viz",
    dt: float = 0.25,
    fps: int = 10,
    trail: int = 20,
    total_time: float | None = None,
) -> VisualizationArtifacts:
    replay = sample_offline_swarm_from_config(
        config_dir=config_dir, dt=dt, total_time=total_time
    )
    output_root = Path(output_dir)
    png_path = render_static_plot(replay, output_root / "swarm_reference.png")
    gif_path = render_animation(
        replay, output_root / "swarm_reference.gif", fps=fps, trail=trail
    )
    return VisualizationArtifacts(replay=replay, png_path=png_path, gif_path=gif_path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate offline full-swarm reference trajectory visualization."
    )
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--output-dir", default="artifacts/offline_reference_viz")
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--trail", type=int, default=20)
    parser.add_argument("--total-time", type=float, default=None)
    return parser


def run(args: argparse.Namespace) -> int:
    outputs = generate_reference_visualizations(
        config_dir=args.config_dir,
        output_dir=args.output_dir,
        dt=args.dt,
        fps=args.fps,
        trail=args.trail,
        total_time=args.total_time,
    )
    print(f"PNG saved to {outputs.png_path}")
    print(f"GIF saved to {outputs.gif_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
