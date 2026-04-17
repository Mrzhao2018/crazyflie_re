"""统一的应用命令行入口。"""

from __future__ import annotations

import argparse
import sys
from typing import Callable

from . import (
    offline_reference_viz,
    replay_analysis,
    run_sim,
    trajectory_compare_runs,
    trajectory_comparison,
)
from .run_real import RealMissionApp
from .trajectory_budget_summary import print_trajectory_budget_summary
from .bootstrap import build_app


CommandHandler = Callable[[argparse.Namespace], int]


def _build_config_parent() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--config-dir",
        default="config",
        help="配置目录，默认使用仓库根目录下的 config",
    )
    return parser


def _build_run_parent(
    config_parent: argparse.ArgumentParser,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False, parents=[config_parent])
    parser.add_argument(
        "--startup-mode",
        choices=["auto", "manual_leader"],
        default=None,
        help="启动模式覆盖：auto 或 manual_leader",
    )
    parser.add_argument(
        "--skip-confirm",
        action="store_true",
        help="跳过启动前按 Enter 确认",
    )
    return parser


def _build_sim_parent(
    config_parent: argparse.ArgumentParser,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False, parents=[config_parent])
    parser.add_argument("--dt", type=float, default=0.25)
    parser.add_argument("--total-time", type=float, default=None)
    parser.add_argument(
        "--output",
        default=None,
        help="可选：将 smoke summary 写入 JSON 文件",
    )
    return parser


def _command_config_dir(args: argparse.Namespace) -> str:
    return getattr(args, "config_dir", "config")


def _run_command(args: argparse.Namespace) -> int:
    return _run_real(args.config_dir, args.startup_mode, args.skip_confirm)


def _budget_command(args: argparse.Namespace) -> int:
    return print_trajectory_budget_summary(args.config_dir)


def _command_handlers() -> dict[str, CommandHandler]:
    return {
        "run": _run_command,
        "budget": _budget_command,
        "replay": replay_analysis.run,
        "viz": offline_reference_viz.run,
        "compare": trajectory_comparison.run,
        "compare-runs": trajectory_compare_runs.run,
        "sim": run_sim.run,
    }


def build_parser() -> argparse.ArgumentParser:
    config_parent = _build_config_parent()
    run_parent = _build_run_parent(config_parent)
    sim_parent = _build_sim_parent(config_parent)

    parser = argparse.ArgumentParser(
        description="Crazyflie AFC Swarm unified CLI.",
        parents=[run_parent],
    )
    parser.add_argument(
        "--trajectory-budget",
        action="store_true",
        help="兼容旧入口：只输出 trajectory 预算摘要，不连接真机",
    )

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("run", help="运行真机任务", parents=[run_parent])

    subparsers.add_parser(
        "budget",
        help="输出 trajectory 预算摘要，不连接真机",
        parents=[config_parent],
    )

    replay_parser = subparsers.add_parser("replay", help="输出 telemetry replay 摘要")
    replay_parser.add_argument("telemetry_path", nargs="?", default=None)

    viz_parser = subparsers.add_parser(
        "viz",
        help="生成离线全机群参考轨迹可视化",
        parents=[config_parent],
    )
    viz_parser.add_argument("--output-dir", default="artifacts/offline_reference_viz")
    viz_parser.add_argument("--dt", type=float, default=0.25)
    viz_parser.add_argument("--fps", type=int, default=10)
    viz_parser.add_argument("--trail", type=int, default=20)
    viz_parser.add_argument("--total-time", type=float, default=None)

    compare_parser = subparsers.add_parser(
        "compare",
        help="生成单次 run 的理想轨迹对比",
        parents=[config_parent],
    )
    compare_parser.add_argument("telemetry_path", nargs="?", default=None)
    compare_parser.add_argument("--output-dir", default=None)
    compare_parser.add_argument(
        "--include-all-phases",
        action="store_true",
        help="默认只统计 formation_run；加上此参数后图表默认使用全任务记录",
    )

    compare_runs_parser = subparsers.add_parser(
        "compare-runs", help="汇总多次 run 的对比摘要"
    )
    compare_runs_parser.add_argument("paths", nargs="+")
    compare_runs_parser.add_argument("--output", default=None)
    compare_runs_parser.add_argument(
        "--formation-rmse-threshold",
        type=float,
        default=None,
        help="可选：formation RMSE 回归阈值",
    )
    compare_runs_parser.add_argument(
        "--frame-valid-threshold",
        type=float,
        default=None,
        help="可选：frame valid rate 回归阈值",
    )
    compare_runs_parser.add_argument(
        "--leader-rmse-threshold",
        type=float,
        default=None,
        help="可选：leader RMSE 回归阈值",
    )
    compare_runs_parser.add_argument(
        "--follower-rmse-threshold",
        type=float,
        default=None,
        help="可选：follower RMSE 回归阈值",
    )

    subparsers.add_parser(
        "sim",
        help="运行最小离线控制链 smoke test",
        parents=[sim_parent],
    )

    return parser


def _run_real(config_dir: str, startup_mode: str | None, skip_confirm: bool) -> int:
    print("=== Crazyflie AFC Swarm ===")
    print("构建系统...")

    components = build_app(config_dir, startup_mode_override=startup_mode)
    app = RealMissionApp(components)

    print("系统构建完成")
    if not skip_confirm:
        print("按Enter启动（需要真机连接）")
        input()

    try:
        app.start()
        app.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        app.shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command is None:
        config_dir = _command_config_dir(args)
        if args.trajectory_budget:
            return print_trajectory_budget_summary(config_dir)
        return _run_real(config_dir, args.startup_mode, args.skip_confirm)

    handler = _command_handlers().get(args.command)
    if handler is None:
        raise SystemExit(f"Unsupported command: {args.command}")
    return handler(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
