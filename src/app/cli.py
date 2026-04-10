"""统一的应用命令行入口。"""

from __future__ import annotations

import argparse
import sys
from typing import Callable

from .offline_reference_viz import main as offline_reference_viz_main
from .replay_analysis import main as replay_analysis_main
from .run_real import RealMissionApp
from .run_sim import main as run_sim_main
from .trajectory_budget_summary import print_trajectory_budget_summary
from .trajectory_compare_runs import main as trajectory_compare_runs_main
from .trajectory_comparison import main as trajectory_comparison_main
from .bootstrap import build_app


ArgvBuilder = Callable[[argparse.Namespace], list[str]]
ArgvRunner = Callable[[list[str]], int]
CommandHandler = Callable[[argparse.Namespace], int]


def _build_argv(*items: object) -> list[str]:
    argv: list[str] = []
    for item in items:
        if item is None:
            continue
        if isinstance(item, (list, tuple)):
            argv.extend(str(value) for value in item)
            continue
        argv.append(str(item))
    return argv


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


def _build_replay_args(args: argparse.Namespace) -> list[str]:
    return _build_argv(args.telemetry_path)


def _build_viz_args(args: argparse.Namespace) -> list[str]:
    return _build_argv(
        "--config-dir",
        args.config_dir,
        "--output-dir",
        args.output_dir,
        "--dt",
        args.dt,
        "--fps",
        args.fps,
        "--trail",
        args.trail,
        ["--total-time", args.total_time] if args.total_time is not None else None,
    )


def _build_compare_args(args: argparse.Namespace) -> list[str]:
    return _build_argv(
        args.telemetry_path,
        "--config-dir",
        args.config_dir,
        ["--output-dir", args.output_dir] if args.output_dir is not None else None,
        ["--include-all-phases"] if args.include_all_phases else None,
    )


def _build_compare_runs_args(args: argparse.Namespace) -> list[str]:
    argv = _build_argv(
        args.paths,
        ["--output", args.output] if args.output is not None else None,
    )
    for option, value in (
        ("--formation-rmse-threshold", args.formation_rmse_threshold),
        ("--frame-valid-threshold", args.frame_valid_threshold),
        ("--leader-rmse-threshold", args.leader_rmse_threshold),
        ("--follower-rmse-threshold", args.follower_rmse_threshold),
    ):
        if value is not None:
            argv.extend(_build_argv(option, value))
    return argv


def _build_sim_args(args: argparse.Namespace) -> list[str]:
    return _build_argv(
        "--config-dir",
        args.config_dir,
        "--dt",
        args.dt,
        ["--total-time", args.total_time] if args.total_time is not None else None,
        ["--output", args.output] if args.output is not None else None,
    )


def _run_command(args: argparse.Namespace) -> int:
    return _run_real(args.config_dir, args.startup_mode, args.skip_confirm)


def _budget_command(args: argparse.Namespace) -> int:
    return print_trajectory_budget_summary(args.config_dir)


def _argv_command(
    args: argparse.Namespace,
    builder: ArgvBuilder,
    runner: ArgvRunner,
) -> int:
    return runner(builder(args))


def _replay_command(args: argparse.Namespace) -> int:
    return _argv_command(args, _build_replay_args, replay_analysis_main)


def _viz_command(args: argparse.Namespace) -> int:
    return _argv_command(args, _build_viz_args, offline_reference_viz_main)


def _compare_command(args: argparse.Namespace) -> int:
    return _argv_command(args, _build_compare_args, trajectory_comparison_main)


def _compare_runs_command(args: argparse.Namespace) -> int:
    return _argv_command(args, _build_compare_runs_args, trajectory_compare_runs_main)


def _sim_command(args: argparse.Namespace) -> int:
    return _argv_command(args, _build_sim_args, run_sim_main)


def _command_handlers() -> dict[str, CommandHandler]:
    return {
        "run": _run_command,
        "budget": _budget_command,
        "replay": _replay_command,
        "viz": _viz_command,
        "compare": _compare_command,
        "compare-runs": _compare_runs_command,
        "sim": _sim_command,
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
