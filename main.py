"""主入口文件"""

import argparse
import logging
from src.app.bootstrap import build_app
from src.app.run_real import RealMissionApp
from src.app.trajectory_budget_summary import print_trajectory_budget_summary

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--startup-mode",
        choices=["auto", "manual_leader"],
        default=None,
        help="启动模式覆盖：auto 或 manual_leader",
    )
    parser.add_argument(
        "--trajectory-budget",
        action="store_true",
        help="只输出 trajectory 预算摘要，不连接真机",
    )
    args = parser.parse_args()

    if args.trajectory_budget:
        raise SystemExit(print_trajectory_budget_summary("config"))

    print("=== Crazyflie AFC Swarm ===")
    print("构建系统...")

    components = build_app("config", startup_mode_override=args.startup_mode)
    app = RealMissionApp(components)

    print("系统构建完成")
    print("按Enter启动（需要真机连接）")
    input()

    try:
        app.start()
        app.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        app.shutdown()
