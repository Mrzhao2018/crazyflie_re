"""WSL-native Crazyswarm2/crazyflie_sim mission runner."""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from pathlib import Path

import yaml

from ..adapters.crazyswarm_sim_config import (
    DEFAULT_CRAZYSWARM_YAML,
    generate_crazyswarm2_yaml,
)
from .bootstrap import build_crazyswarm_sim_app
from .run_real import RealMissionApp


def _assert_wsl_runtime() -> None:
    if os.name != "nt":
        return
    raise RuntimeError(
        "ros2-sim must be run inside WSL, not Windows PowerShell. "
        "Open WSL and run: source /opt/ros/jazzy/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && cd /mnt/e/code/crazyflie_re && "
        "python -m src.app.cli ros2-sim --config-dir config --skip-confirm"
    )


def _ros_launch_env() -> dict[str, str]:
    env = os.environ.copy()
    crazyflie_sim_src = (
        Path.home()
        / "ros2_ws"
        / "src"
        / "crazyswarm2"
        / "crazyflie_sim"
    )
    firmware_build = Path.home() / "crazyflie-firmware" / "build"
    entries = []
    if crazyflie_sim_src.exists():
        entries.append(str(crazyflie_sim_src))
    if firmware_build.exists():
        entries.append(str(firmware_build))
    existing = env.get("PYTHONPATH", "")
    if existing:
        entries.append(existing)
    if entries:
        env["PYTHONPATH"] = os.pathsep.join(entries)
    return env


def _server_params_path(crazyflies_yaml: Path, *, sim_backend: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    server_yaml = (
        Path(get_package_share_directory("crazyflie")) / "config" / "server.yaml"
    )
    urdf_path = (
        Path(get_package_share_directory("crazyflie"))
        / "urdf"
        / "crazyflie_description.urdf"
    )
    crazyflies = yaml.safe_load(crazyflies_yaml.read_text(encoding="utf-8"))
    server_data = yaml.safe_load(server_yaml.read_text(encoding="utf-8"))
    params = {
        **crazyflies,
        **server_data["/crazyflie_server"]["ros__parameters"],
    }
    params["robot_description"] = urdf_path.read_text(encoding="utf-8")
    params.setdefault("sim", {})["backend"] = sim_backend
    output_path = crazyflies_yaml.with_name("generated_crazyflie_server_params.yaml")
    output_path.write_text(
        yaml.safe_dump(
            {"/crazyflie_server": {"ros__parameters": params}},
            sort_keys=False,
            default_flow_style=False,
        ),
        encoding="utf-8",
    )
    return output_path


def _launch_crazyflie_sim(args: argparse.Namespace, crazyflies_yaml: Path) -> subprocess.Popen:
    if not args.legacy_launch:
        params_path = _server_params_path(
            crazyflies_yaml, sim_backend=args.sim_backend
        )
        command = [
            "ros2",
            "run",
            "crazyflie_sim",
            "crazyflie_server",
            "--ros-args",
            "--params-file",
            str(params_path),
        ]
        return subprocess.Popen(command, env=_ros_launch_env())

    command = [
        "ros2",
        "launch",
        "crazyflie",
        "launch.py",
        "backend:=sim",
        f"crazyflies_yaml_file:={crazyflies_yaml}",
        "teleop:=False",
        "rviz:=False",
        "gui:=False",
        "mocap:=False",
    ]
    return subprocess.Popen(command, env=_ros_launch_env())


def _stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def _run_takeoff_smoke(args: argparse.Namespace, crazyflies_yaml: Path) -> int:
    launch_process = None
    if args.launch_server:
        print("Launching crazyflie_sim...")
        launch_process = _launch_crazyflie_sim(args, crazyflies_yaml)
        time.sleep(max(0.0, float(args.launch_wait_s)))
        if launch_process.poll() is not None:
            raise RuntimeError(
                f"crazyflie_sim launch exited early with code {launch_process.returncode}"
            )

    try:
        from crazyflie_py import Crazyswarm

        swarm = Crazyswarm()
        cf = swarm.allcfs.crazyfliesById[int(args.smoke_drone_id)]
        time_helper = swarm.timeHelper
        time_helper.sleep(1.0)
        print(f"[smoke] before takeoff cf{args.smoke_drone_id} pos={cf.get_position()}")
        cf.takeoff(targetHeight=args.smoke_height, duration=args.smoke_duration)
        time_helper.sleep(args.smoke_duration + args.smoke_hold_s)
        print(f"[smoke] after takeoff cf{args.smoke_drone_id} pos={cf.get_position()}")
        cf.land(targetHeight=0.0, duration=2.0)
        time_helper.sleep(2.5)
        return 0
    finally:
        _stop_process(launch_process)


def run(args: argparse.Namespace) -> int:
    from .log_setup import configure_logging
    from .startup_progress import make_reporter

    _assert_wsl_runtime()

    reporter = make_reporter(verbose=args.verbose)
    configure_logging(verbose=args.verbose, reporter=reporter)

    crazyflies_yaml = generate_crazyswarm2_yaml(
        args.config_dir,
        args.crazyflies_yaml,
        channel=args.radio_channel,
    )
    print(f"Generated Crazyswarm2 config: {crazyflies_yaml}")

    if args.smoke_only:
        return _run_takeoff_smoke(args, crazyflies_yaml)

    launch_process = None
    if args.launch_server:
        print("Launching crazyflie_sim...")
        launch_process = _launch_crazyflie_sim(args, crazyflies_yaml)
        time.sleep(max(0.0, float(args.launch_wait_s)))
        if launch_process.poll() is not None:
            raise RuntimeError(
                f"crazyflie_sim launch exited early with code {launch_process.returncode}"
            )
    else:
        print("Using existing Crazyswarm2/crazyflie_sim server")

    components = build_crazyswarm_sim_app(
        args.config_dir,
        startup_mode_override=args.startup_mode,
    )
    if args.all_followers:
        components["config"].control.active_follower_ids = None
        print("ros2-sim: enabling all configured followers for this run")
    app = RealMissionApp(components, progress=reporter)

    if not args.skip_confirm:
        print("Press Enter to start the Crazyswarm2 sim mission")
        input()

    try:
        app.start()
        app.run()
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        app.shutdown()
        reporter.close()
        _stop_process(launch_process)
    return 0


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--crazyflies-yaml",
        default=str(DEFAULT_CRAZYSWARM_YAML),
        help="Generated Crazyswarm2 crazyflies.yaml path",
    )
    parser.add_argument(
        "--radio-channel",
        type=int,
        default=90,
        help="Sim URI radio channel; ids still map to URI last byte",
    )
    parser.add_argument(
        "--launch-server",
        dest="launch_server",
        action="store_true",
        default=True,
        help="Launch crazyflie_sim with ros2 launch before starting the mission",
    )
    parser.add_argument(
        "--no-launch-server",
        dest="launch_server",
        action="store_false",
        help="Use an already running crazyflie_sim server",
    )
    parser.add_argument(
        "--launch-wait-s",
        type=float,
        default=2.0,
        help="Seconds to wait after starting ros2 launch before connecting",
    )
    parser.add_argument(
        "--sim-backend",
        choices=["none", "np", "pinocchio", "dynobench", "neuralswarm"],
        default="none",
        help="crazyflie_sim backend. 'none' perfectly tracks setpoints; 'np' is physics but may run slower than wall time.",
    )
    parser.add_argument(
        "--legacy-launch",
        action="store_true",
        help="Use crazyflie launch.py instead of direct crazyflie_sim server launch",
    )
    parser.add_argument(
        "--all-followers",
        action="store_true",
        help="Sim-only override: ignore control.active_follower_ids and control all followers",
    )
    parser.add_argument(
        "--smoke-only",
        action="store_true",
        help="Only run a single-drone Crazyswarm2 takeoff smoke test",
    )
    parser.add_argument("--smoke-drone-id", type=int, default=1)
    parser.add_argument("--smoke-height", type=float, default=0.5)
    parser.add_argument("--smoke-duration", type=float, default=2.0)
    parser.add_argument("--smoke-hold-s", type=float, default=1.0)
