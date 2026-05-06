"""Unified CLI contracts."""

from src.tests.slow_guard import skip_when_fast_marker_requested

skip_when_fast_marker_requested()

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from src.app.cli import build_parser


args = build_parser().parse_args(["budget", "--config-dir", "config"])
assert args.command == "budget"
assert args.config_dir == "config"

args = build_parser().parse_args(
    ["sim", "--config-dir", "config", "--dt", "5.0", "--output", "foo.json"]
)
assert args.command == "sim"
assert args.config_dir == "config"
assert args.dt == 5.0
assert args.output == "foo.json"

args = build_parser().parse_args(["run", "--verbose"])
assert args.verbose is True

args = build_parser().parse_args(["run", "-v"])
assert args.verbose is True

args = build_parser().parse_args(["run"])
assert args.verbose is False

args = build_parser().parse_args(
    [
        "probe-velocity",
        "--config-dir",
        "config",
        "--drone-id",
        "6",
        "--speed",
        "0.12",
        "--segment-s",
        "0.8",
        "--no-notify-before-velocity",
    ]
)
assert args.command == "probe-velocity"
assert args.config_dir == "config"
assert args.drone_id == 6
assert args.speed == 0.12
assert args.segment_s == 0.8
assert args.notify_before_velocity is False

args = build_parser().parse_args(["probe-velocity"])
assert args.notify_before_velocity is True

args = build_parser().parse_args(
    [
        "probe-full-state-circle",
        "--config-dir",
        "config",
        "--drone-id",
        "3",
        "--radius",
        "0.12",
        "--period-s",
        "8.0",
        "--cycles",
        "1.5",
        "--rate-hz",
        "20.0",
        "--z-bias-compensation",
        "--max-z-bias",
        "0.06",
        "--notify-before-full-state",
        "--diagnostic-log",
        "--set-param",
        "ctrlMel.ki_z=0",
        "--set-param",
        "ctrlMel.ki_m_z=0",
    ]
)
assert args.command == "probe-full-state-circle"
assert args.config_dir == "config"
assert args.drone_id == 3
assert args.radius == 0.12
assert args.period_s == 8.0
assert args.cycles == 1.5
assert args.rate_hz == 20.0
assert args.z_bias_compensation is True
assert args.max_z_bias == 0.06
assert args.notify_before_full_state is True
assert args.diagnostic_log is True
assert args.set_param == ["ctrlMel.ki_z=0", "ctrlMel.ki_m_z=0"]

args = build_parser().parse_args(
    [
        "probe-params",
        "--config-dir",
        "config",
        "--drone-id",
        "5",
        "--filter",
        "stabilizer",
        "--filter",
        "mel",
    ]
)
assert args.command == "probe-params"
assert args.config_dir == "config"
assert args.drone_id == 5
assert args.filter == ["stabilizer", "mel"]

args = build_parser().parse_args(["--verbose"])
assert args.verbose is True

args = build_parser().parse_args(
    [
        "ros2-sim",
        "--config-dir",
        "config",
        "--skip-confirm",
        "--no-launch-server",
        "--all-followers",
    ]
)
assert args.command == "ros2-sim"
assert args.config_dir == "config"
assert args.skip_confirm is True
assert args.launch_server is False
assert args.all_followers is True


result = subprocess.run(
    [sys.executable, "-m", "src.app.cli", "budget", "--config-dir", "config"],
    check=True,
    capture_output=True,
    text=True,
)
assert '"trajectory_enabled"' in result.stdout

result = subprocess.run(
    [sys.executable, "-m", "src.app.cli", "--trajectory-budget", "--config-dir", "config"],
    check=True,
    capture_output=True,
    text=True,
)
assert '"trajectory_enabled"' in result.stdout

result = subprocess.run(
    [
        sys.executable,
        "-m",
        "src.app.cli",
        "sim",
        "--config-dir",
        "config",
        "--dt",
        "5.0",
        "--total-time",
        "10.0",
    ],
    check=True,
    capture_output=True,
    text=True,
)
assert '"all_frame_valid": true' in result.stdout.lower()

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "cli_smoke.json"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "src.app.cli",
            "sim",
            "--config-dir",
            "config",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["all_frame_valid"] is True
    assert '"sample_count"' in result.stdout

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "baseline_results.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "2",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
            "--formation-rmse-threshold",
            "1.0",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["trial_count"] == 2
    assert rendered["best_trial"] is not None
    assert '"trial_count"' in result.stdout

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "delay_compensation_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/generate_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "1",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(baseline_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_delay_compensation_ablation.py",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
            "--estimated-total-delay-ms",
            "50.0",
            "--baseline-results",
            str(baseline_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert len(rendered["trials"]) == 2
    assert '"comparison"' in result.stdout

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "trajectory_condition_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/generate_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "1",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(baseline_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_trajectory_condition_ablation.py",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
            "--condition-soft-limit",
            "3.05",
            "--baseline-results",
            str(baseline_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert len(rendered["trials"]) == 2
    assert '"comparison"' in result.stdout

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "model_order_ablation.json"
    baseline_path = Path(tmp_dir) / "baseline_results.json"
    second_order_path = Path(tmp_dir) / "second_order_baseline_results.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/generate_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "1",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(baseline_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "scripts/generate_second_order_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "1",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(second_order_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_model_order_ablation.py",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
            "--baseline-results",
            str(baseline_path),
            "--second-order-baseline-results",
            str(second_order_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert len(rendered["trials"]) == 2
    assert '"comparison"' in result.stdout

with tempfile.TemporaryDirectory() as tmp_dir:
    output_path = Path(tmp_dir) / "second_order_baseline_results.json"
    result = subprocess.run(
        [
            sys.executable,
            "scripts/generate_second_order_baseline_sweep.py",
            "--grid",
            "quick",
            "--limit-trials",
            "1",
            "--dt",
            "5.0",
            "--total-time",
            "10.0",
            "--output",
            str(output_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rendered = json.loads(output_path.read_text(encoding="utf-8"))
    assert rendered["trial_count"] == 1
    assert rendered["best_trial"] is not None
    assert '"trial_count"' in result.stdout

print("[OK] Unified CLI verified")
