"""Unified CLI contracts."""

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

print("[OK] Unified CLI verified")
