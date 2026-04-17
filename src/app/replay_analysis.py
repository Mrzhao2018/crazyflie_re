"""CLI-style replay analysis over telemetry JSONL."""

from __future__ import annotations

import argparse
import json
import sys

from ..runtime.telemetry_replay import load_records, build_replay


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarise a telemetry JSONL file."
    )
    parser.add_argument(
        "telemetry_path",
        nargs="?",
        default=None,
        help="默认读取 telemetry/run_real.jsonl",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    path = args.telemetry_path or "telemetry/run_real.jsonl"
    replay = build_replay(load_records(path))
    print(json.dumps(replay["summary"], ensure_ascii=False, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    args = build_parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
