"""CLI-style replay analysis over telemetry JSONL."""

from __future__ import annotations

import json
import sys

from ..runtime.telemetry_replay import load_records, build_replay


def main(argv: list[str] | None = None) -> int:
    argv = argv or sys.argv[1:]
    path = argv[0] if argv else "telemetry/run_real.jsonl"
    replay = build_replay(load_records(path))
    print(json.dumps(replay["summary"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
