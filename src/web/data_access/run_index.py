"""扫描 telemetry/ + artifacts/ 目录, 把一次 run 的产物配对成 RunArtifact.

Run 识别规则:
* ``telemetry_dir/*.jsonl`` 每个文件是一次 run, run_id = 文件 stem
* ``artifacts_dir/<run_id>/`` 若存在则关联为衍生产物目录; 不存在则 None
* run_id 含 ``YYYYMMDD_HHMMSS`` 片段时解析为起始时间, 否则退化成文件 mtime
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

_RUN_TIMESTAMP_PATTERN = re.compile(r"(\d{8})_(\d{6})")


@dataclass(frozen=True)
class RunArtifact:
    run_id: str
    telemetry_path: Path
    artifacts_dir: Path | None
    t_start: datetime

    @property
    def has_artifacts(self) -> bool:
        return self.artifacts_dir is not None and self.artifacts_dir.exists()


def _parse_timestamp(run_id: str) -> datetime | None:
    match = _RUN_TIMESTAMP_PATTERN.search(run_id)
    if match is None:
        return None
    try:
        return datetime.strptime(
            f"{match.group(1)}_{match.group(2)}", "%Y%m%d_%H%M%S"
        )
    except ValueError:
        return None


def _infer_t_start(run_id: str, telemetry_path: Path) -> datetime:
    parsed = _parse_timestamp(run_id)
    if parsed is not None:
        return parsed
    return datetime.fromtimestamp(telemetry_path.stat().st_mtime)


class RunIndex:
    def __init__(self, telemetry_dir: Path, artifacts_dir: Path) -> None:
        self.telemetry_dir = Path(telemetry_dir)
        self.artifacts_dir = Path(artifacts_dir)

    def list_runs(self) -> list[RunArtifact]:
        if not self.telemetry_dir.exists():
            return []
        runs: list[RunArtifact] = []
        for jsonl in self.telemetry_dir.glob("*.jsonl"):
            if not jsonl.is_file():
                continue
            runs.append(self._build_artifact(jsonl.stem, jsonl))
        runs.sort(key=lambda run: run.t_start, reverse=True)
        return runs

    def get(self, run_id: str) -> RunArtifact | None:
        telemetry_path = self.telemetry_dir / f"{run_id}.jsonl"
        if not telemetry_path.is_file():
            return None
        return self._build_artifact(run_id, telemetry_path)

    def _build_artifact(self, run_id: str, telemetry_path: Path) -> RunArtifact:
        artifacts_dir = self.artifacts_dir / run_id
        return RunArtifact(
            run_id=run_id,
            telemetry_path=telemetry_path,
            artifacts_dir=artifacts_dir if artifacts_dir.is_dir() else None,
            t_start=_infer_t_start(run_id, telemetry_path),
        )
