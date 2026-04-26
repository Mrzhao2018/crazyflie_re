"""对 telemetry JSONL 做一次性 load + summary, 并按路径缓存.

直接复用 [src/runtime/telemetry_replay.py](../../runtime/telemetry_replay.py)
的 iter_telemetry / analyze_telemetry, 不重复 schema v2 解析逻辑.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

from ...runtime.telemetry_replay import (
    TelemetryStream,
    analyze_telemetry,
    iter_telemetry,
)


@dataclass
class LoadedRun:
    telemetry_path: Path
    stream: TelemetryStream
    summary: dict


class TelemetryLoader:
    """LRU 缓存包装, 多次访问同一 run 详情页不重复 parse."""

    def __init__(self, cache_size: int = 8) -> None:
        if cache_size < 1:
            raise ValueError("cache_size must be >= 1")
        self._cache_size = cache_size
        self._cache: OrderedDict[str, LoadedRun] = OrderedDict()

    def load(self, path: Path) -> LoadedRun:
        key = str(path.resolve())
        if key in self._cache:
            self._cache.move_to_end(key)
            return self._cache[key]
        stream = iter_telemetry(key)
        summary = analyze_telemetry(stream)
        loaded = LoadedRun(telemetry_path=Path(key), stream=stream, summary=summary)
        self._cache[key] = loaded
        if len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)
        return loaded

    def summary_for_phase(self, run: LoadedRun, phase: str | None) -> dict:
        """把 summary 按 phase 过滤. None 表示所有阶段.

        phase 不存在于记录中时, formation_error/role_tracking_error 置为空
        stats 字典 (与 analyze_records._series_stats([]) 一致).
        """
        if phase is None:
            return run.summary
        phase_errors = run.summary.get("phase_tracking_error", {}).get(phase)
        phase_role_errors = run.summary.get("phase_role_tracking_error", {}).get(
            phase, {}
        )
        empty_stats = {
            "count": 0,
            "mean": None,
            "rmse": None,
            "p95": None,
            "max": None,
        }
        return {
            **run.summary,
            "formation_error": phase_errors if phase_errors is not None else empty_stats,
            "role_tracking_error": {
                "leader": phase_role_errors.get("leader", empty_stats),
                "follower": phase_role_errors.get("follower", empty_stats),
            },
            "filtered_phase": phase,
        }
