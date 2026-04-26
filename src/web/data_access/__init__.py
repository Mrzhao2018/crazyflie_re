"""数据访问层 —— Run 索引 + Telemetry 加载.

封装现有 [src/runtime/telemetry_replay.py](../../runtime/telemetry_replay.py)
的 stream/summary 能力, 面向 Web 面板复用.
"""

from .run_index import RunArtifact, RunIndex
from .telemetry_loader import LoadedRun, TelemetryLoader

__all__ = ["RunArtifact", "RunIndex", "LoadedRun", "TelemetryLoader"]
