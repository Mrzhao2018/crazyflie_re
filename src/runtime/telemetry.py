"""最小遥测记录器

schema_version=2 将 JSONL 拆分成三种 kind：

- 第 1 行：``{"kind":"header", "schema_version":2, "config_fingerprint":..., "readiness":..., "fleet":{...}}``
- 事件：``{"kind":"event", "t_wall":..., "event":..., "details":{...}}``
- 记录：``{"kind":"record", "t_wall":..., "mission_state":..., ...}``（不再携带全量 ``phase_events`` / ``config_fingerprint`` / ``readiness``）
"""

from dataclasses import dataclass, asdict
import json
from collections import Counter
import time

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


SCHEMA_VERSION = 2
_DEFAULT_FLUSH_EVERY_N = 50


@dataclass
class TelemetryRecord:
    t_wall: float
    mission_state: str
    startup_mode: str | None
    mission_elapsed: float | None
    trajectory_state: str | None
    trajectory_terminal_reason: str | None
    snapshot_seq: int
    snapshot_t_meas: float
    measured_positions: dict
    fresh_mask: dict
    disconnected_ids: list[int]
    health: dict
    frame_valid: bool | None
    frame_condition_number: float | None
    phase_label: str | None
    leader_mode: str | None
    leader_reference_positions: dict
    follower_reference_positions: dict
    safety_action: str
    safety_reasons: list[str]
    safety_reason_codes: list[str]
    scheduler_reason: str | None
    scheduler_diagnostics: dict
    leader_reference_source: str | None
    manual_axis: str | None
    manual_input_age: float | None
    leader_action_count: int
    follower_action_count: int
    follower_command_norms: dict


@dataclass
class _RecordProjection:
    mission_state: str
    safety_action: str
    scheduler_reason: str | None


class TelemetryRecorder:
    def __init__(self, flush_every_n: int = _DEFAULT_FLUSH_EVERY_N):
        self._fh = None
        self._phase_events: list[dict] = []
        self._record_projections: list[_RecordProjection] = []
        self._record_count = 0
        self._header_written = False
        self._pending_since_flush = 0
        self._flush_every_n = max(1, int(flush_every_n))

    def open(self, path: str) -> None:
        self._fh = open(path, "w", encoding="utf-8")
        self._header_written = False

    def write_header(
        self,
        *,
        config_fingerprint: dict | None = None,
        readiness: dict | None = None,
        fleet_meta: dict | None = None,
    ) -> None:
        """Emit the schema_version=2 header line. Safe to call exactly once per open()."""
        if self._fh is None:
            self._header_written = True
            return
        if self._header_written:
            return
        header = {
            "kind": "header",
            "schema_version": SCHEMA_VERSION,
            "t_wall": time.time(),
            "config_fingerprint": self._json_safe(config_fingerprint or {}),
            "readiness": self._json_safe(readiness or {}),
            "fleet": self._json_safe(fleet_meta or {}),
        }
        self._fh.write(json.dumps(header, ensure_ascii=False) + "\n")
        self._fh.flush()
        self._header_written = True

    def log(self, record: TelemetryRecord) -> None:
        self._record_count += 1
        self._record_projections.append(
            _RecordProjection(
                mission_state=record.mission_state,
                safety_action=record.safety_action,
                scheduler_reason=record.scheduler_reason,
            )
        )
        if self._fh is None:
            return
        payload = {"kind": "record", **asdict(record)}
        safe_payload = self._json_safe(payload)
        self._fh.write(json.dumps(safe_payload, ensure_ascii=False) + "\n")
        self._pending_since_flush += 1
        if self._pending_since_flush >= self._flush_every_n:
            self._fh.flush()
            self._pending_since_flush = 0

    def record_event(self, event: str, **details) -> None:
        event_payload = {
            "t_wall": time.time(),
            "event": event,
            "details": details,
        }
        self._phase_events.append(event_payload)
        if self._fh is None:
            return
        line_payload = {"kind": "event", **event_payload}
        self._fh.write(
            json.dumps(self._json_safe(line_payload), ensure_ascii=False) + "\n"
        )
        self._fh.flush()

    def phase_events(self) -> list[dict]:
        return list(self._phase_events)

    def summary(self) -> dict:
        event_counts = Counter(event["event"] for event in self._phase_events)
        safety_counts = Counter(p.safety_action for p in self._record_projections)
        scheduler_reason_counts = Counter(
            p.scheduler_reason
            for p in self._record_projections
            if p.scheduler_reason
        )
        return {
            "event_counts": dict(event_counts),
            "safety_counts": dict(safety_counts),
            "scheduler_reason_counts": dict(scheduler_reason_counts),
            "record_count": self._record_count,
            "last_mission_state": (
                self._record_projections[-1].mission_state
                if self._record_projections
                else None
            ),
        }

    def _json_safe(self, value):
        if isinstance(value, dict):
            return {self._json_safe(k): self._json_safe(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._json_safe(v) for v in value]
        if isinstance(value, tuple):
            return [self._json_safe(v) for v in value]
        if np is not None:
            if isinstance(value, np.ndarray):
                return self._json_safe(value.tolist())
            if isinstance(value, np.bool_):
                return bool(value)
            if isinstance(value, np.integer):
                return int(value)
            if isinstance(value, np.floating):
                return float(value)
        return value

    def close(self) -> None:
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            self._pending_since_flush = 0
