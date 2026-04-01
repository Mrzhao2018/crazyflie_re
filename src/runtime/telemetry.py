"""最小遥测记录器"""

from dataclasses import dataclass, asdict
import json
from collections import Counter
import time

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


@dataclass
class TelemetryRecord:
    t_wall: float
    mission_state: str
    startup_mode: str | None
    mission_elapsed: float | None
    readiness: dict
    phase_events: list[dict]
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


class TelemetryRecorder:
    def __init__(self):
        self._fh = None
        self._phase_events: list[dict] = []
        self._records: list[TelemetryRecord] = []

    def open(self, path: str) -> None:
        self._fh = open(path, "w", encoding="utf-8")

    def log(self, record: TelemetryRecord) -> None:
        self._records.append(record)
        if self._fh is None:
            return
        safe_payload = self._json_safe(asdict(record))
        self._fh.write(json.dumps(safe_payload, ensure_ascii=False) + "\n")
        self._fh.flush()

    def record_event(self, event: str, **details) -> None:
        self._phase_events.append(
            {"t_wall": time.time(), "event": event, "details": details}
        )

    def phase_events(self) -> list[dict]:
        return list(self._phase_events)

    def summary(self) -> dict:
        event_counts = Counter(event["event"] for event in self._phase_events)
        safety_counts = Counter(record.safety_action for record in self._records)
        scheduler_reason_counts = Counter(
            record.scheduler_reason
            for record in self._records
            if record.scheduler_reason
        )
        return {
            "event_counts": dict(event_counts),
            "safety_counts": dict(safety_counts),
            "scheduler_reason_counts": dict(scheduler_reason_counts),
            "record_count": len(self._records),
            "last_mission_state": self._records[-1].mission_state
            if self._records
            else None,
        }

    def export_replay(self) -> dict:
        return {
            "phase_events": self.phase_events(),
            "records": [self._json_safe(asdict(record)) for record in self._records],
            "summary": self.summary(),
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
            self._fh.close()
            self._fh = None
