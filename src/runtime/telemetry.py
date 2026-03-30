"""最小遥测记录器"""

from dataclasses import dataclass, asdict
import json
from collections import Counter
import time


@dataclass
class TelemetryRecord:
    t_wall: float
    mission_state: str
    readiness: dict
    phase_events: list[dict]
    snapshot_seq: int
    snapshot_t_meas: float
    health: dict
    frame_valid: bool | None
    frame_condition_number: float | None
    safety_action: str
    safety_reasons: list[str]
    safety_reason_codes: list[str]
    scheduler_reason: str | None
    scheduler_diagnostics: dict
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
        self._fh.write(json.dumps(asdict(record), ensure_ascii=False) + "\n")
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
            "records": [asdict(record) for record in self._records],
            "summary": self.summary(),
        }

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
