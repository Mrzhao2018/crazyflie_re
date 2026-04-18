"""最小遥测记录器 —— 后台 writer 线程 + queue。

schema_version=2 三种 kind：header / event / record。
主循环线程只做入队；真正的 json.dumps + fh.write 发生在 daemon writer 线程里。
`close()` 会 drain 队列并 join 线程。
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import json
import queue
import threading
import time
from collections import Counter

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


SCHEMA_VERSION = 2
_DEFAULT_FLUSH_EVERY_N = 50
_QUEUE_MAX_SIZE = 4096  # record-heavy 长任务上限；超过后主循环会丢 record（但保留 event）


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
        self._flush_every_n = max(1, int(flush_every_n))
        self._queue: queue.Queue[tuple[str, dict] | None] = queue.Queue(
            maxsize=_QUEUE_MAX_SIZE
        )
        self._writer_thread: threading.Thread | None = None
        self._records_dropped = 0

    # ---- lifecycle -------------------------------------------------------

    def open(self, path: str) -> None:
        self._fh = open(path, "w", encoding="utf-8")
        self._header_written = False
        self._start_writer()

    def _start_writer(self) -> None:
        if self._writer_thread is not None and self._writer_thread.is_alive():
            return
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="telemetry-writer", daemon=True
        )
        self._writer_thread.start()

    def close(self) -> None:
        if self._writer_thread is not None:
            self._queue.put(None)  # sentinel
            self._writer_thread.join(timeout=5.0)
            self._writer_thread = None
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None

    # ---- producer-side API ----------------------------------------------

    def write_header(
        self,
        *,
        config_fingerprint: dict | None = None,
        readiness: dict | None = None,
        fleet_meta: dict | None = None,
    ) -> None:
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
        self._queue.put(("header", header))
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
        try:
            self._queue.put_nowait(("record", payload))
        except queue.Full:
            self._records_dropped += 1

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
        # event 不能丢 —— 使用阻塞 put（主循环可容忍毫秒级阻塞）
        self._queue.put(("event", line_payload))

    # ---- writer-side -----------------------------------------------------

    def _writer_loop(self) -> None:
        pending_since_flush = 0
        while True:
            item = self._queue.get()
            if item is None:  # shutdown sentinel
                if self._fh is not None:
                    self._fh.flush()
                return
            kind, payload = item
            if self._fh is None:
                continue
            if kind == "record":
                safe_payload = {"kind": "record", **self._json_safe_record(
                    {k: v for k, v in payload.items() if k != "kind"}
                )}
            else:
                safe_payload = self._json_safe(payload)
            self._fh.write(json.dumps(safe_payload, ensure_ascii=False) + "\n")
            pending_since_flush += 1
            # event / header 立即 flush，record 批量 flush
            if kind in ("event", "header") or pending_since_flush >= self._flush_every_n:
                self._fh.flush()
                pending_since_flush = 0

    # ---- observation -----------------------------------------------------

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
            "records_dropped": self._records_dropped,
        }

    # ---- json normalization ---------------------------------------------

    _FLOAT_ROUND_DIGITS = 9

    @staticmethod
    def _dump_value(value):
        """非递归地把 record 字段里可能出现的 numpy / 容器类型压成 json-safe。"""
        if value is None:
            return None
        if isinstance(value, (str, int, bool)):
            return value
        if isinstance(value, float):
            return round(value, TelemetryRecorder._FLOAT_ROUND_DIGITS)
        if np is not None:
            if isinstance(value, np.bool_):
                return bool(value)
            if isinstance(value, np.integer):
                return int(value)
            if isinstance(value, np.floating):
                return round(float(value), TelemetryRecorder._FLOAT_ROUND_DIGITS)
            if isinstance(value, np.ndarray):
                return value.tolist()
        return value

    def _json_safe_record(self, record_dict: dict) -> dict:
        """针对 TelemetryRecord.asdict() 结果的快速路径。"""
        out: dict = {}
        for key, value in record_dict.items():
            if isinstance(value, dict):
                out[key] = {
                    self._dump_value(k): (
                        self._json_safe(v) if isinstance(v, (dict, list, tuple)) else self._dump_value(v)
                    )
                    for k, v in value.items()
                }
            elif isinstance(value, list):
                out[key] = [
                    self._json_safe(v) if isinstance(v, (dict, list, tuple)) else self._dump_value(v)
                    for v in value
                ]
            else:
                out[key] = self._dump_value(value)
        return out

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
                return round(float(value), self._FLOAT_ROUND_DIGITS)
        if isinstance(value, float):
            return round(value, self._FLOAT_ROUND_DIGITS)
        return value
