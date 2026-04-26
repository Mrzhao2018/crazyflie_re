"""Runtime link-state store.

Tracks the latest connected/disconnected state per drone based on cflib
connection callbacks (`connected`, `connection_lost`, `disconnected`) and keeps a
small pending-event queue so the main thread can mirror state transitions into
telemetry without having the adapter layer depend directly on TelemetryRecorder.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class LinkStateSample:
    state: str  # "connected" | "disconnected"
    t_wall: float
    error: str | None = None


class LinkStateBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[int, LinkStateSample] = {}
        self._pending_events: list[dict[str, object]] = []

    def mark_connected(self, drone_id: int, *, t_wall: float | None = None) -> None:
        t_wall = time.time() if t_wall is None else float(t_wall)
        with self._lock:
            previous = self._latest.get(drone_id)
            sample = LinkStateSample(state="connected", t_wall=t_wall, error=None)
            self._latest[drone_id] = sample
            if previous is None or previous.state != "connected":
                self._pending_events.append(
                    {
                        "drone_id": int(drone_id),
                        "state": "connected",
                        "t_wall": t_wall,
                        "error": None,
                    }
                )

    def mark_disconnected(
        self,
        drone_id: int,
        *,
        error: str | None = None,
        t_wall: float | None = None,
    ) -> None:
        t_wall = time.time() if t_wall is None else float(t_wall)
        with self._lock:
            previous = self._latest.get(drone_id)
            sample = LinkStateSample(
                state="disconnected", t_wall=t_wall, error=error
            )
            self._latest[drone_id] = sample
            if (
                previous is None
                or previous.state != "disconnected"
                or previous.error != error
            ):
                self._pending_events.append(
                    {
                        "drone_id": int(drone_id),
                        "state": "disconnected",
                        "t_wall": t_wall,
                        "error": error,
                    }
                )

    def latest(self) -> dict[int, LinkStateSample]:
        with self._lock:
            return dict(self._latest)

    def disconnected_ids(self) -> list[int]:
        with self._lock:
            return sorted(
                drone_id
                for drone_id, sample in self._latest.items()
                if sample.state == "disconnected"
            )

    def drain_events(self) -> list[dict[str, object]]:
        with self._lock:
            events = list(self._pending_events)
            self._pending_events.clear()
            return events
