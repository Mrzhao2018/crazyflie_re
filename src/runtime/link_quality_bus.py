"""Runtime link-quality state store.

Receives per-metric updates from cflib's ``radio_link_statistics`` callbacks
(``link_quality`` / ``uplink_rssi`` / ``uplink_rate`` / ``downlink_rate`` /
``uplink_congestion`` / ``downlink_congestion``). Aggregates the latest value
of every metric per drone behind a lock so the main loop can read a consistent
snapshot each tick, the same way it reads ``PoseBus`` / ``HealthBus``.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace


SUPPORTED_METRICS: tuple[str, ...] = (
    "link_quality",
    "uplink_rssi",
    "uplink_rate",
    "downlink_rate",
    "uplink_congestion",
    "downlink_congestion",
)


@dataclass
class LinkQualitySample:
    link_quality: float | None = None
    uplink_rssi: float | None = None
    uplink_rate: float | None = None
    downlink_rate: float | None = None
    uplink_congestion: float | None = None
    downlink_congestion: float | None = None
    last_update_t: float | None = None


class LinkQualityBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[int, LinkQualitySample] = {}

    def update(self, drone_id: int, metric: str, value: float, t_wall: float) -> None:
        if metric not in SUPPORTED_METRICS:
            return
        with self._lock:
            current = self._latest.get(drone_id, LinkQualitySample())
            self._latest[drone_id] = replace(
                current, **{metric: float(value), "last_update_t": float(t_wall)}
            )

    def latest(self) -> dict[int, LinkQualitySample]:
        with self._lock:
            return dict(self._latest)
