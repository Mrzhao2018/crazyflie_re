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
from statistics import mean


SUPPORTED_METRICS: tuple[str, ...] = (
    "link_quality",
    "uplink_rssi",
    "uplink_rate",
    "downlink_rate",
    "uplink_congestion",
    "downlink_congestion",
    "latency_ms",
)


@dataclass
class LinkQualitySample:
    link_quality: float | None = None
    uplink_rssi: float | None = None
    uplink_rate: float | None = None
    downlink_rate: float | None = None
    uplink_congestion: float | None = None
    downlink_congestion: float | None = None
    latency_ms: float | None = None
    last_update_t: float | None = None
    updated_metric: str | None = None


class LinkQualityBus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[int, LinkQualitySample] = {}
        self._history: dict[int, list[LinkQualitySample]] = {}
        self._history_window_s = 10.0

    def update(self, drone_id: int, metric: str, value: float, t_wall: float) -> None:
        if metric not in SUPPORTED_METRICS:
            return
        with self._lock:
            current = self._latest.get(drone_id, LinkQualitySample())
            self._latest[drone_id] = replace(
                current,
                **{
                    metric: float(value),
                    "last_update_t": float(t_wall),
                    "updated_metric": metric,
                },
            )
            sample = self._latest[drone_id]
            history = self._history.setdefault(drone_id, [])
            history.append(replace(sample))
            cutoff = float(t_wall) - self._history_window_s
            self._history[drone_id] = [
                item
                for item in history
                if item.last_update_t is not None and item.last_update_t >= cutoff
            ]

    def latest(self) -> dict[int, LinkQualitySample]:
        with self._lock:
            return dict(self._latest)

    def recent_samples(self, window_s: float) -> dict[int, list[LinkQualitySample]]:
        with self._lock:
            latest_t = max(
                (
                    sample.last_update_t
                    for sample in self._latest.values()
                    if sample.last_update_t is not None
                ),
                default=None,
            )
            if latest_t is None:
                return {}
            cutoff = latest_t - max(0.0, float(window_s))
            return {
                drone_id: [
                    replace(sample)
                    for sample in samples
                    if sample.last_update_t is not None
                    and sample.last_update_t >= cutoff
                ]
                for drone_id, samples in self._history.items()
            }

    @staticmethod
    def _percentile(values: list[float], percentile: float) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        index = int(round((len(ordered) - 1) * percentile))
        return float(ordered[index])

    def group_health_summary(
        self,
        fleet,
        *,
        window_s: float,
        congestion_soft_floor: float,
        latency_p95_soft_limit_ms: float,
    ) -> dict[int, dict[str, float | int | None]]:
        grouped: dict[int, list[LinkQualitySample]] = {}
        for drone_id, samples in self.recent_samples(window_s).items():
            try:
                group_id = fleet.get_radio_group(drone_id)
            except Exception:
                group_id = 0
            grouped.setdefault(group_id, []).extend(samples)

        summary: dict[int, dict[str, float | int | None]] = {}
        for group_id, samples in grouped.items():
            link_values = [
                float(sample.link_quality)
                for sample in samples
                if sample.updated_metric == "link_quality"
                and sample.link_quality is not None
            ]
            congestion_values = [
                float(value)
                for sample in samples
                for value in (
                    [sample.uplink_congestion]
                    if sample.updated_metric == "uplink_congestion"
                    else [sample.downlink_congestion]
                    if sample.updated_metric == "downlink_congestion"
                    else []
                )
                if value is not None
            ]
            latency_values = [
                float(sample.latency_ms)
                for sample in samples
                if sample.updated_metric == "latency_ms"
                and sample.latency_ms is not None
            ]
            min_link = min(link_values) if link_values else None
            max_congestion = max(congestion_values) if congestion_values else 0.0
            latency_p95 = self._percentile(latency_values, 0.95)
            score = 100.0 if min_link is None else float(min_link)
            if congestion_soft_floor > 0 and max_congestion > congestion_soft_floor:
                score = min(score, max(0.0, 100.0 - max_congestion))
            if latency_p95 is not None and latency_p95 > latency_p95_soft_limit_ms:
                score = min(
                    score,
                    max(0.0, 100.0 * latency_p95_soft_limit_ms / latency_p95),
                )
            summary[group_id] = {
                "score": round(float(score), 6),
                "min_link_quality": min_link,
                "mean_link_quality": mean(link_values) if link_values else None,
                "p5_link_quality": self._percentile(link_values, 0.05),
                "max_congestion": max_congestion,
                "latency_p95_ms": latency_p95,
                "drone_count": len(
                    {
                        drone_id
                        for drone_id in self._latest
                        if fleet.get_radio_group(drone_id) == group_id
                    }
                ),
            }
        return summary
