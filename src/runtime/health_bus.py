"""Runtime health state store"""

import threading
from dataclasses import dataclass


@dataclass
class HealthSample:
    t_meas: float
    values: dict


class HealthBus:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: dict[int, HealthSample] = {}

    def update(self, drone_id: int, values: dict, t_meas: float):
        with self._lock:
            self._latest[drone_id] = HealthSample(t_meas=t_meas, values=values)

    def latest(self) -> dict[int, HealthSample]:
        with self._lock:
            return dict(self._latest)
