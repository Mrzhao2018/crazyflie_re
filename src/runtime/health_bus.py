"""Runtime health state store"""

import threading
from dataclasses import dataclass


@dataclass
class HealthSample:
    t_meas: float
    values: dict
    safety_t_meas: float | None = None


class HealthBus:
    _SAFETY_HEALTH_KEYS = frozenset(
        {
            "pm.vbat",
            "kalman.varPX",
            "kalman.varPY",
            "kalman.varPZ",
            "motor.m1",
            "motor.m2",
            "motor.m3",
            "motor.m4",
        }
    )

    def __init__(self):
        self._lock = threading.Lock()
        self._latest: dict[int, HealthSample] = {}
        self._history: dict[int, list[HealthSample]] = {}
        self._history_window_s = 10.0

    def update(self, drone_id: int, values: dict, t_meas: float):
        """Merge 语义：多个 log block（pm.vbat / kalman.var* / motor.* / attitude）
        分别按各自频率推入，values 字段相互叠加，不要覆盖其它 block 的字段。
        ``t_meas`` 仍表示最近一次任意 health/attitude 写入；``safety_t_meas`` 只
        在安全相关 health block 更新时推进，供 preflight freshness 判断使用。"""
        with self._lock:
            existing = self._latest.get(drone_id)
            merged = dict(existing.values) if existing is not None else {}
            merged.update(values)
            safety_t_meas = (
                existing.safety_t_meas if existing is not None else None
            )
            if any(key in self._SAFETY_HEALTH_KEYS for key in values):
                safety_t_meas = t_meas
            self._latest[drone_id] = HealthSample(
                t_meas=t_meas,
                values=merged,
                safety_t_meas=safety_t_meas,
            )
            sample = self._latest[drone_id]
            history = self._history.setdefault(drone_id, [])
            history.append(
                HealthSample(
                    t_meas=sample.t_meas,
                    values=dict(sample.values),
                    safety_t_meas=sample.safety_t_meas,
                )
            )
            cutoff = t_meas - self._history_window_s
            self._history[drone_id] = [item for item in history if item.t_meas >= cutoff]

    def latest(self) -> dict[int, HealthSample]:
        with self._lock:
            return dict(self._latest)

    def recent_samples(self, window_s: float) -> dict[int, list[HealthSample]]:
        with self._lock:
            latest_t = max((sample.t_meas for sample in self._latest.values()), default=None)
            if latest_t is None:
                return {}
            cutoff = latest_t - max(0.0, float(window_s))
            return {
                drone_id: [
                    HealthSample(
                        t_meas=sample.t_meas,
                        values=dict(sample.values),
                        safety_t_meas=sample.safety_t_meas,
                    )
                    for sample in samples
                    if sample.t_meas >= cutoff
                ]
                for drone_id, samples in self._history.items()
            }
