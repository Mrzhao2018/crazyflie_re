"""reset_estimator 阶段应把 kalman variance 注入 progress detail。"""

from __future__ import annotations

from contextlib import contextmanager

from src.app.run_real import RealMissionApp
from src.runtime.safety_manager import SafetyDecision
from src.tests.run_real_fixtures import build_components, make_snapshot


class _CaptureProgress:
    def __init__(self) -> None:
        self.steps: list[tuple[int, int, str | None]] = []
        self.total_phases: int | None = None
        self.closed = False

    @contextmanager
    def phase(self, key: str, title: str, total: int | None = None):
        yield self

    def step(self, done: int, total: int, detail: str | None = None) -> None:
        self.steps.append((done, total, detail))

    def warn(self, msg: str) -> None:
        pass

    def set_total_phases(self, total: int) -> None:
        self.total_phases = total

    def close(self) -> None:
        self.closed = True


def test_reset_estimator_progress_includes_variance_detail():
    components = build_components(
        [make_snapshot(1), make_snapshot(2)],
        [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
    )

    sample_type = type(
        "Sample",
        (),
        {
            "t_meas": 0.0,
            "values": {
                "pm.vbat": 4.0,
                "kalman.varPX": 0.0009,
                "kalman.varPY": 0.0004,
                "kalman.varPZ": 0.0007,
            },
        },
    )
    components["health_bus"] = type(
        "HealthBusWithVariance",
        (),
        {
            "latest": lambda self: {
                drone_id: sample_type() for drone_id in components["fleet"].all_ids()
            }
        },
    )()

    progress = _CaptureProgress()
    app = RealMissionApp(components, progress=progress)

    assert app.start() is True
    reset_steps = [
        detail
        for _done, _total, detail in progress.steps
        if detail is not None and detail.startswith("drone=")
    ]
    assert any("var_max=0.0009" in detail for detail in reset_steps)
    assert progress.total_phases is not None
    assert progress.closed is True
