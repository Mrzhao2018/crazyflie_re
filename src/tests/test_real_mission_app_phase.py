"""Unit tests for RealMissionApp._phase helper.

The helper wraps reporter.phase AND emits telemetry 'startup_phase' events.
"""

from unittest.mock import MagicMock

import pytest

from src.app.run_real import RealMissionApp
from src.app.startup_progress import NullProgressReporter, TextProgressReporter

def _bare_app(telemetry, progress):
    """Bypass __init__ because we only exercise _phase here."""
    app = RealMissionApp.__new__(RealMissionApp)
    app.telemetry = telemetry
    app._progress = progress
    return app


def test_phase_records_begin_and_ok_events():
    telemetry = MagicMock()
    app = _bare_app(telemetry, NullProgressReporter())

    with app._phase("connect", "连接"):
        pass

    calls = telemetry.record_event.call_args_list
    assert len(calls) == 2
    assert calls[0].args == ("startup_phase",)
    assert calls[0].kwargs["phase"] == "connect"
    assert calls[0].kwargs["status"] == "begin"
    assert calls[1].kwargs["phase"] == "connect"
    assert calls[1].kwargs["status"] == "ok"
    assert "duration_s" in calls[1].kwargs


def test_phase_records_fail_on_exception():
    telemetry = MagicMock()
    app = _bare_app(telemetry, NullProgressReporter())

    with pytest.raises(RuntimeError, match="boom"):
        with app._phase("connect", "连接"):
            raise RuntimeError("boom")

    calls = telemetry.record_event.call_args_list
    statuses = [c.kwargs["status"] for c in calls]
    assert statuses == ["begin", "fail"]
    assert "boom" in calls[1].kwargs["detail"]
    assert "duration_s" in calls[1].kwargs


def test_phase_drives_reporter_as_well():
    telemetry = MagicMock()
    reporter = TextProgressReporter(total_phases=9)
    app = _bare_app(telemetry, reporter)

    with app._phase("connect", "连接"):
        pass

    assert reporter._phase_index == 1


def test_phase_works_without_telemetry():
    app = _bare_app(telemetry=None, progress=NullProgressReporter())
    with app._phase("connect", "连接"):
        pass


def test_fail_start_raises_startup_aborted():
    """_fail_start must raise _StartupAborted so that _phase can mark the
    phase FAIL. start() catches the sentinel at the top level and returns False.
    """
    from src.app.mission_errors import MissionErrors
    from src.app.run_real import _StartupAborted

    telemetry = MagicMock()
    fsm = MagicMock()
    app = _bare_app(telemetry, NullProgressReporter())
    app.comp = {"fsm": fsm, "telemetry": telemetry}
    app.fsm = fsm
    app.telemetry_reporter = MagicMock()
    app._running = False
    app._shutdown_flushed = True
    app._terminal_land_executed = True
    app.landing_flow = MagicMock()

    def _noop_shutdown():
        pass

    app.shutdown = _noop_shutdown  # type: ignore[assignment]

    with pytest.raises(_StartupAborted):
        app._fail_start(
            "boom",
            definition=MissionErrors.Readiness.STARTUP_FAILED,
        )

    fsm.force_abort.assert_called_once()
