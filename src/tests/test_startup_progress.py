"""Unit tests for startup progress reporters."""

import logging
import pytest

from src.app.startup_progress import (
    NullProgressReporter,
    StartupProgressReporter,
    TextProgressReporter,
)


def test_null_reporter_is_silent(caplog):
    reporter = NullProgressReporter()
    with caplog.at_level(logging.DEBUG):
        with reporter.phase("connect", "Connecting"):
            reporter.step(3, 10, "drone=3")
            reporter.warn("low battery")
        reporter.close()
    assert caplog.records == []


def test_null_reporter_conforms_to_protocol():
    assert isinstance(NullProgressReporter(), StartupProgressReporter)


def test_text_reporter_logs_phase_ok(caplog):
    reporter = TextProgressReporter(verbose=False, total_phases=9)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with reporter.phase("connect", "连接 Crazyflie"):
            pass
    messages = [r.getMessage() for r in caplog.records]
    assert any("[1/9] 连接 Crazyflie ..." in m for m in messages)
    assert any("[1/9] 连接 Crazyflie OK" in m for m in messages)


def test_text_reporter_logs_phase_fail(caplog):
    reporter = TextProgressReporter(verbose=False, total_phases=9)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with pytest.raises(RuntimeError):
            with reporter.phase("connect", "连接"):
                raise RuntimeError("boom")
    messages = [r.getMessage() for r in caplog.records]
    assert any("FAIL" in m and "boom" in m for m in messages)


def test_text_reporter_step_hidden_by_default(caplog):
    reporter = TextProgressReporter(verbose=False, total_phases=9)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with reporter.phase("wait_for_params", "等参数"):
            reporter.step(3, 10, "drone=3")
    assert not any("drone=3" in r.getMessage() for r in caplog.records)


def test_text_reporter_step_visible_when_verbose(caplog):
    reporter = TextProgressReporter(verbose=True, total_phases=9)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with reporter.phase("wait_for_params", "等参数"):
            reporter.step(3, 10, "drone=3")
    assert any("3/10" in r.getMessage() and "drone=3" in r.getMessage()
               for r in caplog.records)


def test_text_reporter_phase_index_increments(caplog):
    reporter = TextProgressReporter(verbose=False, total_phases=9)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with reporter.phase("connect", "a"):
            pass
        with reporter.phase("wait_for_params", "b"):
            pass
    messages = [r.getMessage() for r in caplog.records]
    assert any("[1/9]" in m for m in messages)
    assert any("[2/9]" in m for m in messages)


def test_text_reporter_set_total_phases(caplog):
    reporter = TextProgressReporter(verbose=False)
    reporter.set_total_phases(7)
    with caplog.at_level(logging.INFO, logger="src.app.startup_progress"):
        with reporter.phase("connect", "a"):
            pass
    assert any("[1/7]" in r.getMessage() for r in caplog.records)


def test_text_reporter_conforms_to_protocol():
    assert isinstance(TextProgressReporter(), StartupProgressReporter)
