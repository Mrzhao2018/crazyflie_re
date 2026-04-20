# Startup Progress UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the real-mission `run` entrypoint visible, two-tier startup progress feedback via a rich TUI (with automatic fallback to scrolling text), and fix the root logger initialization gap.

**Architecture:** Introduce a `StartupProgressReporter` protocol with two concrete implementations (`RichProgressReporter` and `TextProgressReporter`) and a `make_reporter` factory that applies the fallback decision tree. A `configure_logging` entry point wires the root logger to the same console as the reporter. `RealMissionApp` gains a `_phase` context manager that drives both the reporter and `telemetry.record_event("startup_phase", ...)` for every startup phase.

**Tech Stack:** Python 3.11+, `rich >= 13` (optional), `logging` stdlib, `pytest`. Project already uses `unittest.mock` in contract tests.

**Reference spec:** `specs/2026-04-20-startup-progress-ui-design.md`

---

## File Structure

New files:
- `src/app/startup_progress.py` — protocol, NullProgressReporter, TextProgressReporter, RichProgressReporter, make_reporter
- `src/app/log_setup.py` — configure_logging(verbose, reporter)
- `src/tests/test_startup_progress.py` — unit tests for all reporter classes + factory
- `src/tests/test_log_setup.py` — unit tests for configure_logging
- `src/tests/test_real_mission_app_phase.py` — unit tests for RealMissionApp._phase

Modified files:
- `src/app/run_real.py` — constructor accepts progress; add `_phase` helper + `_StartupAborted` sentinel; refactor `_fail_start` to raise; wrap `start()` phases with `with self._phase(...)`
- `src/app/cli.py` — `_build_run_parent` adds `--verbose/-v`; `_run_real` builds reporter, calls `configure_logging`, passes reporter to `RealMissionApp`
- `README.md` — add `rich` to optional dependencies

---

## Task 1: Reporter Protocol + Null/Text implementations

**Files:**
- Create: `src/app/startup_progress.py`
- Create: `src/tests/test_startup_progress.py`

- [ ] **Step 1: Write failing tests**

Create `src/tests/test_startup_progress.py`:

```python
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
```

- [ ] **Step 2: Run test and verify failure**

```bash
python -m pytest src/tests/test_startup_progress.py -v
```

Expected: `ModuleNotFoundError: No module named 'src.app.startup_progress'`.

- [ ] **Step 3: Create startup_progress.py with protocol + Null + Text**

Create `src/app/startup_progress.py`:

```python
"""Startup progress reporters: protocol, null impl, text impl.

Rich TUI impl and factory live in the same module but added in later tasks.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import Any, Iterator, Protocol, runtime_checkable

logger = logging.getLogger(__name__)


@runtime_checkable
class StartupProgressReporter(Protocol):
    """Displays startup phase progress. Telemetry events are NOT emitted here."""

    console: Any

    @contextmanager
    def phase(
        self, key: str, title: str, total: int | None = None
    ) -> Iterator["StartupProgressReporter"]:
        ...

    def step(self, done: int, total: int, detail: str | None = None) -> None:
        ...

    def warn(self, msg: str) -> None:
        ...

    def set_total_phases(self, total: int) -> None:
        ...

    def close(self) -> None:
        ...


class NullProgressReporter:
    """No-op reporter; default when no progress UI is wired (e.g. unit tests)."""

    console: Any = None

    @contextmanager
    def phase(self, key, title, total=None):
        yield self

    def step(self, done, total, detail=None):
        pass

    def warn(self, msg):
        pass

    def set_total_phases(self, total):
        pass

    def close(self):
        pass


class TextProgressReporter:
    """Scrolling-text reporter. Used as fallback when Rich is unavailable."""

    console: Any = None

    def __init__(self, verbose: bool = False, total_phases: int = 9):
        self._verbose = verbose
        self._total_phases = total_phases
        self._phase_index = 0

    def set_total_phases(self, total: int) -> None:
        self._total_phases = total

    @contextmanager
    def phase(self, key, title, total=None):
        self._phase_index += 1
        prefix = f"[{self._phase_index}/{self._total_phases}]"
        started_at = time.monotonic()
        logger.info("%s %s ...", prefix, title)
        try:
            yield self
        except Exception as exc:
            duration = time.monotonic() - started_at
            logger.error("%s %s FAIL (%.1fs) — %s", prefix, title, duration, exc)
            raise
        else:
            duration = time.monotonic() - started_at
            logger.info("%s %s OK (%.1fs)", prefix, title, duration)

    def step(self, done, total, detail=None):
        if not self._verbose:
            return
        suffix = f" {detail}" if detail else ""
        logger.info("       %d/%d%s", done, total, suffix)

    def warn(self, msg):
        logger.warning("       WARN %s", msg)

    def close(self):
        pass
```

- [ ] **Step 4: Run tests and verify pass**

```bash
python -m pytest src/tests/test_startup_progress.py -v
```

Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add src/app/startup_progress.py src/tests/test_startup_progress.py
git commit -m "feat(app): add StartupProgressReporter protocol with Null/Text impls"
```

---

## Task 2: RichProgressReporter

**Files:**
- Modify: `src/app/startup_progress.py` (append class)
- Modify: `src/tests/test_startup_progress.py` (append tests)

- [ ] **Step 1: Write failing tests**

Append to `src/tests/test_startup_progress.py`:

```python
def test_rich_reporter_renders_phase_rows():
    pytest.importorskip("rich")
    from io import StringIO
    from rich.console import Console

    from src.app.startup_progress import RichProgressReporter

    buf = StringIO()
    console = Console(
        file=buf, force_terminal=True, width=120, record=True, color_system=None
    )
    reporter = RichProgressReporter(verbose=False, total_phases=9, console=console)
    try:
        with reporter.phase("connect", "连接 Crazyflie"):
            reporter.step(5, 10)
        with pytest.raises(RuntimeError):
            with reporter.phase("wait_for_params", "等参数"):
                raise RuntimeError("boom")
    finally:
        reporter.close()

    output = console.export_text(clear=False)
    assert "连接 Crazyflie" in output
    assert "OK" in output
    assert "FAIL" in output
    assert "boom" in output


def test_rich_reporter_close_is_idempotent():
    pytest.importorskip("rich")
    from rich.console import Console

    from src.app.startup_progress import RichProgressReporter

    reporter = RichProgressReporter(total_phases=9, console=Console(quiet=True))
    reporter.close()
    reporter.close()


def test_rich_reporter_conforms_to_protocol():
    pytest.importorskip("rich")
    from rich.console import Console

    from src.app.startup_progress import (
        RichProgressReporter,
        StartupProgressReporter,
    )

    reporter = RichProgressReporter(total_phases=9, console=Console(quiet=True))
    try:
        assert isinstance(reporter, StartupProgressReporter)
    finally:
        reporter.close()
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python -m pytest src/tests/test_startup_progress.py -v -k rich
```

Expected: `ImportError: cannot import name 'RichProgressReporter'` (or similar).

- [ ] **Step 3: Append RichProgressReporter to startup_progress.py**

Append at the end of `src/app/startup_progress.py`:

```python
class RichProgressReporter:
    """Rich Live + Table TUI reporter. Only constructed when rich is importable."""

    def __init__(
        self,
        verbose: bool = False,
        total_phases: int = 9,
        console: Any = None,
    ):
        from rich.console import Console

        self._verbose = verbose
        self._total_phases = total_phases
        self._phase_index = 0
        self._rows: list[dict[str, Any]] = []
        self._active_row: dict[str, Any] | None = None
        self.console: Any = console if console is not None else Console()
        self._live: Any = None

    def set_total_phases(self, total: int) -> None:
        self._total_phases = total

    def _ensure_live(self) -> None:
        if self._live is None:
            from rich.live import Live

            self._live = Live(
                self._render(),
                console=self.console,
                refresh_per_second=8,
                transient=False,
            )
            self._live.start()

    def _render(self):
        from rich.table import Table

        table = Table.grid(padding=(0, 1))
        table.add_column()
        table.add_column()
        table.add_column()
        for row in self._rows:
            status = row["status"]
            if status == "begin":
                if row.get("total"):
                    marker = f"{row.get('done', 0)}/{row['total']}"
                else:
                    marker = "..."
            elif status == "ok":
                marker = f"[green]OK[/] ({row['duration']:.1f}s)"
            elif status == "fail":
                marker = f"[red]FAIL[/] {row.get('reason', '')}"
            else:
                marker = status
            prefix = f"[{row['index']}/{self._total_phases}]"
            detail = row.get("detail") or ""
            table.add_row(prefix, row["title"], f"{marker}  {detail}".rstrip())
        return table

    @contextmanager
    def phase(self, key, title, total=None):
        self._phase_index += 1
        started_at = time.monotonic()
        row: dict[str, Any] = {
            "index": self._phase_index,
            "key": key,
            "title": title,
            "status": "begin",
            "total": total,
            "done": 0,
            "detail": None,
        }
        self._rows.append(row)
        self._active_row = row
        self._ensure_live()
        self._live.update(self._render())
        try:
            yield self
        except Exception as exc:
            row["status"] = "fail"
            row["duration"] = time.monotonic() - started_at
            row["reason"] = str(exc)[:120]
            self._live.update(self._render())
            raise
        else:
            row["status"] = "ok"
            row["duration"] = time.monotonic() - started_at
            self._live.update(self._render())
        finally:
            self._active_row = None

    def step(self, done, total, detail=None):
        if self._active_row is None:
            return
        self._active_row["done"] = done
        self._active_row["total"] = total
        if self._verbose and detail:
            self._active_row["detail"] = detail
        if self._live is not None:
            self._live.update(self._render())

    def warn(self, msg):
        if self._active_row is not None:
            self._active_row["detail"] = f"WARN {msg}"
            if self._live is not None:
                self._live.update(self._render())

    def close(self):
        if self._live is not None:
            self._live.stop()
            self._live = None
```

- [ ] **Step 4: Run tests and verify pass**

```bash
python -m pytest src/tests/test_startup_progress.py -v
```

Expected: 11 passed (8 from Task 1 + 3 from Task 2).

- [ ] **Step 5: Commit**

```bash
git add src/app/startup_progress.py src/tests/test_startup_progress.py
git commit -m "feat(app): add RichProgressReporter (rich Live + Table)"
```

---

## Task 3: `make_reporter` factory with fallback

**Files:**
- Modify: `src/app/startup_progress.py` (append factory)
- Modify: `src/tests/test_startup_progress.py` (append tests)

- [ ] **Step 1: Write failing tests**

Append to `src/tests/test_startup_progress.py`:

```python
def test_make_reporter_no_rich_env(monkeypatch):
    monkeypatch.setenv("AFC_NO_RICH", "1")
    from src.app.startup_progress import TextProgressReporter, make_reporter

    assert isinstance(make_reporter(), TextProgressReporter)


def test_make_reporter_non_tty(monkeypatch):
    monkeypatch.delenv("AFC_NO_RICH", raising=False)

    class FakeStream:
        def isatty(self):
            return False

    from src.app.startup_progress import TextProgressReporter, make_reporter

    reporter = make_reporter(stdout=FakeStream())
    assert isinstance(reporter, TextProgressReporter)


def test_make_reporter_rich_missing(monkeypatch):
    monkeypatch.delenv("AFC_NO_RICH", raising=False)
    import builtins

    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "rich" or name.startswith("rich."):
            raise ImportError("simulated missing rich")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)

    class FakeStream:
        def isatty(self):
            return True

    from src.app.startup_progress import TextProgressReporter, make_reporter

    reporter = make_reporter(stdout=FakeStream())
    assert isinstance(reporter, TextProgressReporter)


def test_make_reporter_rich_tty(monkeypatch):
    pytest.importorskip("rich")
    monkeypatch.delenv("AFC_NO_RICH", raising=False)

    class FakeStream:
        def isatty(self):
            return True

    from src.app.startup_progress import RichProgressReporter, make_reporter

    reporter = make_reporter(stdout=FakeStream())
    try:
        assert isinstance(reporter, RichProgressReporter)
    finally:
        reporter.close()
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python -m pytest src/tests/test_startup_progress.py -v -k make_reporter
```

Expected: `ImportError: cannot import name 'make_reporter'`.

- [ ] **Step 3: Append factory to startup_progress.py**

Append at the end of `src/app/startup_progress.py`:

```python
def make_reporter(
    verbose: bool = False,
    total_phases: int = 9,
    stdout: Any = None,
) -> StartupProgressReporter:
    """Pick a reporter by the fallback decision tree.

    Order: AFC_NO_RICH env -> rich import -> stdout.isatty() -> Rich.
    """
    import os
    import sys

    stream = stdout if stdout is not None else sys.stdout

    if os.environ.get("AFC_NO_RICH") == "1":
        return TextProgressReporter(verbose=verbose, total_phases=total_phases)

    try:
        import rich  # noqa: F401
    except ImportError:
        logger.info(
            "rich 未安装，降级滚动文本（可 `pip install rich` 启用 TUI）"
        )
        return TextProgressReporter(verbose=verbose, total_phases=total_phases)

    is_tty = getattr(stream, "isatty", lambda: False)()
    if not is_tty:
        return TextProgressReporter(verbose=verbose, total_phases=total_phases)

    return RichProgressReporter(verbose=verbose, total_phases=total_phases)
```

- [ ] **Step 4: Run tests and verify pass**

```bash
python -m pytest src/tests/test_startup_progress.py -v
```

Expected: 15 passed (11 from Task 1+2 + 4 from Task 3).

- [ ] **Step 5: Commit**

```bash
git add src/app/startup_progress.py src/tests/test_startup_progress.py
git commit -m "feat(app): add make_reporter factory with AFC_NO_RICH/tty fallback"
```

---

## Task 4: `configure_logging` entry point

**Files:**
- Create: `src/app/log_setup.py`
- Create: `src/tests/test_log_setup.py`

- [ ] **Step 1: Write failing tests**

Create `src/tests/test_log_setup.py`:

```python
"""Unit tests for configure_logging."""

import logging

from src.app.log_setup import configure_logging
from src.app.startup_progress import NullProgressReporter, TextProgressReporter


def test_configure_logging_default_is_info():
    configure_logging(verbose=False, reporter=TextProgressReporter())
    assert logging.getLogger().level == logging.INFO


def test_configure_logging_verbose_is_debug():
    configure_logging(verbose=True, reporter=TextProgressReporter())
    assert logging.getLogger().level == logging.DEBUG


def test_configure_logging_replaces_existing_handlers():
    root = logging.getLogger()
    dummy = logging.Handler()
    root.addHandler(dummy)
    configure_logging(verbose=False, reporter=TextProgressReporter())
    assert dummy not in root.handlers


def test_configure_logging_text_reporter_uses_streamhandler():
    configure_logging(verbose=False, reporter=TextProgressReporter())
    root = logging.getLogger()
    assert len(root.handlers) == 1
    handler = root.handlers[0]
    assert isinstance(handler, logging.StreamHandler)


def test_configure_logging_null_reporter_falls_back_to_stream():
    configure_logging(verbose=False, reporter=NullProgressReporter())
    root = logging.getLogger()
    assert len(root.handlers) == 1
    assert isinstance(root.handlers[0], logging.StreamHandler)
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python -m pytest src/tests/test_log_setup.py -v
```

Expected: `ModuleNotFoundError: No module named 'src.app.log_setup'`.

- [ ] **Step 3: Create log_setup.py**

Create `src/app/log_setup.py`:

```python
"""Root logger initialization. Must be called once from the CLI entrypoint.

Shares a single rich.console.Console with the startup progress reporter when
the reporter exposes one, so RichHandler output does not tear the Live panel.
"""

from __future__ import annotations

import logging
import sys
from typing import Any


_TEXT_FORMAT = "[%(asctime)s] %(message)s"
_TEXT_DATEFMT = "%H:%M:%S"


def configure_logging(verbose: bool, reporter: Any) -> None:
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)

    level = logging.DEBUG if verbose else logging.INFO
    root.setLevel(level)

    handler: logging.Handler
    console = getattr(reporter, "console", None)
    if console is not None:
        try:
            from rich.logging import RichHandler

            handler = RichHandler(
                console=console,
                rich_tracebacks=True,
                show_path=False,
                markup=False,
            )
            handler.setFormatter(logging.Formatter("%(message)s"))
        except ImportError:
            handler = logging.StreamHandler(sys.stdout)
            handler.setFormatter(
                logging.Formatter(_TEXT_FORMAT, datefmt=_TEXT_DATEFMT)
            )
    else:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(_TEXT_FORMAT, datefmt=_TEXT_DATEFMT))

    handler.setLevel(level)
    root.addHandler(handler)
```

- [ ] **Step 4: Run tests and verify pass**

```bash
python -m pytest src/tests/test_log_setup.py -v
```

Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add src/app/log_setup.py src/tests/test_log_setup.py
git commit -m "feat(app): add configure_logging that shares console with reporter"
```

---

## Task 5: `RealMissionApp._phase` helper (reporter + telemetry)

**Files:**
- Modify: `src/app/run_real.py` (imports, `__init__`, add `_phase`; no change to `start()` yet)
- Create: `src/tests/test_real_mission_app_phase.py`

- [ ] **Step 1: Write failing tests**

Create `src/tests/test_real_mission_app_phase.py`:

```python
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
```

- [ ] **Step 2: Run tests and verify failure**

```bash
python -m pytest src/tests/test_real_mission_app_phase.py -v
```

Expected: `AttributeError: ... has no attribute '_phase'`.

- [ ] **Step 3: Modify run_real.py**

In `src/app/run_real.py` add imports near the top (after the existing `import logging` line):

```python
from contextlib import contextmanager

from .startup_progress import NullProgressReporter, StartupProgressReporter
```

Define the sentinel (place right before the `class RealMissionApp:` line):

```python
class _StartupAborted(Exception):
    """Internal sentinel: _fail_start was invoked; start() should return False."""
```

Modify the existing `__init__` signature and body. Replace the current `def __init__(self, components: dict):` block with:

```python
    def __init__(
        self,
        components: dict,
        progress: StartupProgressReporter | None = None,
    ):
        self.comp = components
        self.telemetry = components.get("telemetry")
        self.fleet = components.get("fleet")
        self.scheduler = components.get("scheduler")
        self.transport = components.get("transport")
        self.fsm = components.get("fsm")
        self.telemetry_reporter = MissionTelemetryReporter(
            self.telemetry, self.fleet
        )
        self.failure_policy = FailurePolicy(self)
        self.landing_flow = LandingFlow(self)
        self._running = False
        self._last_processed_seq = -1
        self._trajectory_started = False
        self._shutdown_flushed = False
        self._terminal_land_executed = False
        self._trajectory_state = "inactive"
        self._trajectory_terminal_reason = None
        self._readiness_report = {
            "wait_for_params": {},
            "reset_estimator": {},
            "trajectory_prepare": {},
            "pose_ready": False,
        }
        self._config_fingerprint = self._build_config_fingerprint()
        self._progress: StartupProgressReporter = progress or NullProgressReporter()
```

Add the `_phase` method inside `RealMissionApp` (place it immediately after `__init__`):

```python
    @contextmanager
    def _phase(self, key: str, title: str, total: int | None = None):
        telemetry = self.telemetry
        if telemetry is not None:
            telemetry.record_event("startup_phase", phase=key, status="begin")
        start = time.monotonic()
        try:
            with self._progress.phase(key, title, total):
                yield
        except Exception as exc:
            if telemetry is not None:
                telemetry.record_event(
                    "startup_phase",
                    phase=key,
                    status="fail",
                    duration_s=time.monotonic() - start,
                    detail=str(exc)[:200],
                )
            raise
        else:
            if telemetry is not None:
                telemetry.record_event(
                    "startup_phase",
                    phase=key,
                    status="ok",
                    duration_s=time.monotonic() - start,
                )
```

- [ ] **Step 4: Run tests and verify pass**

```bash
python -m pytest src/tests/test_real_mission_app_phase.py -v
```

Expected: 4 passed.

- [ ] **Step 5: Run existing run_real tests to confirm no regression**

```bash
python -m pytest src/tests/test_run_real.py -v
```

Expected: all green (existing behavior unchanged; `__init__`'s new parameter defaults to None).

- [ ] **Step 6: Commit**

```bash
git add src/app/run_real.py src/tests/test_real_mission_app_phase.py
git commit -m "feat(run_real): add _phase helper + progress reporter constructor arg"
```

---

## Task 6: Refactor `_fail_start` to raise `_StartupAborted`

**Files:**
- Modify: `src/app/run_real.py`

Background: `_fail_start` currently returns `False`, which callers `return` from `start()`. For phase failures to surface correctly through `with self._phase(...):` the helper must raise. We convert `_fail_start` to raise `_StartupAborted` (defined in Task 5), and catch it at the top of `start()`.

**This task does NOT yet wrap phases with `_phase`. It only changes the failure plumbing so tests still pass and behavior is preserved.**

- [ ] **Step 1: Write regression test for start() failure path**

Append to `src/tests/test_real_mission_app_phase.py`:

```python
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
    app.telemetry_reporter = MagicMock()
    app._running = False
    app._shutdown_flushed = True  # skip landing_flow paths
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
```

- [ ] **Step 2: Run test and verify failure**

```bash
python -m pytest src/tests/test_real_mission_app_phase.py::test_fail_start_raises_startup_aborted -v
```

Expected: fail — `_fail_start` currently returns `False` instead of raising.

- [ ] **Step 3: Modify `_fail_start`**

In `src/app/run_real.py`, replace the body of `_fail_start`. Current body ends with `return False`; change the final two statements from:

```python
        self.comp["fsm"].force_abort()
        self.shutdown()
        return False
```

to:

```python
        self.comp["fsm"].force_abort()
        self.shutdown()
        raise _StartupAborted(reason)
```

Update the return type annotation of `_fail_start` from `-> bool` to `-> None` (only the annotation; the `def` line stays otherwise the same).

- [ ] **Step 4: Wrap `start()` body with top-level try/except**

In `src/app/run_real.py`, modify `start(self):`. Replace the outer control flow so that the existing body is inside a try/except. Show the structure:

```python
    def start(self):
        """启动"""
        logger.info("=== 启动真机任务 ===")
        try:
            return self._start_impl()
        except _StartupAborted:
            return False
```

Rename the existing body of `start()` from after `logger.info("=== 启动真机任务 ===")` down to the final `return True` (currently at line 605) into a new method `_start_impl(self):`. Do NOT move the initial `logger.info("=== 启动真机任务 ===")` line.

Also replace every `return self._fail_start(...)` in the now-renamed `_start_impl` with a bare call (drop the `return`, since `_fail_start` raises):

```python
        self._fail_start(
            "连接失败",
            definition=MissionErrors.Connection.CONNECT_ALL_FAILED,
            exception=exc,
            connect_outcome=...,
            ...
        )
```

There are 10 such call sites in the current `start()`: lines 168, 186, 222, 239, 290, 309, 336, 355, 429, 459, 465, 473, 484, 493, 511. Grep to double-check:

```bash
grep -n "return self._fail_start" src/app/run_real.py
```

Replace each `return self._fail_start(...)` with `self._fail_start(...)`. Leave the argument list untouched.

- [ ] **Step 5: Run the new regression test and existing suite**

```bash
python -m pytest src/tests/test_real_mission_app_phase.py src/tests/test_run_real.py -v
```

Expected: all green. The previous assertion that `start()` returns `False` on failure is preserved because `_start_impl` raises `_StartupAborted`, which `start()` catches and returns `False`.

- [ ] **Step 6: Commit**

```bash
git add src/app/run_real.py src/tests/test_real_mission_app_phase.py
git commit -m "refactor(run_real): _fail_start raises _StartupAborted; start() catches"
```

---

## Task 7: Wrap `start()` phases with `_phase`

**Files:**
- Modify: `src/app/run_real.py` (`_start_impl` only)

This task replaces each startup segment with a `with self._phase(...)` block and adds `self._progress.step(...)` calls inside loops. We also compute the runtime `total_phases` and call `self._progress.set_total_phases(total)` before phase 1.

- [ ] **Step 1: Compute dynamic total_phases at the top of `_start_impl`**

Insert at the very top of `_start_impl`, before the existing `timestamp = datetime.now().strftime(...)` line:

```python
        comm = self.comp["config"].comm
        startup_mode = self.comp.get("startup_mode", "auto")
        leader_ref_gen = self.comp["leader_ref_gen"]
        trajectory_enabled = (
            startup_mode == "auto"
            and leader_ref_gen.reference_at(0.0).mode == "trajectory"
            and leader_ref_gen.reference_at(0.0).trajectory is not None
        )
        total_phases = 6  # connect + onboard_controller + pose + health + preflight + takeoff
        if comm.readiness_wait_for_params:
            total_phases += 1
        if comm.readiness_reset_estimator:
            total_phases += 1
        if trajectory_enabled:
            total_phases += 1
        self._progress.set_total_phases(total_phases)
```

- [ ] **Step 2: Wrap phase 1 — connect**

Replace the current connect block (between `# 连接` comment and the end of the `except` block around line 195) with:

```python
        # 连接 (phase 1)
        if not self._safe_transition(MissionState.CONNECT):
            self._fail_start(
                "FSM failed before connect",
                definition=MissionErrors.Readiness.FSM_CONNECT_TRANSITION_FAILED,
            )

        radio_group_count = len(
            {
                self.comp["fleet"].get_radio_group(d)
                for d in self.comp["fleet"].all_ids()
            }
        )
        progress_state = {"done": 0}

        def _on_group_start(group_id, drone_ids):
            self.telemetry_reporter.record_connect_group_start(group_id, drone_ids)

        def _on_group_result(group_id, drone_ids, result):
            self.telemetry_reporter.record_connect_group_result(
                group_id, drone_ids, result
            )
            progress_state["done"] += 1
            self._progress.step(
                progress_state["done"],
                radio_group_count,
                detail=f"group={group_id}",
            )

        connect_report: dict[str, object] = {}
        with self._phase("connect", "连接 Crazyflie", total=radio_group_count):
            try:
                connect_report = self.comp["link_manager"].connect_all(
                    on_group_start=_on_group_start,
                    on_group_result=_on_group_result,
                    parallel_groups=comm.connect_groups_in_parallel,
                )
                self.telemetry_reporter.record_connect_all(
                    ok=True, report=connect_report
                )
            except Exception as exc:
                connect_report = self._last_connect_report()
                self.telemetry_reporter.record_connect_all(
                    ok=False, report=connect_report, error=str(exc)
                )
                self._fail_start(
                    "连接失败",
                    definition=MissionErrors.Connection.CONNECT_ALL_FAILED,
                    exception=exc,
                    connect_outcome=self.telemetry_reporter.connect_all_outcome(
                        connect_report, False
                    ),
                    failed_group_ids=self.telemetry_reporter.failed_connect_group_ids(
                        connect_report
                    ),
                    connected=connect_report.get("connected", []),
                    failures=connect_report.get("failures", []),
                    radio_groups=connect_report.get("radio_groups", {}),
                )
```

- [ ] **Step 3: Wrap phase 2 — wait_for_params**

Replace the current `if self.comp["config"].comm.readiness_wait_for_params:` block with:

```python
        if comm.readiness_wait_for_params:
            total = len(self.comp["fleet"].all_ids())
            with self._phase("wait_for_params", "等待参数同步", total=total):
                done = 0
                drone_id = None
                try:
                    group_pool = self.comp.get("group_executor_pool")

                    def _on_done(drone_id: int) -> None:
                        nonlocal done
                        self._readiness_report["wait_for_params"][drone_id] = True
                        self.comp["telemetry"].record_event(
                            "wait_for_params", drone_id=drone_id, ok=True
                        )
                        done += 1
                        self._progress.step(done, total, detail=f"drone={drone_id}")

                    if group_pool is not None:
                        from ..adapters.wait_for_params import (
                            wait_for_params_per_group,
                        )

                        wait_for_params_per_group(
                            self.comp["transport"],
                            self.comp["fleet"],
                            group_pool,
                            on_done=_on_done,
                        )
                    else:
                        for drone_id in self.comp["fleet"].all_ids():
                            self.comp["transport"].wait_for_params(drone_id)
                            _on_done(drone_id)
                except Exception as exc:
                    self._fail_start(
                        "参数同步失败",
                        definition=MissionErrors.Readiness.WAIT_FOR_PARAMS_FAILED,
                        exception=exc,
                        drone_id=drone_id,
                    )
```

- [ ] **Step 4: Wrap phase 3 — reset_estimator**

Replace `if self.comp["config"].comm.readiness_reset_estimator:` block with:

```python
        if comm.readiness_reset_estimator:
            total = len(self.comp["fleet"].all_ids())
            with self._phase("reset_estimator", "重置估计器", total=total):
                drone_id = None
                done = 0
                try:
                    for drone_id in self.comp["fleet"].all_ids():
                        self.comp["transport"].reset_estimator_and_wait(drone_id)
                        self._readiness_report["reset_estimator"][drone_id] = True
                        self.comp["telemetry"].record_event(
                            "reset_estimator", drone_id=drone_id, ok=True
                        )
                        done += 1
                        self._progress.step(done, total, detail=f"drone={drone_id}")
                except Exception as exc:
                    self._fail_start(
                        "重置估计器失败",
                        definition=MissionErrors.Readiness.RESET_ESTIMATOR_FAILED,
                        exception=exc,
                        drone_id=drone_id,
                    )
```

- [ ] **Step 5: Wrap phase 4 — onboard_controller**

Replace the current `output_mode = self.comp["config"].control.output_mode` block through the end of its except with:

```python
        output_mode = self.comp["config"].control.output_mode
        onboard_ctrl = self.comp["config"].control.onboard_controller
        total = len(self.comp["fleet"].all_ids())
        with self._phase(
            "onboard_controller", "设置 onboard controller", total=total
        ):
            drone_id = None
            controller_switched: list[int] = []
            done = 0
            try:
                for drone_id in self.comp["fleet"].all_ids():
                    self.comp["transport"].set_onboard_controller(
                        drone_id, onboard_ctrl
                    )
                    controller_switched.append(drone_id)
                    self.comp["telemetry"].record_event(
                        "set_onboard_controller",
                        drone_id=drone_id,
                        controller=onboard_ctrl,
                        ok=True,
                    )
                    done += 1
                    self._progress.step(done, total, detail=f"drone={drone_id}")
            except Exception as exc:
                self.comp["telemetry"].record_event(
                    "set_onboard_controller",
                    drone_id=drone_id,
                    controller=onboard_ctrl,
                    ok=False,
                    error=str(exc),
                )
                if output_mode == "full_state":
                    for rollback_drone_id in reversed(controller_switched):
                        try:
                            self.comp["transport"].set_onboard_controller(
                                rollback_drone_id, "pid"
                            )
                            self.comp["telemetry"].record_event(
                                "rollback_onboard_controller",
                                drone_id=rollback_drone_id,
                                from_controller=onboard_ctrl,
                                controller="pid",
                                ok=True,
                            )
                        except Exception as rollback_exc:
                            self.comp["telemetry"].record_event(
                                "rollback_onboard_controller",
                                drone_id=rollback_drone_id,
                                from_controller=onboard_ctrl,
                                controller="pid",
                                ok=False,
                                error=str(rollback_exc),
                            )
                    self._fail_start(
                        "full_state 模式下设置 onboard controller 失败，中止启动",
                        exception=exc,
                        drone_id=drone_id,
                        controller=onboard_ctrl,
                        output_mode=output_mode,
                    )
                self._progress.warn(
                    f"onboard controller {onboard_ctrl} 设置失败 (drone={drone_id}): {exc}"
                )
                logger.warning(
                    "onboard controller %s 设置失败 (drone=%s): %s —— 继续启动，但沿用机载当前 controller 状态",
                    onboard_ctrl,
                    drone_id,
                    exc,
                )
```

- [ ] **Step 6: Wrap phase 5 — pose_source**

Replace both the `# 启动定位` block AND the `# 等待定位稳定` block with a single `_phase`:

```python
        total = len(self.comp["fleet"].all_ids())
        with self._phase("pose_source", "定位就绪", total=total):
            try:
                self.comp["pose_source"].register_callback(self._on_pose_update)
                self.comp["pose_source"].start()
            except Exception as exc:
                self._fail_start(
                    "定位源启动失败",
                    definition=MissionErrors.Readiness.POSE_SOURCE_START_FAILED,
                    exception=exc,
                )

            console_tap = self.comp.get("console_tap")
            if console_tap is not None:
                try:
                    console_tap._on_line = self._on_console_line
                    console_tap.start()
                except Exception:
                    logger.exception("Console tap start failed; 继续启动")

            for _ in range(20):
                snapshot = self.comp["pose_bus"].latest()
                if snapshot:
                    fresh_count = int(sum(1 for f in snapshot.fresh_mask if f))
                    self._progress.step(fresh_count, total)
                    if all(snapshot.fresh_mask):
                        self._readiness_report["pose_ready"] = True
                        self.comp["telemetry"].record_event("pose_ready", ok=True)
                        break
                time.sleep(0.1)
            else:
                self.comp["telemetry"].record_event("pose_ready", ok=False)
                self._fail_start(
                    "部分无人机定位未就绪，中止任务",
                    definition=MissionErrors.Readiness.POSE_TIMEOUT,
                )
```

- [ ] **Step 7: Wrap phase 6 — health_ready**

Replace the `logger.info("等待健康数据...")` block with:

```python
        total = len(self.comp["fleet"].all_ids())
        with self._phase("health_ready", "健康数据就绪", total=total):
            for _ in range(30):
                health_samples = self.comp["health_bus"].latest()
                ready_ids = [
                    drone_id
                    for drone_id in self.comp["fleet"].all_ids()
                    if drone_id in health_samples
                    and "pm.vbat" in health_samples[drone_id].values
                ]
                self._progress.step(len(ready_ids), total)
                if len(ready_ids) == total:
                    self._readiness_report["health_ready"] = True
                    self.comp["telemetry"].record_event("health_ready", ok=True)
                    break
                time.sleep(0.1)
            else:
                self.comp["telemetry"].record_event("health_ready", ok=False)
                self._fail_start(
                    "健康状态数据未就绪，中止任务",
                    definition=MissionErrors.Readiness.HEALTH_TIMEOUT,
                )
```

- [ ] **Step 8: Wrap phase 7 — trajectory_upload**

Wrap the existing `leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)` … `self._set_trajectory_state("ready")` block. Replace that entire block with:

```python
        leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
        if trajectory_enabled and leader_ref.mode == "trajectory":
            per_leader = leader_ref.trajectory.get("per_leader", {})
            leader_ids = list(self.comp["fleet"].leader_ids())
            total = len(leader_ids)
            with self._phase("trajectory_upload", "轨迹上传", total=total):
                try:
                    trajectory_upload_specs = {}
                    for drone_id in leader_ids:
                        spec = per_leader.get(drone_id, {})
                        pieces = spec.get("pieces", [])
                        start_addr = spec.get("start_addr", 0)
                        trajectory_id = spec.get("trajectory_id", 1)
                        estimated_bytes = len(pieces) * POLY4D_RAW_PIECE_BYTES
                        fits_memory = (
                            start_addr + estimated_bytes <= TRAJECTORY_MEMORY_BYTES
                        )
                        self.comp["telemetry"].record_event(
                            "trajectory_budget_check",
                            drone_id=drone_id,
                            pieces=len(pieces),
                            estimated_bytes=estimated_bytes,
                            start_addr=start_addr,
                            capacity=TRAJECTORY_MEMORY_BYTES,
                            fits_memory=fits_memory,
                        )
                        trajectory_upload_specs[drone_id] = {
                            "pieces": pieces,
                            "start_addr": start_addr,
                            "trajectory_id": trajectory_id,
                        }

                    upload_results = self.comp[
                        "transport"
                    ].upload_trajectories_by_group(
                        trajectory_upload_specs,
                        parallel_groups=comm.trajectory_upload_groups_in_parallel,
                    )

                    done = 0
                    for drone_id in leader_ids:
                        spec = per_leader.get(drone_id, {})
                        pieces = spec.get("pieces", [])
                        start_addr = spec.get("start_addr", 0)
                        trajectory_id = spec.get("trajectory_id", 1)
                        estimated_bytes = len(pieces) * POLY4D_RAW_PIECE_BYTES
                        fits_memory = (
                            start_addr + estimated_bytes <= TRAJECTORY_MEMORY_BYTES
                        )
                        piece_count = int(upload_results[drone_id]["piece_count"])
                        self._readiness_report["trajectory_prepare"][drone_id] = {
                            "uploaded": True,
                            "defined": True,
                            "pieces": piece_count,
                            "estimated_bytes": estimated_bytes,
                            "fits_memory": fits_memory,
                            "trajectory_id": trajectory_id,
                            "nominal_position": spec.get("nominal_position"),
                        }
                        self.comp["telemetry"].record_event(
                            "trajectory_prepare",
                            drone_id=drone_id,
                            uploaded=True,
                            defined=True,
                            pieces=piece_count,
                            estimated_bytes=estimated_bytes,
                            fits_memory=fits_memory,
                            trajectory_id=trajectory_id,
                            nominal_position=spec.get("nominal_position"),
                        )
                        done += 1
                        self._progress.step(
                            done, total, detail=f"leader={drone_id} pieces={piece_count}"
                        )
                except Exception as exc:
                    self._fail_start(
                        "轨迹准备失败",
                        definition=MissionErrors.Readiness.TRAJECTORY_PREPARE_FAILED,
                        exception=exc,
                    )

                self._set_trajectory_state("prepared")
                self.comp["telemetry"].record_event(
                    "trajectory_readiness_summary",
                    ok=all(
                        item.get("uploaded") and item.get("defined")
                        for item in self._readiness_report[
                            "trajectory_prepare"
                        ].values()
                    ),
                    leaders=self._readiness_report["trajectory_prepare"],
                )
                self._set_trajectory_state("ready")
                logger.info("=== Trajectory readiness summary ===")
                for drone_id, item in self._readiness_report[
                    "trajectory_prepare"
                ].items():
                    logger.info(
                        "leader=%s pieces=%s est_bytes=%s fits=%s uploaded=%s defined=%s",
                        drone_id,
                        item.get("pieces"),
                        item.get("estimated_bytes"),
                        item.get("fits_memory"),
                        item.get("uploaded"),
                        item.get("defined"),
                    )
```

- [ ] **Step 9: Wrap phase 8 — preflight**

Replace the FSM transitions around preflight and the `preflight_report = self.comp["preflight"].run()` block with:

```python
        if not self._safe_transition(MissionState.POSE_READY):
            self._fail_start(
                "FSM failed entering POSE_READY",
                definition=MissionErrors.Readiness.FSM_POSE_READY_TRANSITION_FAILED,
            )

        if not self._safe_transition(MissionState.PREFLIGHT):
            self._fail_start(
                "FSM failed entering PREFLIGHT",
                definition=MissionErrors.Readiness.FSM_PREFLIGHT_TRANSITION_FAILED,
            )
        self.comp["readiness_report"] = self._readiness_report

        with self._phase("preflight", "preflight"):
            try:
                preflight_report = self.comp["preflight"].run()
            except Exception as exc:
                self._fail_start(
                    "Preflight 执行异常",
                    definition=MissionErrors.Readiness.PREFLIGHT_EXCEPTION,
                    exception=exc,
                )
            self.comp["telemetry"].record_event(
                "preflight",
                ok=preflight_report.ok,
                failed_codes=preflight_report.failed_codes,
            )
            if not preflight_report.ok:
                self._fail_start(
                    f"Preflight failed: {preflight_report.reasons} codes={preflight_report.failed_codes}",
                    definition=MissionErrors.Readiness.PREFLIGHT_FAILED,
                    failed_codes=preflight_report.failed_codes,
                )
```

- [ ] **Step 10: Wrap phase 9 — takeoff_settle_align**

Replace from `# Takeoff - 所有无人机起飞` down to the final `return True` at the end of `_start_impl`, wrapping everything in one `_phase`:

```python
        with self._phase("takeoff_settle_align", "起飞 / settle / align"):
            if not self._safe_transition(MissionState.TAKEOFF):
                self._fail_start(
                    "FSM failed entering TAKEOFF",
                    definition=MissionErrors.Readiness.FSM_TAKEOFF_TRANSITION_FAILED,
                )

            self.comp["leader_executor"].execute(
                [self._leader_takeoff_action(self.comp["fleet"].leader_ids())]
            )
            follower_takeoff_result = self.comp["follower_executor"].takeoff(
                self.comp["fleet"].follower_ids(), height=0.5, duration=2.0
            )
            self.telemetry_reporter.record_executor_summary(
                "follower_takeoff_execution", [follower_takeoff_result]
            )

            time.sleep(3.0)

            if not self._safe_transition(MissionState.SETTLE):
                self._fail_start(
                    "FSM failed entering SETTLE",
                    definition=MissionErrors.Readiness.FSM_SETTLE_TRANSITION_FAILED,
                )
            time.sleep(2.0)

            initial_leader_ref = self.comp["leader_ref_gen"].reference_at(0.0)
            if startup_mode == "manual_leader":
                initial_leader_ref = self._manual_initial_structure_reference()
            if (
                startup_mode == "auto"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "batch_goto"
            ):
                from ..runtime.command_plan import LeaderAction

                align_action = LeaderAction(
                    kind="batch_goto",
                    drone_ids=self.comp["fleet"].leader_ids(),
                    payload={
                        "positions": initial_leader_ref.positions,
                        "duration": 2.0,
                    },
                )
                self.comp["leader_executor"].execute([align_action])
                self.comp["telemetry"].record_event(
                    "formation_align", ok=True, duration=2.0
                )
                time.sleep(2.2)
            elif (
                startup_mode == "manual_leader"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "batch_goto"
            ):
                from ..runtime.command_plan import LeaderAction

                align_action = LeaderAction(
                    kind="batch_goto",
                    drone_ids=self.comp["fleet"].leader_ids(),
                    payload={
                        "positions": initial_leader_ref.positions,
                        "duration": 2.0,
                    },
                )
                self.comp["leader_executor"].execute([align_action])
                self.comp["telemetry"].record_event(
                    "manual_structure_align", ok=True, duration=2.0
                )
                time.sleep(2.2)
            elif (
                startup_mode == "auto"
                and initial_leader_ref is not None
                and initial_leader_ref.mode == "trajectory"
            ):
                trajectory_entry_positions = self._trajectory_entry_start_positions(
                    initial_leader_ref
                )
                if trajectory_entry_positions:
                    from ..runtime.command_plan import LeaderAction

                    align_action = LeaderAction(
                        kind="batch_goto",
                        drone_ids=self.comp["fleet"].leader_ids(),
                        payload={
                            "positions": trajectory_entry_positions,
                            "duration": 2.0,
                        },
                    )
                    self.comp["leader_executor"].execute([align_action])
                    self.comp["telemetry"].record_event(
                        "trajectory_entry_align",
                        ok=True,
                        duration=2.0,
                        positions=trajectory_entry_positions,
                    )
                    time.sleep(2.2)

            snapshot = self.comp["pose_bus"].latest()
            if snapshot:
                for drone_id in self.comp["fleet"].all_ids():
                    idx = self.comp["fleet"].id_to_index(drone_id)
                    if snapshot.positions[idx][2] < 0.3:
                        self.comp["telemetry"].record_event(
                            "takeoff_validation", ok=False, drone_id=drone_id
                        )
                        self._record_error_event(
                            definition=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                            message="起飞后高度验证失败",
                            drone_id=drone_id,
                            altitude=float(snapshot.positions[idx][2]),
                        )
                        self._emergency_land(
                            trigger_error=MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED,
                        )
                        raise _StartupAborted("takeoff validation failed")

            if startup_mode == "manual_leader":
                self._initialize_manual_mode(snapshot)

        logger.info("系统就绪")
        self.comp["telemetry"].record_event("startup_complete", ok=True)
        self._progress.close()
        return True
```

- [ ] **Step 11: Close progress on the failure path too**

In `start()` (the wrapper, not `_start_impl`), update so that the reporter is always closed:

```python
    def start(self):
        """启动"""
        logger.info("=== 启动真机任务 ===")
        try:
            return self._start_impl()
        except _StartupAborted:
            return False
        finally:
            self._progress.close()
```

- [ ] **Step 12: Run the full test suite**

```bash
python -m pytest src/tests/ -v
```

Expected: all pre-existing tests still green; no new failures.

- [ ] **Step 13: Commit**

```bash
git add src/app/run_real.py
git commit -m "feat(run_real): wrap start() phases with _phase (9 phases + dynamic total)"
```

---

## Task 8: CLI integration — reporter, logging, `--verbose`

**Files:**
- Modify: `src/app/cli.py`
- Modify: `src/tests/test_cli.py` (if present) or create if missing

- [ ] **Step 1: Inspect existing test_cli.py**

```bash
ls src/tests/test_cli.py
```

If it exists read it to see the style; otherwise Step 3 creates it.

- [ ] **Step 2: Write failing test for --verbose flag**

Append (or create) `src/tests/test_cli.py`:

```python
"""CLI argument parsing tests."""

from src.app.cli import build_parser


def test_run_subcommand_accepts_verbose_flag():
    parser = build_parser()
    args = parser.parse_args(["run", "--verbose"])
    assert args.verbose is True


def test_run_subcommand_verbose_short_flag():
    parser = build_parser()
    args = parser.parse_args(["run", "-v"])
    assert args.verbose is True


def test_run_subcommand_verbose_defaults_false():
    parser = build_parser()
    args = parser.parse_args(["run"])
    assert args.verbose is False


def test_top_level_verbose_flag():
    """Compat entry `python main.py --verbose` should also work."""
    parser = build_parser()
    args = parser.parse_args(["--verbose"])
    assert args.verbose is True
```

- [ ] **Step 3: Run tests and verify failure**

```bash
python -m pytest src/tests/test_cli.py -v -k verbose
```

Expected: `AttributeError: Namespace has no attribute 'verbose'` or similar.

- [ ] **Step 4: Modify `_build_run_parent` and `_run_real`**

In `src/app/cli.py`, modify `_build_run_parent` to add the flag. Replace its body with:

```python
def _build_run_parent(
    config_parent: argparse.ArgumentParser,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False, parents=[config_parent])
    parser.add_argument(
        "--startup-mode",
        choices=["auto", "manual_leader"],
        default=None,
        help="启动模式覆盖：auto 或 manual_leader",
    )
    parser.add_argument(
        "--skip-confirm",
        action="store_true",
        help="跳过启动前按 Enter 确认",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="展开每台无人机的启动细节；root logger 切 DEBUG",
    )
    return parser
```

Modify `_run_command` to forward the new arg:

```python
def _run_command(args: argparse.Namespace) -> int:
    return _run_real(
        args.config_dir, args.startup_mode, args.skip_confirm, args.verbose
    )
```

Modify `_run_real` to build reporter and configure logging:

```python
def _run_real(
    config_dir: str,
    startup_mode: str | None,
    skip_confirm: bool,
    verbose: bool,
) -> int:
    from .log_setup import configure_logging
    from .startup_progress import make_reporter

    reporter = make_reporter(verbose=verbose)
    configure_logging(verbose=verbose, reporter=reporter)

    print("=== Crazyflie AFC Swarm ===")
    print("构建系统...")

    components = build_app(config_dir, startup_mode_override=startup_mode)
    app = RealMissionApp(components, progress=reporter)

    print("系统构建完成")
    if not skip_confirm:
        print("按Enter启动（需要真机连接）")
        input()

    try:
        app.start()
        app.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        app.shutdown()
        reporter.close()
    return 0
```

Modify the default-command branch of `main` to pass `verbose`:

```python
def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command is None:
        config_dir = _command_config_dir(args)
        if args.trajectory_budget:
            return print_trajectory_budget_summary(config_dir)
        return _run_real(
            config_dir, args.startup_mode, args.skip_confirm, args.verbose
        )

    handler = _command_handlers().get(args.command)
    if handler is None:
        raise SystemExit(f"Unsupported command: {args.command}")
    return handler(args)
```

- [ ] **Step 5: Run cli tests**

```bash
python -m pytest src/tests/test_cli.py -v
```

Expected: 4 verbose-flag tests pass; any pre-existing tests still pass.

- [ ] **Step 6: Manual smoke — text fallback**

```bash
AFC_NO_RICH=1 python -m src.app.cli run --skip-confirm 2>&1 | head -20
```

Expected: `[HH:MM:SS] === Crazyflie AFC Swarm ===` style prefixed lines, then (because no radio is attached) an error at the `connect` phase. Key acceptance: "[1/N] 连接 Crazyflie ..." AND "[1/N] 连接 Crazyflie FAIL ..." appear in the output.

- [ ] **Step 7: Manual smoke — rich TUI (only when a terminal is attached)**

```bash
python -m src.app.cli run --skip-confirm
```

Expected: Rich Live panel with phase rows appears and disappears cleanly after the connect failure; terminal is restored (no orphan cursor).

- [ ] **Step 8: Commit**

```bash
git add src/app/cli.py src/tests/test_cli.py
git commit -m "feat(cli): wire startup progress reporter + --verbose into run"
```

---

## Task 9: README — document optional `rich` dependency and startup UX

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update optional-dependencies list**

In `README.md`, locate the "可选:" block under "环境说明" (roughly line 333) and append `rich` as a new bullet:

```markdown
可选:

- `cflinkcpp`（`comm.radio_driver=cpp` 时）
- `Sphinx` / `sphinx-rtd-theme`（构建 `docs/` 时）
- `rich`（真机 `run` 启动阶段 TUI；未安装时自动降级为滚动文本）
```

- [ ] **Step 2: Add a short "启动状态显示" subsection under "真机主入口"**

After the `python -m src.app.cli run --startup-mode manual_leader` code block (around line 185), insert:

```markdown
**启动状态显示：**

- 默认使用 rich TUI 展示 9 个启动阶段（连接 / 等参数 / 重置估计器 / onboard controller / 定位 / 健康 / 轨迹上传 / preflight / takeoff+settle+align）。
- `--verbose` / `-v` 展开每台无人机的 per-step 细节；root logger 切 DEBUG。
- rich 未安装、非 TTY（重定向）、或 `AFC_NO_RICH=1` 时自动降级为 `[HH:MM:SS] [i/N] 阶段名 ...` 滚动文本。
- 每个阶段 begin/ok/fail 以 `startup_phase` 事件写入 telemetry。
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: document rich optional dependency and startup phase UX"
```

---

## Self-Review Notes

- **Spec coverage:** All 9 phases from the spec are wrapped in Task 7. The reporter protocol, Null/Text/Rich impls, factory, `configure_logging`, `_phase` helper, and CLI `--verbose` are each their own task. `AFC_NO_RICH` is tested in Task 3. `startup_phase` telemetry event is tested in Task 5. README update lands in Task 9.
- **Type consistency:** `StartupProgressReporter` protocol methods (`phase / step / warn / set_total_phases / close` + `console` property) match across NullProgressReporter, TextProgressReporter, RichProgressReporter, and the `_phase` helper usage in `run_real.py`.
- **Telemetry schema:** Uses existing `telemetry.record_event(event, **details)` signature (verified at `src/runtime/telemetry.py:150`). No new dataclass, no schema bump.
- **Failure plumbing:** `_fail_start` refactor in Task 6 is a separate commit from the phase wrapping in Task 7 so that bisection can distinguish the behavioral refactor from the UI overlay.
- **Backwards compat:** `RealMissionApp(components)` (no progress arg) still works thanks to the default `NullProgressReporter`; `python main.py` (no subcommand, no --verbose) still works because `verbose` defaults to False.
