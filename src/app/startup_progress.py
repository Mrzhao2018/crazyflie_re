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
