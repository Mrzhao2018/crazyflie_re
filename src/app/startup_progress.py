"""Startup progress reporters: protocol, null impl, text impl.

Rich TUI impl and factory live in the same module but added in later tasks.
"""

from __future__ import annotations

import logging
import os
import sys
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


def make_reporter(
    verbose: bool = False,
    total_phases: int = 9,
    stdout: Any = None,
) -> StartupProgressReporter:
    """Pick a reporter by the fallback decision tree.

    Order: AFC_NO_RICH env -> rich import -> stdout.isatty() -> Rich.
    """
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
