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
