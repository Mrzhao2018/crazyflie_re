"""pytest bridge for script-style contract tests.

Historical convention in this repo: most ``src/tests/test_*.py`` files run
module-level assertions and end with ``print("[OK] ...")``; they are designed
to be invoked as ``python -m src.tests.test_xxx``.

Pytest's default collector skips those because it only looks for
``def test_*()`` functions. This conftest bridges the two styles:

* Modules that contain any ``def test_*`` / ``class Test*`` symbol are left to
  the default collector (avoid double-execution).
* Modules that don't expose pytest-style test symbols are collected as a
  single ``ScriptTestItem`` that re-executes the file with ``runpy`` under
  ``__main__``. Any unhandled exception (``AssertionError`` / ``ValueError`` /
  ``SystemExit`` non-zero) is reported as a test failure.
"""

from __future__ import annotations

import ast
import runpy
from pathlib import Path

import pytest

from src.tests.slow_guard import SLOW_TEST_FILES


def _has_pytest_style_tests(path: Path) -> bool:
    """Return True when *path* defines any ``test_*`` function or ``Test*`` class."""
    try:
        source = path.read_text(encoding="utf-8")
    except OSError:
        return False
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError:
        return False
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.name.startswith("test_"):
                return True
        elif isinstance(node, ast.ClassDef):
            if node.name.startswith("Test"):
                return True
    return False


class ScriptTestItem(pytest.Item):
    def __init__(self, *, name: str, parent, path: Path) -> None:
        super().__init__(name, parent)
        self._script_path = path

    def runtest(self) -> None:
        runpy.run_path(str(self._script_path), run_name="__main__")

    def reportinfo(self) -> tuple[Path, int, str]:
        return self._script_path, 0, f"script: {self.name}"


class ScriptTestFile(pytest.File):
    def collect(self):
        yield ScriptTestItem.from_parent(
            self, name=self.path.stem, path=Path(self.path)
        )


def pytest_collect_file(parent, file_path):
    """Wrap script-style test modules as single pytest items."""
    path = Path(file_path)
    if path.suffix != ".py":
        return None
    if not path.name.startswith("test_"):
        return None
    if _has_pytest_style_tests(path):
        return None  # let the default collector handle def test_* / Test* classes
    return ScriptTestFile.from_parent(parent, path=file_path)


def pytest_ignore_collect(collection_path, config):
    """Avoid even collecting slow script tests during the common fast test loop."""
    path = Path(collection_path)
    markexpr = getattr(config.option, "markexpr", "") or ""
    if path.name in SLOW_TEST_FILES and "not slow" in markexpr:
        return True
    return None


def pytest_collection_modifyitems(config, items):
    """Mark known long-running tests so daily runs can skip them with ``-m 'not slow'``."""
    slow_marker = pytest.mark.slow
    for item in items:
        path = Path(str(item.fspath))
        if path.name in SLOW_TEST_FILES:
            item.add_marker(slow_marker)
