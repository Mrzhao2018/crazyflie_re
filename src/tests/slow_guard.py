"""Helpers for slow script-style tests."""

from __future__ import annotations

import sys

import pytest

# Single source of truth for "slow" script tests: must match
# `pytest_collection_modifyitems` / `pytest_ignore_collect` in conftest.
SLOW_TEST_FILES: frozenset[str] = frozenset(
    {
        "test_baseline_sweep.py",
        "test_cli.py",
        "test_delay_compensation_ablation.py",
        "test_leader_trajectory.py",
        "test_model_order_ablation.py",
        "test_run_sim.py",
        "test_second_order_baseline_sweep.py",
        "test_trajectory_budget_summary.py",
        "test_trajectory_compare_runs.py",
        "test_trajectory_comparison.py",
        "test_trajectory_condition_ablation.py",
    }
)

__all__ = ["SLOW_TEST_FILES", "skip_when_fast_marker_requested"]


def skip_when_fast_marker_requested() -> None:
    """Skip module-level script tests before expensive top-level work starts."""
    args = " ".join(sys.argv)
    if "-m" in sys.argv and "not slow" in args:
        pytest.skip("slow test skipped by -m 'not slow'", allow_module_level=True)
