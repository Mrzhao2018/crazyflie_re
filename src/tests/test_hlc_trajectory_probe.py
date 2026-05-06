"""Single-drone HLC trajectory probe helpers."""

import numpy as np

from src.app.hlc_trajectory_probe import (
    estimate_phase_lag,
    tracking_error_components,
)


def test_tracking_error_components_reports_total_xy_and_z():
    current = np.array([1.03, 1.04, 0.58])
    target = np.array([1.0, 1.0, 0.5])

    components = tracking_error_components(current, target)

    assert np.isclose(components["err_xy"], 0.05)
    assert np.isclose(components["err_z"], 0.08)
    assert np.isclose(components["err"], np.sqrt(0.05**2 + 0.08**2))


def test_estimate_phase_lag_matches_nearest_reference_sample():
    reference = {
        0.0: np.array([1.0, 0.0, 0.5]),
        1.0: np.array([0.0, 1.0, 0.5]),
        2.0: np.array([-1.0, 0.0, 0.5]),
    }

    lag, distance = estimate_phase_lag(
        np.array([0.02, 0.98, 0.5]),
        expected_elapsed_s=1.5,
        reference_by_time=reference,
    )

    assert np.isclose(lag, 0.5)
    assert distance < 0.03
