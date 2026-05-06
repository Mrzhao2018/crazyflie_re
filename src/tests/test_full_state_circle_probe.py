import math

import numpy as np

from src.app.full_state_circle_probe import (
    _switch_to_full_state_mode,
    apply_z_bias_compensation,
    apply_param_overrides,
    circle_setpoint,
    parse_param_assignment,
    restore_param_overrides,
    tracking_error_components,
)


def test_circle_setpoint_generates_consistent_pos_vel_acc():
    center = np.array([1.0, 2.0, 0.5], dtype=float)
    radius = 0.2
    period_s = 4.0

    pos0, vel0, acc0 = circle_setpoint(center, radius=radius, period_s=period_s, t_s=0.0)
    np.testing.assert_allclose(pos0, np.array([1.2, 2.0, 0.5]))
    np.testing.assert_allclose(vel0, np.array([0.0, math.pi * 0.1, 0.0]))
    np.testing.assert_allclose(acc0, np.array([-math.pi**2 * 0.05, 0.0, 0.0]))

    pos_q, vel_q, acc_q = circle_setpoint(
        center,
        radius=radius,
        period_s=period_s,
        t_s=period_s / 4.0,
    )
    np.testing.assert_allclose(pos_q, np.array([1.0, 2.2, 0.5]), atol=1e-12)
    np.testing.assert_allclose(vel_q, np.array([-math.pi * 0.1, 0.0, 0.0]), atol=1e-12)
    np.testing.assert_allclose(acc_q, np.array([0.0, -math.pi**2 * 0.05, 0.0]), atol=1e-12)


def test_tracking_error_components_split_xy_and_signed_z():
    target = np.array([1.0, 2.0, 0.5], dtype=float)
    current = np.array([1.03, 1.96, 0.56], dtype=float)

    components = tracking_error_components(current, target)

    assert np.isclose(components["err_xy"], 0.05)
    assert np.isclose(components["err_z"], 0.06)
    assert np.isclose(components["err"], math.sqrt(0.05**2 + 0.06**2))


def test_apply_z_bias_compensation_limited_to_configured_range():
    target = np.array([1.0, 2.0, 0.5], dtype=float)

    compensated = apply_z_bias_compensation(target, z_bias=0.06, max_abs_bias=0.04)

    np.testing.assert_allclose(compensated, np.array([1.0, 2.0, 0.46]))
    np.testing.assert_allclose(target, np.array([1.0, 2.0, 0.5]))


def test_switch_to_full_state_can_notify_before_stopping_hlc():
    events = []

    class _FakeTransport:
        def notify_setpoint_stop(self, drone_id):
            events.append(("notify", drone_id))

        def set_onboard_controller(self, drone_id, controller):
            events.append(("controller", drone_id, controller))

    class _FakeHighLevelCommander:
        def stop(self):
            events.append(("hl_stop", 5))

    class _FakeLinkManager:
        def get(self, drone_id):
            assert drone_id == 5
            return type(
                "FakeScf",
                (),
                {
                    "cf": type(
                        "FakeCf",
                        (),
                        {"high_level_commander": _FakeHighLevelCommander()},
                    )()
                },
            )()

    _switch_to_full_state_mode(
        link_manager=_FakeLinkManager(),
        transport=_FakeTransport(),
        drone_id=5,
        controller="mellinger",
        notify_before_full_state=True,
    )

    assert events == [
        ("notify", 5),
        ("hl_stop", 5),
        ("controller", 5, "mellinger"),
    ]


def test_parse_param_assignment_requires_name_value_pair():
    assert parse_param_assignment("ctrlMel.ki_z=0") == ("ctrlMel.ki_z", "0")

    try:
        parse_param_assignment("ctrlMel.ki_z")
        raise AssertionError("Expected invalid assignment to fail")
    except ValueError as exc:
        assert "--set-param" in str(exc)


def test_apply_and_restore_param_overrides():
    events = []

    class _FakeParam:
        def __init__(self):
            self.values = {"ctrlMel.ki_z": "0.05", "ctrlMel.ki_m_z": "500"}

        def get_value(self, name):
            events.append(("get", name))
            return self.values[name]

        def set_value(self, name, value):
            events.append(("set", name, value))
            self.values[name] = value

    scf = type(
        "FakeScf",
        (),
        {"cf": type("FakeCf", (), {"param": _FakeParam()})()},
    )()

    originals = apply_param_overrides(
        scf,
        ["ctrlMel.ki_z=0", "ctrlMel.ki_m_z=0"],
    )
    restore_param_overrides(scf, originals)

    assert originals == {"ctrlMel.ki_z": "0.05", "ctrlMel.ki_m_z": "500"}
    assert events == [
        ("get", "ctrlMel.ki_z"),
        ("set", "ctrlMel.ki_z", "0"),
        ("get", "ctrlMel.ki_m_z"),
        ("set", "ctrlMel.ki_m_z", "0"),
        ("set", "ctrlMel.ki_m_z", "500"),
        ("set", "ctrlMel.ki_z", "0.05"),
    ]
