from src.app.param_probe import (
    format_diagnostic_values,
    param_matches_filters,
    select_param_names,
)


def test_select_param_names_filters_case_insensitively():
    toc = {
        "stabilizer": {"controller": object(), "estimator": object()},
        "ctrlMel": {"kp_z": object(), "mass": object()},
        "kalman": {"varPZ": object()},
        "sound": {"effect": object()},
    }

    selected = select_param_names(toc, filters=["mel", "stabilizer", "varpz"])

    assert selected == [
        "ctrlMel.kp_z",
        "ctrlMel.mass",
        "kalman.varPZ",
        "stabilizer.controller",
        "stabilizer.estimator",
    ]


def test_param_matches_filters_allows_empty_filter_list():
    assert param_matches_filters("stabilizer.controller", []) is True
    assert param_matches_filters("stabilizer.controller", None) is True
    assert param_matches_filters("stabilizer.controller", ["mel"]) is False


def test_format_diagnostic_values_includes_thrust_and_motors():
    rendered = format_diagnostic_values(
        {
            "stabilizer.thrust": 42000,
            "pm.vbat": 3.91,
            "motor.m1": 10001,
            "motor.m2": 10002,
            "motor.m3": 10003,
            "motor.m4": 10004,
        }
    )

    assert rendered == (
        " thrust=42000.000 vbat=3.910 "
        "motors=[10001.000,10002.000,10003.000,10004.000]"
    )
