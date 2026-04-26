"""RealMissionApp velocity stream watchdog tests."""

import time

from src.app.mission_errors import MissionErrors
from src.app.run_real import RealMissionApp
from src.runtime.failure_policy import VELOCITY_STREAM_WATCHDOG_FACTOR
from src.tests.run_real_fixtures import build_components, make_snapshot
from src.runtime.mission_fsm import MissionState
from src.runtime.safety_manager import SafetyDecision


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
app = RealMissionApp(components)

transport = components["transport"]
transport._last_velocity_command_time = {
    5: time.monotonic() - 10.0,
    6: time.monotonic() - 10.0,
}

app._check_velocity_stream_watchdog(snapshot_t_meas=1.0)

watchdog_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "velocity_stream_watchdog"
)
assert watchdog_event["details"]["ok"] is False
assert watchdog_event["details"]["follower_tx_freq"] == 8.0
assert watchdog_event["details"]["category"] == MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.category
assert watchdog_event["details"]["code"] == MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.code
assert watchdog_event["details"]["stage"] == MissionErrors.Runtime.VELOCITY_STREAM_WATCHDOG.stage
assert watchdog_event["details"]["radio_groups"][2]["drone_ids"] == [5, 6]
assert len(watchdog_event["details"]["stale_followers"]) == 2
assert all(
    item["watchdog_timeout"] == (1.0 / 8.0) * VELOCITY_STREAM_WATCHDOG_FACTOR
    for item in watchdog_event["details"]["stale_followers"]
)


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
    watchdog_action="hold",
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.RUN
transport = components["transport"]
transport._last_velocity_command_time = {5: time.monotonic() - 10.0}

stale = app._check_velocity_stream_watchdog(snapshot_t_meas=1.0)

assert [item["drone_id"] for item in stale] == [5]
assert components["fsm"].state() == MissionState.HOLD
assert len(components["follower_executor"].hold_calls) == 1
hold_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "hold_entered"
)
assert hold_event["details"]["reason"] == "watchdog"
assert hold_event["details"]["category"] == MissionErrors.Runtime.WATCHDOG_HOLD.category
assert hold_event["details"]["code"] == MissionErrors.Runtime.WATCHDOG_HOLD.code
assert hold_event["details"]["stage"] == MissionErrors.Runtime.WATCHDOG_HOLD.stage


components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3), make_snapshot(4)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
    watchdog_action="degrade",
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
transport = components["transport"]
transport._last_velocity_command_time = {
    5: time.monotonic() - 10.0,
    6: time.monotonic() - 10.0,
}

app.run()

watchdog_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "velocity_stream_watchdog" and event["details"]["action"] == "degrade"
)
assert sorted(item["drone_id"] for item in watchdog_event["details"]["stale_followers"]) == [5, 6]
degrade_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "watchdog_degrade"
)
assert degrade_event["details"]["follower_ids"] == [5, 6]
assert degrade_event["details"]["category"] == MissionErrors.Runtime.WATCHDOG_DEGRADE.category
assert degrade_event["details"]["code"] == MissionErrors.Runtime.WATCHDOG_DEGRADE.code
assert degrade_event["details"]["stage"] == MissionErrors.Runtime.WATCHDOG_DEGRADE.stage
assert degrade_event["details"]["radio_groups"][2]["drone_ids"] == [5, 6]
parked_entries = [ids for ids in components["scheduler"].parked_history if ids]
assert parked_entries  # 至少一次 degrade 触发过 parked plan
assert all(ids == [5, 6] for ids in parked_entries)
hold_summary = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "follower_hold_execution"
)
assert hold_summary["details"]["radio_groups"][2]["successes"] == [5, 6]
assert len(components["follower_executor"].velocity_calls) == 0


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
    watchdog_action="degrade",
)
app = RealMissionApp(components)
app._apply_watchdog_degrade(
    [{"drone_id": 5, "command_age": 1.0, "watchdog_timeout": 0.3}]
)
app._clear_watchdog_degrade(active_commands={5: None})
recovered_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "watchdog_degrade_recovered"
)
assert recovered_event["details"]["category"] == MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.category
assert recovered_event["details"]["code"] == MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.code
assert recovered_event["details"]["stage"] == MissionErrors.Runtime.WATCHDOG_DEGRADE_RECOVERED.stage
assert recovered_event["details"]["radio_groups"][2]["drone_ids"] == [5]


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
components["fleet"]._id_to_radio[6] = 3
app = RealMissionApp(components)

retryable_failure = {
    "kind": "velocity",
    "successes": [6],
    "failures": [
        {
            "drone_id": 5,
            "radio_group": 2,
            "command_kind": "velocity",
            "failure_category": "timeout",
            "retryable": True,
        }
    ],
}

app._apply_follower_failure_policy(retryable_failure)
assert not any(
    event["event"] == "executor_group_degrade"
    for event in components["telemetry"].events
)
assert app.failure_policy.follower_group_failure_streaks[2] == 1

app._apply_follower_failure_policy(retryable_failure)
degrade_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "executor_group_degrade"
)
assert degrade_event["details"]["category"] == MissionErrors.Runtime.EXECUTOR_GROUP_DEGRADE.category
assert degrade_event["details"]["code"] == MissionErrors.Runtime.EXECUTOR_GROUP_DEGRADE.code
assert degrade_event["details"]["stage"] == MissionErrors.Runtime.EXECUTOR_GROUP_DEGRADE.stage
assert degrade_event["details"]["action"] == "degrade"
assert degrade_event["details"]["follower_ids"] == [5]
assert degrade_event["details"]["triggered_groups"][0]["group_id"] == 2
assert degrade_event["details"]["triggered_groups"][0]["streak"] == 2
assert degrade_event["details"]["triggered_groups"][0]["non_retryable"] is False
assert degrade_event["details"]["group_failure_streaks"][2] == 2
assert 5 in app.failure_policy.watchdog_degraded_followers
assert components["fsm"].state() != MissionState.HOLD


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.RUN

non_retryable_failure = {
    "kind": "velocity",
    "successes": [],
    "failures": [
        {
            "drone_id": 5,
            "radio_group": 2,
            "command_kind": "velocity",
            "failure_category": "link_lookup",
            "retryable": False,
        }
    ],
}

app._apply_follower_failure_policy(non_retryable_failure)
hold_policy_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "executor_group_hold"
)
assert hold_policy_event["details"]["category"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.category
assert hold_policy_event["details"]["code"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.code
assert hold_policy_event["details"]["stage"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.stage
assert hold_policy_event["details"]["action"] == "hold"
assert hold_policy_event["details"]["triggered_groups"][0]["non_retryable"] is True
hold_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "hold_entered"
)
assert hold_event["details"]["reason"] == "executor_failure"
assert hold_event["details"]["category"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.category
assert hold_event["details"]["code"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.code
assert hold_event["details"]["stage"] == MissionErrors.Runtime.EXECUTOR_GROUP_HOLD.stage
assert components["fsm"].state() == MissionState.HOLD

print("[OK] RealMissionApp velocity watchdog verified")