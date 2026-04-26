"""FailurePolicy executor_group_failure_streak config contract."""

from src.app.run_real import RealMissionApp
from src.runtime.mission_fsm import MissionState
from src.runtime.safety_manager import SafetyDecision
from src.tests.run_real_fixtures import build_components, make_snapshot


components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
components["fleet"]._id_to_radio[6] = 3
components["config"].safety.executor_group_failure_streak = 3
app = RealMissionApp(components)
components["fsm"]._state = MissionState.RUN

retryable_failure = {
    "kind": "velocity",
    "successes": [6],  # 保持另一个 radio_group 处于活跃但健康状态，避免直接走 hold
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
app._apply_follower_failure_policy(retryable_failure)
assert not any(
    event["event"] == "executor_group_degrade"
    for event in components["telemetry"].events
), "streak=3 时前两次 retryable failure 不应触发 degrade"
assert app.failure_policy.follower_group_failure_streaks[2] == 2

app._apply_follower_failure_policy(retryable_failure)
degrade_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "executor_group_degrade"
)
assert degrade_event["details"]["streak_threshold"] == 3
assert degrade_event["details"]["triggered_groups"][0]["streak"] == 3
assert degrade_event["details"]["group_failure_streaks"][2] == 3

print("[OK] FailurePolicy executor_group_failure_streak config verified")
