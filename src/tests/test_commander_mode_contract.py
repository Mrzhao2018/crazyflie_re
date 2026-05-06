"""Commander mode transition contract tests."""

import numpy as np

from src.app.run_real import RealMissionApp
import src.runtime.landing_flow as landing_flow_module
from src.runtime.command_plan import FollowerAction
from src.runtime.mission_fsm import MissionState
from src.runtime.safety_manager import SafetyDecision
from src.tests.run_real_fixtures import build_components, make_snapshot


def test_orderly_land_records_streaming_to_high_level_stop_contract():
    components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
    components["config"].control.output_mode = "full_state"
    components["config"].control.onboard_controller = "mellinger"
    app = RealMissionApp(components)
    components["fsm"]._state = MissionState.RUN

    original_sleep = landing_flow_module.time.sleep
    try:
        landing_flow_module.time.sleep = lambda _seconds: None
        app._orderly_land(
            reason_event="mission_complete_land",
            safety_action="MISSION_COMPLETE",
            safety_reasons=["mission_complete"],
            safety_reason_codes=["MISSION_COMPLETE"],
            scheduler_reason="mission_complete",
            scheduler_diagnostics={"mission_complete": True},
            trajectory_terminal_reason="mission_complete",
        )
    finally:
        landing_flow_module.time.sleep = original_sleep

    switch_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "commander_mode_switch"
    )
    assert switch_event["details"]["code"] == "COMMANDER_MODE_SWITCH"
    assert switch_event["details"]["from_mode"] == "streaming_setpoint"
    assert switch_event["details"]["to_mode"] == "high_level_commander"
    assert switch_event["details"]["reason"] == "mission_complete_land"

    notify_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "setpoint_stop_notify"
    )
    assert notify_event["details"]["code"] == "SETPOINT_STOP_NOTIFY"
    assert notify_event["details"]["drone_ids"] == components["fleet"].follower_ids()
    assert notify_event["details"]["ok"] is True
    assert components["follower_executor"].stop_calls == [
        components["fleet"].follower_ids()
    ]
    land_event_names = [event["event"] for event in components["telemetry"].events]
    assert land_event_names.index("setpoint_stop_notify") < land_event_names.index(
        "leader_land_execution"
    )
    assert [
        call for call in components["transport"].controller_calls if call[1] == "pid"
    ] == [(drone_id, "pid") for drone_id in components["fleet"].follower_ids()]
    assert land_event_names.index("landing_onboard_controller") < land_event_names.index(
        "follower_land_execution"
    )


def test_stop_notify_failure_does_not_block_emergency_land():
    components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
    app = RealMissionApp(components)
    components["fsm"]._state = MissionState.RUN

    def raise_stop(_drone_ids):
        raise RuntimeError("notify failed")

    components["follower_executor"].stop_velocity_mode = raise_stop

    original_sleep = landing_flow_module.time.sleep
    try:
        landing_flow_module.time.sleep = lambda _seconds: None
        app._emergency_land()
    finally:
        landing_flow_module.time.sleep = original_sleep

    notify_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "setpoint_stop_notify"
    )
    assert notify_event["details"]["ok"] is False
    assert components["leader_executor"].actions
    assert components["follower_executor"].velocity_calls


def test_emergency_land_goes_directly_to_land_without_zero_velocity_brake():
    components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
    app = RealMissionApp(components)
    components["fsm"]._state = MissionState.RUN

    original_sleep = landing_flow_module.time.sleep
    try:
        landing_flow_module.time.sleep = lambda _seconds: None
        app._emergency_land()
    finally:
        landing_flow_module.time.sleep = original_sleep

    event_names = [event["event"] for event in components["telemetry"].events]
    assert "follower_brake" not in event_names
    assert not components["follower_executor"].hold_calls
    assert components["follower_executor"].stop_calls == [
        components["fleet"].follower_ids()
    ]
    assert components["follower_executor"].land_calls == []
    descent_actions = [
        action
        for batch in components["follower_executor"].velocity_calls
        for action in batch
    ]
    assert descent_actions
    assert {action.drone_id for action in descent_actions} == set(
        components["fleet"].follower_ids()
    )
    assert all(action.velocity[2] < 0.0 for action in descent_actions)
    assert any(
        event["event"] == "follower_direct_descent_execution"
        for event in components["telemetry"].events
    )
    assert "setpoint_stop_notify" in event_names
    assert components["leader_executor"].actions


def test_hold_timeout_land_goes_directly_to_land_without_zero_velocity_brake():
    components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
    app = RealMissionApp(components)
    components["fsm"]._state = MissionState.HOLD

    original_sleep = landing_flow_module.time.sleep
    try:
        landing_flow_module.time.sleep = lambda _seconds: None
        app._orderly_land(
            reason_event="hold_timeout_land",
            safety_action="HOLD_TIMEOUT",
            safety_reasons=["hold_timeout"],
            safety_reason_codes=["HOLD_TIMEOUT"],
            scheduler_reason="hold_timeout",
            scheduler_diagnostics={"hold_timeout": True},
            trajectory_terminal_reason="hold_timeout",
        )
    finally:
        landing_flow_module.time.sleep = original_sleep

    event_names = [event["event"] for event in components["telemetry"].events]
    assert "follower_brake" not in event_names
    assert not components["follower_executor"].hold_calls
    assert components["follower_executor"].stop_calls == [
        components["fleet"].follower_ids()
    ]
    assert components["follower_executor"].land_calls == []
    descent_actions = [
        action
        for batch in components["follower_executor"].velocity_calls
        for action in batch
    ]
    assert descent_actions
    assert {action.drone_id for action in descent_actions} == set(
        components["fleet"].follower_ids()
    )
    assert all(action.velocity[2] < 0.0 for action in descent_actions)


def test_runtime_records_streaming_setpoint_activity():
    class OneShotStreamingScheduler:
        def __init__(self):
            self.app = None

        def plan(
            self,
            snapshot,
            mission_state,
            leader_ref,
            commands,
            safety_decision,
            parked_follower_ids=None,
        ):
            return type(
                "Plan",
                (),
                {
                    "leader_actions": [],
                    "follower_actions": [
                        FollowerAction(
                            kind="velocity",
                            drone_id=5,
                            velocity=np.array([0.1, 0.0, 0.0]),
                        )
                    ],
                    "hold_actions": [],
                    "diagnostics": {"reason": "execute"},
                },
            )()

    components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
    components["scheduler"] = OneShotStreamingScheduler()
    app = RealMissionApp(components)
    components["scheduler"].app = app
    components["fsm"]._state = MissionState.SETTLE

    original_execute_velocity = components["follower_executor"].execute_velocity

    def stop_after_velocity(actions):
        result = original_execute_velocity(actions)
        app._running = False
        return result

    components["follower_executor"].execute_velocity = stop_after_velocity
    app.run()

    streaming_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "streaming_setpoint_active"
    )
    assert streaming_event["details"]["code"] == "STREAMING_SETPOINT_ACTIVE"
    assert streaming_event["details"]["action_count"] == 1
    assert streaming_event["details"]["kinds"] == ["velocity"]
    assert streaming_event["details"]["drone_ids"] == [5]
