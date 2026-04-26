import numpy as np

from src.app.run_real import RealMissionApp
from src.config.schema import SafetyConfig
from src.runtime.command_plan import HoldAction
from src.runtime.mission_fsm import MissionState
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision, SafetyManager
from src.tests.run_real_fixtures import build_components, make_snapshot


class StopAfterPlanScheduler:
    def __init__(self):
        self.calls = []
        self.parked_history = []
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
        parked = list(parked_follower_ids or [])
        self.calls.append((snapshot.seq, mission_state, safety_decision.action))
        self.parked_history.append(parked)
        if self.app is not None:
            self.app._running = False
        return type(
            "Plan",
            (),
            {
                "leader_actions": [],
                "follower_actions": [],
                "hold_actions": [HoldAction(drone_id=drone_id) for drone_id in parked],
                "diagnostics": {"reason": "captured"},
            },
        )()


class FrameValiditySafety:
    def fast_gate(self, snapshot):
        return (False, [])

    def evaluate(
        self,
        snapshot,
        frame=None,
        commands=None,
        follower_ref=None,
        health=None,
        ignored_disconnected_ids=None,
    ):
        if frame is not None and not frame.valid:
            return SafetyDecision("HOLD", ["frame_invalid"])
        return SafetyDecision("EXECUTE", [])


class InvalidFrameEstimator:
    def __init__(self):
        self.calls = 0

    def estimate(self, snapshot, leader_ids):
        self.calls += 1
        return type(
            "Frame",
            (),
            {
                "valid": False,
                "condition_number": 200.0,
                "leader_positions": {
                    lid: snapshot.positions[i] for i, lid in enumerate(leader_ids)
                },
            },
        )()


def test_partial_disconnect_group_degrade_does_not_abort_mission():
    partial_disconnect = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=make_snapshot(1).positions.copy(),
        fresh_mask=np.array([True, True, True, True, False, False], dtype=bool),
        disconnected_ids=[5, 6],
    )
    components = build_components(
        [partial_disconnect],
        [SafetyDecision("EXECUTE", [])],
    )
    components["config"].safety = SafetyConfig(
        boundary_min=[-2.0, -2.0, -0.5],
        boundary_max=[2.0, 2.0, 2.5],
        pose_timeout=1.0,
        max_condition_number=100.0,
        hold_auto_land_timeout=0.2,
        velocity_stream_watchdog_action="telemetry",
        fast_gate_group_degrade_enabled=True,
    )
    components["safety"] = SafetyManager(components["config"].safety, components["fleet"])
    components["scheduler"] = StopAfterPlanScheduler()

    app = RealMissionApp(components)
    components["scheduler"].app = app
    components["fsm"]._state = MissionState.SETTLE

    app.run()

    assert components["fsm"].state() != MissionState.ABORT
    assert components["scheduler"].parked_history == [[5, 6]]
    assert any(
        event["event"] == "fast_gate_group_degrade"
        for event in components["telemetry"].events
    )
    assert any(
        event["event"] == "follower_hold_execution"
        and event["details"]["radio_groups"][2]["successes"] == [5, 6]
        for event in components["telemetry"].events
    )


def test_hold_does_not_recover_without_new_pose():
    repeated_snapshot = make_snapshot(1)
    components = build_components(
        [repeated_snapshot, repeated_snapshot, repeated_snapshot],
        [SafetyDecision("EXECUTE", [])],
    )
    components["frame_estimator"] = InvalidFrameEstimator()
    components["safety"] = FrameValiditySafety()
    components["scheduler"] = StopAfterPlanScheduler()

    app = RealMissionApp(components)
    components["scheduler"].app = app
    components["fsm"]._state = MissionState.SETTLE

    hold_call_count = {"count": 0}
    original_execute_hold = components["follower_executor"].execute_hold

    def stop_after_second_hold(actions):
        hold_call_count["count"] += 1
        result = original_execute_hold(actions)
        if hold_call_count["count"] >= 2:
            app._running = False
        return result

    components["follower_executor"].execute_hold = stop_after_second_hold

    app.run()

    assert components["fsm"].state() == MissionState.HOLD
    assert hold_call_count["count"] >= 2
    assert components["scheduler"].calls == []
    assert not any(
        event["event"] == "hold_recovered" for event in components["telemetry"].events
    )


def test_successful_parked_hold_clears_degraded_followers():
    components = build_components(
        [make_snapshot(1), make_snapshot(2)],
        [SafetyDecision("EXECUTE", []), SafetyDecision("ABORT", ["stop"])],
    )
    app = RealMissionApp(components)
    app.failure_policy.watchdog_degraded_followers.add(5)
    components["fsm"]._state = MissionState.SETTLE

    original_execute_hold = components["follower_executor"].execute_hold

    def stop_after_first_hold(actions):
        result = original_execute_hold(actions)
        app._running = False
        return result

    components["follower_executor"].execute_hold = stop_after_first_hold

    app.run()

    assert 5 not in app.failure_policy.watchdog_degraded_followers
    assert any(
        event["event"] == "watchdog_degrade_recovered"
        for event in components["telemetry"].events
    )
