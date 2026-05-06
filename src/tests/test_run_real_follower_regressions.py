import numpy as np

from src.app.run_real import RealMissionApp
from src.config.schema import SafetyConfig
from src.runtime.command_plan import FollowerAction, HoldAction
from src.runtime.follower_controller import FollowerCommandSet
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


class FullStateSendOnceScheduler:
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
        if self.app is not None:
            self.app._running = False
        return type(
            "Plan",
            (),
            {
                "leader_actions": [],
                "follower_actions": [
                    FollowerAction(
                        kind="full_state",
                        drone_id=5,
                        velocity=commands.commands[5],
                        position=commands.target_positions[5],
                        acceleration=commands.target_accelerations[5],
                    )
                ],
                "hold_actions": [],
                "diagnostics": {
                    "reason": "execute",
                    "follower_tx_groups_sent": [2],
                    "follower_tx_groups_blocked": [],
                    "follower_tx_groups_stale": [],
                },
            },
        )()


class FullStateControllerProbe:
    def __init__(self):
        self.commit_calls = []
        self.reset_calls = []

    def compute(self, snapshot, follower_ref, follower_ids, fleet):
        return FollowerCommandSet(
            commands={5: np.array([0.1, 0.0, 0.0])},
            diagnostics={"command_norms": {5: 0.1}},
            target_positions={5: np.array([0.0, 0.0, 1.0])},
            target_accelerations={5: np.zeros(3)},
            full_state_state={
                5: {
                    "target_position": np.array([0.0, 0.0, 1.0]),
                    "target_velocity": np.array([0.1, 0.0, 0.0]),
                    "t_meas": snapshot.t_meas,
                }
            },
        )

    def commit_full_state_state(self, commands, follower_ids=None):
        self.commit_calls.append((commands, sorted(follower_ids or [])))

    def reset_full_state_state(self, follower_ids=None):
        self.reset_calls.append(None if follower_ids is None else sorted(follower_ids))


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
        health_window=None,
        pose_window=None,
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
        fast_gate_group_degrade_streak=1,
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


def test_partial_disconnect_group_degrade_waits_for_streak_threshold():
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
        fast_gate_group_degrade_streak=4,
    )
    components["safety"] = SafetyManager(components["config"].safety, components["fleet"])
    components["scheduler"] = StopAfterPlanScheduler()

    app = RealMissionApp(components)
    components["scheduler"].app = app
    components["fsm"]._state = MissionState.SETTLE

    app.run()

    assert components["scheduler"].parked_history == [[]]
    assert any(
        event["event"] == "fast_gate_group_degrade_pending"
        for event in components["telemetry"].events
    )
    assert not any(
        event["event"] == "fast_gate_group_degrade"
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


def test_successful_full_state_send_commits_controller_state_and_records_timing():
    components = build_components(
        [make_snapshot(1)],
        [SafetyDecision("EXECUTE", [])],
    )
    controller = FullStateControllerProbe()
    scheduler = FullStateSendOnceScheduler()
    components["follower_controller"] = controller
    components["scheduler"] = scheduler
    app = RealMissionApp(components)
    scheduler.app = app
    components["fsm"]._state = MissionState.SETTLE

    app.run()

    assert controller.commit_calls
    assert controller.commit_calls[0][1] == [5]
    stream_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "streaming_setpoint_active"
    )
    assert stream_event["details"]["execute_duration_s"] >= 0.0
    assert stream_event["details"]["sent_groups"] == [2]
    assert stream_event["details"]["blocked_groups"] == []
    assert stream_event["details"]["stale_groups"] == []


def test_hold_resets_full_state_controller_state():
    components = build_components(
        [make_snapshot(1), make_snapshot(2)],
        [SafetyDecision("HOLD", ["pause"]), SafetyDecision("ABORT", ["stop"])],
    )
    controller = FullStateControllerProbe()
    components["follower_controller"] = controller
    app = RealMissionApp(components)
    components["fsm"]._state = MissionState.SETTLE

    app.run()

    assert controller.reset_calls
