"""RealMissionApp orchestration contract tests with fakes"""

import numpy as np
from pathlib import Path

from src.app.run_real import RealMissionApp
from src.tests.run_real_fixtures import (
    FakeFleet,
    FakeFollowerController,
    FakeFollowerExecutor,
    FakeFollowerRefGen,
    FakeFrameEstimator,
    FakeKeyReader,
    FakeLeaderExecutor,
    FakeLeaderRefGen,
    FakeLinkManager,
    FakeManualInput,
    FakePoseBus,
    FakePoseSource,
    FakePreflight,
    FakeScheduler,
    FakeSafety,
    FakeTelemetry,
    FakeTransport,
    build_components,
    make_snapshot,
)
from src.runtime.manual_input_port import ManualLeaderIntent
from src.runtime.manual_leader_reference import ManualLeaderReferenceSource
from src.runtime.manual_leader_state import ManualLeaderState
from src.adapters.manual_input_keyboard import KeyboardManualInputSource
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision
from src.runtime.follower_controller import FollowerCommandSet
from src.app.mission_errors import MissionErrors


# start() connect failure -> ABORT
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], connect_fail=True
)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
connect_error = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "mission_error"
)
assert connect_error["details"]["category"] == MissionErrors.Connection.CONNECT_ALL_FAILED.category
assert connect_error["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_FAILED.code
assert connect_error["details"]["stage"] == MissionErrors.Connection.CONNECT_ALL_FAILED.stage
assert connect_error["details"]["connect_outcome"] == "failed"
assert connect_error["details"]["failed_group_ids"] == [0]
connect_group_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_result"
)
assert connect_group_event["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_FAILED.code


# start() preflight failure -> ABORT
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], preflight_ok=False
)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
preflight_error = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "mission_error"
)
assert preflight_error["details"]["category"] == MissionErrors.Readiness.PREFLIGHT_FAILED.category
assert preflight_error["details"]["code"] == MissionErrors.Readiness.PREFLIGHT_FAILED.code
assert preflight_error["details"]["stage"] == MissionErrors.Readiness.PREFLIGHT_FAILED.stage
assert preflight_error["details"]["failed_codes"] == ["BAD"]


# reconnect success reattaches console tap once per drone
components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
components["config"].comm.reconnect_attempts = 1
components["config"].comm.reconnect_backoff_s = 0.0
components["config"].comm.reconnect_timeout_s = 0.1
components["console_tap"] = type(
    "RecordingConsoleTap",
    (),
    {
        "__init__": lambda self: setattr(self, "calls", []),
        "reattach_drone": lambda self, drone_id: self.calls.append(drone_id),
    },
)()

def reconnect_ok(drone_id, *, attempts, backoff_s, timeout_s):
    return {
        "ok": True,
        "drone_id": drone_id,
        "attempt_count": 1,
        "radio_group": components["fleet"].get_radio_group(drone_id),
    }


components["link_manager"].reconnect = reconnect_ok
app = RealMissionApp(components)
assert app.failure_policy.attempt_reconnect([1]) is True
assert components["console_tap"].calls == [1]


# start() success drives startup stages and telemetry open
components = build_components(
    [make_snapshot(1), make_snapshot(1)], [SafetyDecision("EXECUTE", [])]
)
app = RealMissionApp(components)
assert app.start() is True
assert components["fsm"].state() == MissionState.SETTLE
assert components["pose_source"].started is True
assert components["telemetry"].opened is not None
assert Path(components["telemetry"].opened).name.startswith("run_real_")
assert Path(components["telemetry"].opened).suffix == ".jsonl"
assert components["telemetry_path"] == components["telemetry"].opened
assert components["telemetry"].header is not None
assert components["telemetry"].header["config_fingerprint"]["startup_mode"] == "auto"
assert components["telemetry"].header["config_fingerprint"]["leader_count"] == 4
assert components["telemetry"].header["fleet"]["drone_count"] == 6
assert components["telemetry"].header["fleet"]["leader_ids"] == [1, 2, 3, 4]
assert components["telemetry"].header["fleet"]["follower_ids"] == [5, 6]
assert components["transport"].wait_calls == components["fleet"].all_ids()
assert components["transport"].reset_calls == components["fleet"].all_ids()
assert any(event["event"] == "startup_mode" for event in components["telemetry"].events)
assert any(event["event"] == "connect_group_start" for event in components["telemetry"].events)
assert any(event["event"] == "connect_group_result" for event in components["telemetry"].events)
connect_all_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_all"
)
assert connect_all_event["details"]["ok"] is True
assert connect_all_event["details"]["category"] == MissionErrors.Connection.CONNECT_ALL_OK.category
assert connect_all_event["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_OK.code
assert connect_all_event["details"]["stage"] == MissionErrors.Connection.CONNECT_ALL_OK.stage
assert connect_all_event["details"]["outcome"] == "success"
assert connect_all_event["details"]["radio_groups"][1]["connected"] == [3, 4]
assert any(
    event["event"] == "config_fingerprint"
    and event["details"]["fingerprint"]["startup_mode"] == "auto"
    for event in components["telemetry"].events
)
assert any(event["event"] == "health_ready" for event in components["telemetry"].events)
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "takeoff" for action in batch)
    for batch in components["leader_executor"].actions
)
assert any(
    event["event"]
    in {"trajectory_prepare", "formation_align", "trajectory_entry_align"}
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "trajectory_entry_align"
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "trajectory_budget_check"
    for event in components["telemetry"].events
)
budget_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "trajectory_budget_check"
)
assert budget_event["details"]["fits_memory"] is True
assert any(
    event["event"] == "trajectory_readiness_summary"
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "trajectory_state" and event["details"]["state"] == "ready"
    for event in components["telemetry"].events
)
readiness_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "trajectory_readiness_summary"
)
assert readiness_event["details"]["ok"] is True
assert set(readiness_event["details"]["leaders"].keys()) == {1, 2, 3, 4}
assert not any(
    event["event"] == "trajectory_start" for event in components["telemetry"].events
)
assert len(components["follower_executor"].takeoff_calls) == 1
assert any(
    event["event"] == "follower_takeoff_execution" and event["details"]["ok"] is True
    for event in components["telemetry"].events
)


# run() HOLD path executes hold and does not velocity-execute
components = build_components(
    [make_snapshot(1), make_snapshot(1)],
    [SafetyDecision("HOLD", ["frame"]), SafetyDecision("HOLD", ["frame"])],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app._running = True


def stop_after_hold(actions):
    components["follower_executor"].hold_calls.append(actions)
    app._running = False
    return {
        "kind": "hold",
        "successes": [action.drone_id for action in actions],
        "failures": [],
    }


components["follower_executor"].execute_hold = stop_after_hold
app.run()
assert len(components["follower_executor"].hold_calls) >= 1
assert len(components["follower_executor"].velocity_calls) == 0
assert components["fsm"].state() == MissionState.HOLD


# degraded follower filtering preserves full_state references for remaining followers
class CapturingScheduler:
    def __init__(self):
        self.commands_seen = None
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
        self.commands_seen = commands
        if self.app is not None:
            self.app._running = False
        return type(
            "Plan",
            (),
            {
                "leader_actions": [],
                "follower_actions": [],
                "hold_actions": [],
                "diagnostics": {"reason": "captured"},
            },
        )()


class FullStateFollowerController:
    def compute(self, snapshot, follower_ref, follower_ids, fleet):
        return FollowerCommandSet(
            commands={fid: np.zeros(3, dtype=float) for fid in follower_ids},
            diagnostics={"output_mode": "full_state"},
            target_positions={
                5: np.array([0.0, 0.0, 1.0]),
                6: np.array([0.5, 0.5, 1.0]),
            },
            target_accelerations={
                5: np.array([0.0, 0.0, 0.0]),
                6: np.array([0.1, 0.0, 0.0]),
            },
        )


components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
components["scheduler"] = CapturingScheduler()
components["follower_controller"] = FullStateFollowerController()
app = RealMissionApp(components)
components["scheduler"].app = app
app.failure_policy.watchdog_degraded_followers.add(5)
components["fsm"]._state = MissionState.SETTLE
app.run()
captured_commands = components["scheduler"].commands_seen
assert captured_commands is not None
assert set(captured_commands.commands.keys()) == {6}
assert set(captured_commands.target_positions.keys()) == {6}
assert set(captured_commands.target_accelerations.keys()) == {6}
assert captured_commands.target_positions[6].tolist() == [0.5, 0.5, 1.0]
assert captured_commands.target_accelerations[6].tolist() == [0.1, 0.0, 0.0]


# executor summaries aggregate failures into radio-group buckets
components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("ABORT", ["stop"])],
)
app = RealMissionApp(components)
app.telemetry_reporter.record_executor_summary(
    "leader_execution",
    [
        {
            "kind": "batch_goto",
            "successes": [1, 2],
            "failures": [
                {
                    "drone_id": 3,
                    "radio_group": 1,
                    "command_kind": "batch_goto",
                    "failure_category": "timeout",
                    "retryable": True,
                    "reason": "leader_link_loss",
                },
                {
                    "drone_id": 4,
                    "radio_group": 1,
                    "command_kind": "batch_goto",
                    "failure_category": "transport_runtime",
                    "retryable": False,
                    "reason": "leader_busy",
                },
            ],
        }
    ],
)
app.telemetry_reporter.record_executor_summary(
    "follower_velocity_execution",
    [
        {
            "kind": "velocity",
            "successes": [5],
            "failures": [
                {
                    "drone_id": 6,
                    "radio_group": 2,
                    "command_kind": "velocity",
                    "failure_category": "timeout",
                    "retryable": True,
                    "reason": "follower_timeout",
                }
            ],
        }
    ],
)
leader_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "leader_execution"
)
assert leader_event["details"]["ok"] is False
assert leader_event["details"]["radio_groups"][0]["successes"] == [1, 2]
assert [
    item["drone_id"] for item in leader_event["details"]["radio_groups"][1]["failures"]
] == [3, 4]
assert leader_event["details"]["radio_groups"][1]["failures"][0]["command_kind"] == "batch_goto"
assert leader_event["details"]["radio_groups"][1]["failures"][0]["failure_category"] == "timeout"
assert leader_event["details"]["radio_groups"][1]["failures"][1]["retryable"] is False
follower_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "follower_velocity_execution"
)
assert follower_event["details"]["ok"] is False
assert follower_event["details"]["radio_groups"][2]["successes"] == [5]
assert [
    item["drone_id"] for item in follower_event["details"]["radio_groups"][2]["failures"]
] == [6]
assert follower_event["details"]["radio_groups"][2]["failures"][0]["command_kind"] == "velocity"
assert follower_event["details"]["radio_groups"][2]["failures"][0]["retryable"] is True


# connect_all failure exposes group-level partial results
components = build_components(
    [make_snapshot(1)],
    [SafetyDecision("EXECUTE", [])],
    failed_connect_drones=[3],
)
app = RealMissionApp(components)
assert app.start() is False
connect_all_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_all"
)
assert connect_all_event["details"]["ok"] is False
assert connect_all_event["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_FAILED.code
assert connect_all_event["details"]["outcome"] == "partial_failure"
assert connect_all_event["details"]["failed_group_ids"] == [1]
assert connect_all_event["details"]["radio_groups"][0]["connected"] == [1, 2]
assert connect_all_event["details"]["radio_groups"][1]["failures"][0]["drone_id"] == 3
connect_error = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "mission_error"
)
assert connect_error["details"]["connect_outcome"] == "partial_failure"
assert connect_error["details"]["radio_groups"][1]["failures"][0]["group_id"] == 1


# run() ABORT path triggers emergency land semantics
components = build_components(
    [make_snapshot(1), make_snapshot(1)],
    [SafetyDecision("ABORT", ["disconnect"]), SafetyDecision("ABORT", ["disconnect"])],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert components["fsm"].state() == MissionState.ABORT
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1
abort_record = components["telemetry"].records[-1]
assert abort_record.safety_action == "ABORT"
assert abort_record.scheduler_reason == "emergency_land"
assert abort_record.trajectory_state == "terminated"
assert abort_record.trajectory_terminal_reason == "abort"
assert any(
    event["event"] == "emergency_land" for event in components["telemetry"].events
)


# no-new-pose should not recompute follower control twice
components = build_components(
    [make_snapshot(1), make_snapshot(1), make_snapshot(1)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert components["follower_controller"].calls == 1
assert len(components["telemetry"].records) >= 2
record = components["telemetry"].records[-2]
terminal_record = components["telemetry"].records[-1]
assert record.scheduler_reason == "fake_plan"
assert terminal_record.scheduler_reason == "emergency_land"
assert terminal_record.trajectory_state == "terminated"
assert terminal_record.trajectory_terminal_reason == "abort"
assert record.mission_state in {MissionState.RUN.value, MissionState.ABORT.value}
assert record.phase_label == "formation_run"
assert record.measured_positions[1] == [1.0, 0.0, 0.8]
assert record.fresh_mask[1] is True
assert record.disconnected_ids == []
assert record.leader_mode == "trajectory"
# run() was called directly without start(); header writing is start()'s job so it stays None here.
assert components["telemetry"].header is None
assert record.health
assert len(components["telemetry"].events) >= 1
assert record.startup_mode == "auto"
assert any(
    event["event"] in {"fsm_transition", "preflight", "trajectory_prepare"}
    for event in components["telemetry"].events
)
assert components["telemetry"].summary()["event_counts"]
assert components["telemetry"].phase_events()
if any(
    event["event"] == "trajectory_prepare" for event in components["telemetry"].events
):
    assert len(components["transport"].upload_calls) >= 1
    assert len(components["transport"].define_calls) >= 1

# run() can recover from HOLD when safety recovers
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3)],
    [
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "hold_recovered" for event in components["telemetry"].events
)
assert any(event["event"] == "hold_entered" for event in components["telemetry"].events)
assert any(
    event["event"] == "follower_hold_execution" for event in components["telemetry"].events
)
assert components["follower_controller"].calls >= 1


# sustained HOLD auto-lands after timeout
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3), make_snapshot(4)],
    [
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "hold_timeout_land" for event in components["telemetry"].events
)
timeout_record = components["telemetry"].records[-1]
assert timeout_record.safety_action == "HOLD_TIMEOUT"
assert timeout_record.scheduler_reason == "hold_timeout"
assert timeout_record.trajectory_terminal_reason == "hold_timeout"
assert len(components["follower_executor"].land_calls) == 1

# start() failure path cleans up resources
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], preflight_ok=False
)
app = RealMissionApp(components)
assert app.start() is False
assert components["telemetry"].closed is True
assert components["pose_source"].stopped is True


# start() fails if health is never ready
components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
components["health_bus"] = type(
    "NoHealthBus",
    (),
    {
        "latest": lambda self: {},
        "update": lambda self, drone_id, values, t_meas: None,
    },
)()
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
assert any(
    event["event"] == "health_ready" and event["details"]["ok"] is False
    for event in components["telemetry"].events
)
health_error = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "mission_error"
)
assert health_error["details"]["code"] == MissionErrors.Readiness.HEALTH_TIMEOUT.code
assert health_error["details"]["category"] == MissionErrors.Readiness.HEALTH_TIMEOUT.category
assert health_error["details"]["stage"] == MissionErrors.Readiness.HEALTH_TIMEOUT.stage


# run() unexpected exception is coded as runtime error and lands urgently
components = build_components(
    [make_snapshot(1), make_snapshot(2)],
    [SafetyDecision("EXECUTE", []), SafetyDecision("EXECUTE", [])],
)
components["follower_controller"] = type(
    "BrokenFollowerController",
    (),
    {
        "compute": lambda self, snapshot, follower_ref, follower_ids, fleet: (_ for _ in ()).throw(
            RuntimeError("controller exploded")
        )
    },
)()
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
runtime_error = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "mission_error"
)
assert runtime_error["details"]["category"] == MissionErrors.Runtime.RUN_LOOP_EXCEPTION.category
assert runtime_error["details"]["code"] == MissionErrors.Runtime.RUN_LOOP_EXCEPTION.code
assert runtime_error["details"]["stage"] == MissionErrors.Runtime.RUN_LOOP_EXCEPTION.stage
assert runtime_error["details"]["exception_type"] == "RuntimeError"
assert any(
    event["event"] == "emergency_land"
    and event["details"].get("trigger_code") == MissionErrors.Runtime.RUN_LOOP_EXCEPTION.code
    for event in components["telemetry"].events
)
assert components["fsm"].state() == MissionState.ABORT


# shutdown() from active mission performs symmetric landing and flushes telemetry
components = build_components(
    [make_snapshot(1), make_snapshot(1)], [SafetyDecision("EXECUTE", [])]
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.RUN
app.shutdown()
assert components["fsm"].state() == MissionState.LAND
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "land" for action in batch)
    for batch in components["leader_executor"].actions
)
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1
assert components["telemetry"].closed is True
assert len(components["telemetry"].records) >= 1
shutdown_record = components["telemetry"].records[-1]
assert shutdown_record.safety_action == "SHUTDOWN"
assert shutdown_record.scheduler_reason == "shutdown"
assert shutdown_record.trajectory_state == "terminated"
assert shutdown_record.trajectory_terminal_reason == "shutdown"
assert any(
    event["event"] == "manual_shutdown_land" for event in components["telemetry"].events
)
assert any(
    event["event"] == "follower_land_execution" for event in components["telemetry"].events
)
assert any(event["event"] == "shutdown" for event in components["telemetry"].events)


# auto mode ends by mission duration and lands automatically
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(25)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "trajectory_start" for event in components["telemetry"].events
)
assert any(
    event["event"] == "mission_complete" for event in components["telemetry"].events
)
assert any(
    event["event"] == "mission_complete_land"
    for event in components["telemetry"].events
)
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1
mission_complete_record = components["telemetry"].records[-1]
assert mission_complete_record.safety_action == "MISSION_COMPLETE"
assert mission_complete_record.scheduler_reason == "mission_complete"
assert mission_complete_record.trajectory_state == "terminated"
assert mission_complete_record.trajectory_terminal_reason == "mission_complete"


# manual startup mode skips auto-only leader startup behavior
manual_state = ManualLeaderState(default_axis="z")
manual_input = FakeManualInput(
    [
        ManualLeaderIntent(axis_switch=True),
        ManualLeaderIntent(target_switch=True),
        ManualLeaderIntent(translation_delta=(0.2, 0.0, 0.0)),
        ManualLeaderIntent(rotation_delta_deg=5.0),
    ]
)
manual_source = ManualLeaderReferenceSource(
    type(
        "Formation",
        (),
        {
            "nominal_position": lambda self, drone_id: np.array(
                [float(drone_id), 0.0, 0.5]
            )
        },
    )(),
    FakeFleet(),
    manual_state,
)
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
    startup_mode="manual_leader",
    leader_ref_mode="batch_goto",
    manual_input=manual_input,
    manual_state=manual_state,
)
components["leader_ref_gen"] = manual_source
app = RealMissionApp(components)
assert app.start() is True
assert not any(
    event["event"] == "trajectory_prepare" for event in components["telemetry"].events
)
assert not any(
    event["event"] == "formation_align" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_structure_align"
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_mode_armed" for event in components["telemetry"].events
)
app.run()
assert manual_input.started is True
assert any(
    event["event"] == "manual_input_started" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_intent" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_axis_switch" for event in components["telemetry"].events
)
manual_record = components["telemetry"].records[-1]
assert manual_record.startup_mode == "manual_leader"
assert manual_record.leader_reference_source == "ManualLeaderReferenceSource"
assert manual_record.manual_axis in {"x", "y", "z"}
assert any(
    event["event"] == "manual_intent" for event in components["telemetry"].events
)


# keyboard input adapter maps keys into ManualLeaderIntent
reader = FakeKeyReader(["w", "q", "x", "c", "v", "?"])
keyboard_input = KeyboardManualInputSource(
    translation_step=0.2,
    vertical_step=0.15,
    scale_step=0.05,
    rotation_step_deg=7.0,
    key_reader=reader,
)
keyboard_input.start()
intent = keyboard_input.poll()
assert intent is not None and intent.translation_delta == (0.2, 0.0, 0.0)
intent = keyboard_input.poll()
assert intent is not None and intent.scale_delta == 0.05
intent = keyboard_input.poll()
assert intent is not None and intent.rotation_delta_deg == 7.0
intent = keyboard_input.poll()
assert intent is not None and intent.axis_switch is True
intent = keyboard_input.poll()
assert intent is not None and intent.target_switch is True
assert keyboard_input.poll() is None


# manual leader state supports target switching between swarm and individual leaders
state = ManualLeaderState(default_axis="z")
state.apply_intent(ManualLeaderIntent(target_switch=True))
snap = state.snapshot()
assert snap.target_mode == "leader"
assert snap.selected_leader_id == 1
state.apply_intent(ManualLeaderIntent(translation_delta=(0.3, 0.0, 0.0)))
snap = state.snapshot()
assert np.allclose(snap.per_leader_offsets[1], np.array([0.3, 0.0, 0.0]))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
snap = state.snapshot()
assert snap.target_mode == "swarm"
assert snap.selected_leader_id is None

print("[OK] RealMissionApp orchestration contracts verified")
