"""Startup flow quasi-system tests"""

from src.app.run_real import RealMissionApp
from src.app.mission_errors import MissionErrors
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.tests.run_real_fixtures import (
    FakeFleet,
    FakeLinkManager,
    FakeTransport,
    FakePoseSource,
    FakePoseBus,
    FakePreflight,
    FakeTelemetry,
    FakeLeaderExecutor,
    FakeFollowerExecutor,
    FakeFrameEstimator,
    FakeFollowerRefGen,
    FakeFollowerController,
    FakeSafety,
    FakeScheduler,
    FakeLeaderRefGen,
    make_snapshot,
)
from src.runtime.safety_manager import SafetyDecision


def build_startup_components(
    connect_fail=False,
    failed_connect_drones=None,
    preflight_ok=True,
    fresh=True,
    z=0.8,
    connect_groups_in_parallel=False,
    trajectory_upload_groups_in_parallel=False,
):
    fake_config = type(
        "FakeConfig",
        (),
        {
            "startup": type("FakeStartup", (), {"mode": "auto", "manual": None})(),
            "comm": type(
                "FakeComm",
                (),
                {
                    "follower_tx_freq": 8.0,
                    "readiness_wait_for_params": True,
                    "readiness_reset_estimator": True,
                    "connect_groups_in_parallel": connect_groups_in_parallel,
                    "trajectory_upload_groups_in_parallel": trajectory_upload_groups_in_parallel,
                },
            )(),
            "mission": type(
                "FakeMission",
                (),
                {
                    "duration": 20.0,
                    "leader_motion": type(
                        "FakeLeaderMotion", (), {"trajectory_enabled": True}
                    )(),
                },
            )(),
        },
    )()

    fleet = FakeFleet()

    fake_health_bus = type(
        "FakeHealthBus",
        (),
        {
            "latest": lambda self: {
                did: type("Sample", (), {"values": {"pm.vbat": 4.0}})()
                for did in [1, 2, 3, 4, 5, 6]
            },
            "update": lambda self, drone_id, values, t_meas: None,
        },
    )()

    return {
        "config": fake_config,
        "config_dir": "config",
        "repo_root": "repo",
        "startup_mode": "auto",
        "fsm": MissionFSM(),
        "fleet": fleet,
        "link_manager": FakeLinkManager(
            fleet,
            should_fail=connect_fail,
            failed_drones=failed_connect_drones,
        ),
        "transport": FakeTransport(),
        "pose_source": FakePoseSource(),
        "pose_bus": FakePoseBus([make_snapshot(1, fresh=fresh, z=z)] * 3),
        "preflight": FakePreflight(
            ok=preflight_ok, reasons=[] if preflight_ok else ["bad"]
        ),
        "telemetry": FakeTelemetry(),
        "health_bus": fake_health_bus,
        "leader_executor": FakeLeaderExecutor(),
        "follower_executor": FakeFollowerExecutor(),
        "frame_estimator": FakeFrameEstimator(),
        "follower_ref_gen": FakeFollowerRefGen(),
        "follower_controller": FakeFollowerController(),
        "safety": FakeSafety([SafetyDecision("EXECUTE", [])]),
        "scheduler": FakeScheduler(),
        "leader_ref_gen": FakeLeaderRefGen(),
    }


components = build_startup_components(connect_fail=True)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
assert any(
    event["event"] == "mission_error"
    and event["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_FAILED.code
    for event in components["telemetry"].events
)
connect_group_start_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_start"
)
assert connect_group_start_event["details"]["category"] == MissionErrors.Connection.CONNECT_GROUP_START.category
assert connect_group_start_event["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_START.code
assert connect_group_start_event["details"]["outcome"] == "start"
connect_group_result_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_result"
)
assert connect_group_result_event["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_FAILED.code
assert connect_group_result_event["details"]["outcome"] == "failed"
assert connect_group_result_event["details"]["failure_drone_ids"] == [1]

components = build_startup_components(failed_connect_drones=[3])
app = RealMissionApp(components)
assert app.start() is False
connect_group_events = [
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_result"
]
assert connect_group_events[0]["details"]["group_id"] == 0
assert connect_group_events[0]["details"]["ok"] is True
assert connect_group_events[0]["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_SUCCESS.code
assert connect_group_events[1]["details"]["group_id"] == 1
assert connect_group_events[1]["details"]["ok"] is False
assert connect_group_events[1]["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_FAILED.code
assert connect_group_events[1]["details"]["outcome"] == "failed"
assert connect_group_events[1]["details"]["failures"][0]["drone_id"] == 3
connect_all_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_all"
)
assert connect_all_event["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_FAILED.code
assert connect_all_event["details"]["outcome"] == "partial_failure"
assert connect_all_event["details"]["failed_group_ids"] == [1]
assert connect_all_event["details"]["radio_groups"][0]["connected"] == [1, 2]
assert connect_all_event["details"]["radio_groups"][1]["failures"][0]["drone_id"] == 3

components = build_startup_components(failed_connect_drones=[4])
app = RealMissionApp(components)
assert app.start() is False
partial_group_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_result"
    and event["details"]["group_id"] == 1
)
assert partial_group_event["details"]["code"] == MissionErrors.Connection.CONNECT_GROUP_PARTIAL_FAILURE.code
assert partial_group_event["details"]["outcome"] == "partial_failure"
assert partial_group_event["details"]["connected"] == [3]
assert partial_group_event["details"]["failure_drone_ids"] == [4]

components = build_startup_components(preflight_ok=False)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
assert any(
    event["event"] == "mission_error"
    and event["details"]["code"] == MissionErrors.Readiness.PREFLIGHT_FAILED.code
    for event in components["telemetry"].events
)

components = build_startup_components(z=0.1)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
assert any(
    event["event"] == "mission_error"
    and event["details"]["code"] == MissionErrors.Readiness.TAKEOFF_VALIDATION_FAILED.code
    for event in components["telemetry"].events
)

components = build_startup_components()
app = RealMissionApp(components)
assert app.start() is True
assert components["fsm"].state() == MissionState.SETTLE
assert components["telemetry"].opened is not None
assert components["transport"].wait_calls == components["fleet"].all_ids()
assert components["transport"].reset_calls == components["fleet"].all_ids()
assert any(event["event"] == "startup_mode" for event in components["telemetry"].events)
assert any(event["event"] == "config_fingerprint" for event in components["telemetry"].events)
connect_group_events = [
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_group_result"
]
assert [event["details"]["group_id"] for event in connect_group_events] == [0, 1, 2]
assert all(event["details"]["ok"] is True for event in connect_group_events)
connect_all_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "connect_all"
)
assert connect_all_event["details"]["ok"] is True
assert connect_all_event["details"]["code"] == MissionErrors.Connection.CONNECT_ALL_OK.code
assert connect_all_event["details"]["outcome"] == "success"
assert connect_all_event["details"]["radio_groups"][2]["connected"] == [5, 6]
assert any(
    event["event"] in {"wait_for_params", "reset_estimator", "fsm_transition"}
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "follower_takeoff_execution" and event["details"]["ok"] is True
    for event in components["telemetry"].events
)
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "takeoff" for action in batch)
    for batch in components["leader_executor"].actions
)
assert len(components["follower_executor"].takeoff_calls) == 1

components = build_startup_components(
    connect_groups_in_parallel=True,
    trajectory_upload_groups_in_parallel=True,
)
app = RealMissionApp(components)
assert app.start() is True
assert len(components["transport"].upload_calls) == 4
assert len(components["transport"].define_calls) == 4

print("[OK] Startup flow quasi-system contracts verified")
