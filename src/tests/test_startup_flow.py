"""Startup flow quasi-system tests"""

from src.app.run_real import RealMissionApp
from src.app.mission_errors import MissionErrors
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.tests.test_run_real import (
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


def build_startup_components(connect_fail=False, preflight_ok=True, fresh=True, z=0.8):
    fake_config = type(
        "FakeConfig",
        (),
        {
            "startup": type("FakeStartup", (), {"mode": "auto", "manual": None})(),
            "comm": type(
                "FakeComm",
                (),
                {
                    "readiness_wait_for_params": True,
                    "readiness_reset_estimator": True,
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
        "fleet": FakeFleet(),
        "link_manager": FakeLinkManager(should_fail=connect_fail),
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
assert any(
    event["event"] in {"wait_for_params", "reset_estimator", "fsm_transition"}
    for event in components["telemetry"].events
)
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "takeoff" for action in batch)
    for batch in components["leader_executor"].actions
)
assert len(components["follower_executor"].takeoff_calls) == 1

print("[OK] Startup flow quasi-system contracts verified")
