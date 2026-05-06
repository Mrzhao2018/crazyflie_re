"""Leader trajectory start confirmation tests."""

import numpy as np

from src.app.run_real import RealMissionApp
from src.domain.mission_profile import TrajectoryPiece
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision
from src.tests.run_real_fixtures import build_components, make_snapshot


def _moving_piece(start_x: float) -> TrajectoryPiece:
    return TrajectoryPiece(
        duration=2.0,
        x=[start_x, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        y=[0.0] * 8,
        z=[0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        yaw=[0.0] * 8,
    )


def _static_piece(start_x: float) -> TrajectoryPiece:
    return TrajectoryPiece(
        duration=2.0,
        x=[start_x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        y=[0.0] * 8,
        z=[0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        yaw=[0.0] * 8,
    )


def _leader_ref():
    return type(
        "LeaderRef",
        (),
        {
            "leader_ids": [1, 2, 3, 4],
            "mode": "trajectory",
            "trajectory": {
                "trajectory_id": 1,
                "time_scale": 1.0,
                "relative_position": False,
                "relative_yaw": False,
                "reversed": False,
                "per_leader": {
                    1: {"pieces": [_moving_piece(1.0)], "time_scale": 1.0},
                    2: {"pieces": [_moving_piece(0.0)], "time_scale": 1.0},
                    3: {"pieces": [_moving_piece(-1.0)], "time_scale": 1.0},
                    4: {"pieces": [_static_piece(0.0)], "time_scale": 1.0},
                },
            },
            "positions": {},
        },
    )()


def _components():
    components = build_components(
        [make_snapshot(1), make_snapshot(2)],
        [SafetyDecision("EXECUTE", [])],
    )
    components["config"].safety.leader_trajectory_start_verify_delay_s = 0.0
    components["config"].safety.leader_trajectory_start_min_displacement_m = 0.05
    components["config"].safety.leader_trajectory_start_max_retries = 1
    return components


def _moved_snapshot() -> PoseSnapshot:
    snapshot = make_snapshot(2)
    positions = np.array(snapshot.positions, dtype=float)
    positions[0, 0] += 0.08
    positions[1, 0] += 0.08
    positions[2, 0] += 0.08
    return PoseSnapshot(
        seq=snapshot.seq,
        t_meas=snapshot.t_meas,
        positions=positions,
        fresh_mask=snapshot.fresh_mask,
        disconnected_ids=snapshot.disconnected_ids,
    )


def _snapshot_with_time(seq: int, t_meas: float) -> PoseSnapshot:
    snapshot = make_snapshot(seq)
    return PoseSnapshot(
        seq=snapshot.seq,
        t_meas=t_meas,
        positions=snapshot.positions,
        fresh_mask=snapshot.fresh_mask,
        disconnected_ids=snapshot.disconnected_ids,
    )


def test_leader_trajectory_start_retries_once_then_reports_failure():
    components = _components()
    app = RealMissionApp(components)
    leader_ref = _leader_ref()
    snapshot = make_snapshot(1)

    app._start_leader_trajectory(leader_ref, snapshot, mission_elapsed=6.0)
    assert app._check_leader_trajectory_start_motion(
        make_snapshot(2), mission_elapsed=6.1, leader_ref=leader_ref
    ) == "retried"
    assert app._check_leader_trajectory_start_motion(
        make_snapshot(3), mission_elapsed=6.2, leader_ref=leader_ref
    ) == "failed"

    start_calls = [
        action
        for batch in components["leader_executor"].actions
        for action in batch
        if action.kind == "start_trajectory"
    ]
    assert len(start_calls) == 2
    assert any(
        event["event"] == "leader_trajectory_start_failed"
        for event in components["telemetry"].events
    )


def test_leader_trajectory_start_waits_for_pose_time_after_main_loop_stall():
    components = _components()
    components["config"].safety.leader_trajectory_start_verify_delay_s = 1.0
    app = RealMissionApp(components)
    leader_ref = _leader_ref()

    app._start_leader_trajectory(
        leader_ref,
        _snapshot_with_time(seq=10, t_meas=100.0),
        mission_elapsed=6.0,
    )

    assert app._check_leader_trajectory_start_motion(
        _snapshot_with_time(seq=11, t_meas=100.2),
        mission_elapsed=10.0,
        leader_ref=leader_ref,
    ) == "pending"
    start_events = [
        event for event in components["telemetry"].events if event["event"] == "trajectory_start"
    ]
    assert len(start_events) == 1


def test_auto_trajectory_run_clock_starts_at_formation_run():
    components = _components()

    class _MissionProfile:
        def trajectory_start_time(self):
            return 6.0

    components["mission_profile"] = _MissionProfile()
    app = RealMissionApp(components)

    assert app._run_elapsed_start_offset("auto") == 6.0


def test_leader_trajectory_start_confirms_when_moving_leaders_move():
    components = _components()
    app = RealMissionApp(components)
    leader_ref = _leader_ref()

    app._start_leader_trajectory(leader_ref, make_snapshot(1), mission_elapsed=6.0)
    assert app._check_leader_trajectory_start_motion(
        _moved_snapshot(), mission_elapsed=6.1, leader_ref=leader_ref
    ) == "confirmed"

    assert len(components["leader_executor"].actions) == 1
    confirm_event = next(
        event
        for event in components["telemetry"].events
        if event["event"] == "leader_trajectory_motion_confirmed"
    )
    assert confirm_event["details"]["moving_leader_ids"] == [1, 2, 3]
