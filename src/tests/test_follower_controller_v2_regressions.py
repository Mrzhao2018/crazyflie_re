import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.follower_reference import FollowerReferenceSet
from src.runtime.follower_controller_v2 import FollowerControllerV2
from src.runtime.pose_snapshot import PoseSnapshot


def test_v2_clears_stale_follower_state_before_reentry():
    config = ConfigLoader.load("config")
    config.control.dynamics_model_order = 2
    config.control.feedforward_gain_xy = 1.0
    config.control.feedforward_gain_z = 1.0
    config.control.max_feedforward_velocity_xy = 1.0
    config.control.max_feedforward_velocity_z = 1.0
    config.control.radial_feedforward_scale_xy = 0.0
    config.control.radial_gain_scale_xy = 0.0

    fleet = FleetModel(config.fleet)
    controller = FollowerControllerV2(config.control)

    nominal = np.array(config.mission.nominal_positions, dtype=float)
    follower_ids = fleet.follower_ids()
    follower_targets = {
        fid: nominal[fleet.id_to_index(fid)].copy() for fid in follower_ids
    }
    with_feedforward = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=follower_targets,
        target_velocities={
            fid: np.array([0.2, 0.0, 0.0], dtype=float) for fid in follower_ids
        },
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )
    without_feedforward = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=follower_targets,
        target_velocities=None,
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )

    initial_snapshot = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )
    controller.compute(initial_snapshot, with_feedforward, follower_ids, fleet)

    stale_mask = np.ones(len(nominal), dtype=bool)
    stale_mask[fleet.id_to_index(5)] = False
    stale_snapshot = PoseSnapshot(
        seq=2,
        t_meas=0.5,
        positions=nominal.copy(),
        fresh_mask=stale_mask,
        disconnected_ids=[5],
    )
    controller.compute(stale_snapshot, without_feedforward, follower_ids, fleet)

    reentry_snapshot = PoseSnapshot(
        seq=3,
        t_meas=1.0,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )
    reentry_commands = controller.compute(
        reentry_snapshot, without_feedforward, follower_ids, fleet
    )

    assert np.linalg.norm(reentry_commands.commands[5]) < 1e-9


def test_full_state_derives_feedforward_from_smoothed_position_reference():
    config = ConfigLoader.load("config")
    config.control.output_mode = "full_state"
    config.control.full_state_position_smoothing_alpha = 0.45
    config.control.full_state_max_position_step = 0.04
    config.control.max_velocity = 0.65
    config.control.max_acceleration = 2.0

    fleet = FleetModel(config.fleet)
    controller = FollowerControllerV2(config.control)
    nominal = np.array(config.mission.nominal_positions, dtype=float)
    follower_ids = fleet.follower_ids()
    first_targets = {
        fid: nominal[fleet.id_to_index(fid)].copy() for fid in follower_ids
    }
    snapshot = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )
    first_ref = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=first_targets,
        target_velocities=None,
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )
    first_result = controller.compute(snapshot, first_ref, follower_ids, fleet)
    controller.commit_full_state_state(first_result, follower_ids)

    fid = follower_ids[0]
    jumped_targets = {key: value.copy() for key, value in first_targets.items()}
    jumped_targets[fid] = jumped_targets[fid] + np.array([10.0, 0.0, 0.0])
    jumped_ref = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=jumped_targets,
        target_velocities={fid: np.array([10.0, 0.0, 0.0])},
        target_accelerations={fid: np.array([0.0, 0.0, 10.0])},
        frame_condition_number=float("nan"),
        valid=True,
    )
    next_snapshot = PoseSnapshot(
        seq=2,
        t_meas=0.1,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )
    result = controller.compute(next_snapshot, jumped_ref, follower_ids, fleet)

    delta = result.target_positions[fid] - first_targets[fid]
    assert np.isclose(np.linalg.norm(delta), 0.04)
    np.testing.assert_allclose(result.commands[fid], np.array([0.4, 0.0, 0.0]))
    np.testing.assert_allclose(result.target_accelerations[fid], np.array([2.0, 0.0, 0.0]))
    assert result.diagnostics["derived_feedforward_followers"] == [fid]
    assert result.diagnostics["raw_feedforward_ignored_followers"] == [fid]
    assert result.diagnostics["raw_acceleration_ignored_followers"] == [fid]


def test_full_state_preview_does_not_advance_until_committed():
    config = ConfigLoader.load("config")
    config.control.output_mode = "full_state"
    config.control.full_state_position_smoothing_alpha = 1.0
    config.control.full_state_max_position_step = 0.04
    config.control.max_velocity = 0.65
    config.control.max_acceleration = 2.0

    fleet = FleetModel(config.fleet)
    controller = FollowerControllerV2(config.control)
    nominal = np.array(config.mission.nominal_positions, dtype=float)
    follower_ids = fleet.follower_ids()
    fid = follower_ids[0]
    first_targets = {
        fid: nominal[fleet.id_to_index(fid)].copy() for fid in follower_ids
    }
    snapshot = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )
    first_ref = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=first_targets,
        target_velocities=None,
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )

    uncommitted_seed = controller.compute(snapshot, first_ref, follower_ids, fleet)

    jumped_targets = {key: value.copy() for key, value in first_targets.items()}
    jumped_targets[fid] = jumped_targets[fid] + np.array([10.0, 0.0, 0.0])
    jumped_ref = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions=jumped_targets,
        target_velocities=None,
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )
    next_snapshot = PoseSnapshot(
        seq=2,
        t_meas=0.1,
        positions=nominal.copy(),
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )

    preview_without_commit = controller.compute(
        next_snapshot, jumped_ref, follower_ids, fleet
    )
    np.testing.assert_allclose(preview_without_commit.commands[fid], np.zeros(3))

    controller.commit_full_state_state(uncommitted_seed, follower_ids)
    preview_after_commit = controller.compute(next_snapshot, jumped_ref, follower_ids, fleet)
    np.testing.assert_allclose(preview_after_commit.commands[fid], np.array([0.4, 0.0, 0.0]))


def test_full_state_first_reference_is_limited_from_current_position():
    config = ConfigLoader.load("config")
    config.control.output_mode = "full_state"
    config.control.full_state_position_smoothing_alpha = 0.45
    config.control.full_state_max_position_step = 0.04

    fleet = FleetModel(config.fleet)
    controller = FollowerControllerV2(config.control)
    nominal = np.array(config.mission.nominal_positions, dtype=float)
    follower_ids = fleet.follower_ids()
    fid = follower_ids[0]

    current_positions = nominal.copy()
    current_positions[fleet.id_to_index(fid)] = np.array([0.0, 0.0, 0.5])
    target = np.array([0.0, 0.0, 1.5])
    ref = FollowerReferenceSet(
        follower_ids=list(follower_ids),
        target_positions={fid: target},
        target_velocities=None,
        target_accelerations=None,
        frame_condition_number=float("nan"),
        valid=True,
    )
    snapshot = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=current_positions,
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    )

    result = controller.compute(snapshot, ref, [fid], fleet)

    initial_delta = (
        result.target_positions[fid] - current_positions[fleet.id_to_index(fid)]
    )
    assert np.isclose(np.linalg.norm(initial_delta), 0.04)
