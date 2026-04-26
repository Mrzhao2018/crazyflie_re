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
