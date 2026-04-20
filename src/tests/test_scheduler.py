"""Scheduler focused tests"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.domain.stress_matrix_solver import StressMatrixSolver
from src.domain.afc_model import AFCModel
from src.domain.mission_profile import MissionProfile
from src.domain.leader_reference import LeaderReferenceGenerator
from src.domain.follower_reference import FollowerReferenceGenerator
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.follower_controller import FollowerController, FollowerCommandSet
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.scheduler import CommandScheduler
from src.runtime.safety_manager import SafetyDecision


config = ConfigLoader.load("config")
config.mission.leader_motion.trajectory_enabled = False
config.control.gain_xy = 1.2
config.control.gain_z = 0.4
fleet = FleetModel(config.fleet)
nominal = np.array(config.mission.nominal_positions)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)
solver = StressMatrixSolver(formation)
afc = AFCModel(solver.solve_dense(fleet.leader_ids()), fleet)
mission_profile = MissionProfile(config.mission)
leader_gen = LeaderReferenceGenerator(mission_profile, formation, fleet)
follower_gen = FollowerReferenceGenerator(
    formation, afc, config.safety.max_condition_number
)
controller = FollowerController(config.control)
fsm = MissionFSM()
fsm.transition(MissionState.CONNECT)
fsm.transition(MissionState.POSE_READY)
fsm.transition(MissionState.PREFLIGHT)
fsm.transition(MissionState.TAKEOFF)
fsm.transition(MissionState.SETTLE)
fsm.transition(MissionState.RUN)
scheduler = CommandScheduler(config.comm, fsm, fleet)

snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)

leader_ref = leader_gen.reference_at(0.0)
follower_ref = follower_gen.compute(leader_ref.positions, 0.0)
commands = controller.compute(snapshot, follower_ref, fleet.follower_ids(), fleet)
safety = SafetyDecision(action="EXECUTE", reasons=[])

plan1 = scheduler.plan(snapshot, MissionState.RUN, leader_ref, commands, safety)
assert len(plan1.follower_actions) == len(fleet.follower_ids())
assert sorted(plan1.diagnostics["follower_tx_groups_sent"]) == [0, 1, 2]

plan2 = scheduler.plan(snapshot, MissionState.RUN, leader_ref, commands, safety)
assert len(plan2.follower_actions) == 0
assert plan2.diagnostics is not None
assert plan2.diagnostics["reason"] == "deadband_blocked"

snapshot_new = PoseSnapshot(
    seq=2,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
scheduler.last_follower_tx_time = {}
scheduler.last_pose_seq_by_group = {group_id: -1 for group_id in [0, 1, 2]}
plan_deadband = scheduler.plan(
    snapshot_new, MissionState.RUN, leader_ref, commands, safety
)
assert len(plan_deadband.follower_actions) == 0
assert plan_deadband.diagnostics is not None
assert plan_deadband.diagnostics["reason"] == "deadband_blocked"

commands_changed = FollowerCommandSet(
    commands={
        drone_id: vel + np.array([0.1, 0.0, 0.0])
        for drone_id, vel in commands.commands.items()
    },
    diagnostics={"feedforward_followers": fleet.follower_ids()},
)
snapshot_newer = PoseSnapshot(
    seq=3,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
scheduler.last_follower_tx_time = {}
scheduler.last_pose_seq_by_group = {group_id: -1 for group_id in [0, 1, 2]}
plan_changed = scheduler.plan(
    snapshot_newer, MissionState.RUN, leader_ref, commands_changed, safety
)
assert len(plan_changed.follower_actions) == len(fleet.follower_ids())
assert sorted(plan_changed.diagnostics["follower_tx_groups_sent"]) == [0, 1, 2]

full_state_scheduler = CommandScheduler(config.comm, fsm, fleet)
follower_group_ids = sorted(
    {fleet.get_radio_group(drone_id) for drone_id in fleet.follower_ids()}
)
full_state_commands_a = FollowerCommandSet(
    commands={drone_id: np.zeros(3, dtype=float) for drone_id in fleet.follower_ids()},
    diagnostics={"output_mode": "full_state"},
    target_positions={
        drone_id: nominal[fleet.id_to_index(drone_id)].copy()
        for drone_id in fleet.follower_ids()
    },
    target_accelerations={
        drone_id: np.zeros(3, dtype=float) for drone_id in fleet.follower_ids()
    },
)
full_state_plan_a = full_state_scheduler.plan(
    snapshot_newer,
    MissionState.RUN,
    leader_ref,
    full_state_commands_a,
    safety,
)
assert len(full_state_plan_a.follower_actions) == len(fleet.follower_ids())
assert all(action.kind == "full_state" for action in full_state_plan_a.follower_actions)

full_state_commands_b = FollowerCommandSet(
    commands={drone_id: np.zeros(3, dtype=float) for drone_id in fleet.follower_ids()},
    diagnostics={"output_mode": "full_state"},
    target_positions={
        drone_id: nominal[fleet.id_to_index(drone_id)] + np.array([0.2, 0.0, 0.0])
        for drone_id in fleet.follower_ids()
    },
    target_accelerations={
        drone_id: np.zeros(3, dtype=float) for drone_id in fleet.follower_ids()
    },
)
full_state_scheduler.last_follower_tx_time = {}
full_state_scheduler.last_pose_seq_by_group = {
    group_id: -1 for group_id in follower_group_ids
}
full_state_plan_b = full_state_scheduler.plan(
    PoseSnapshot(
        seq=4,
        t_meas=0.0,
        positions=nominal,
        fresh_mask=np.ones(len(nominal), dtype=bool),
        disconnected_ids=[],
    ),
    MissionState.RUN,
    leader_ref,
    full_state_commands_b,
    safety,
)
assert len(full_state_plan_b.follower_actions) == len(fleet.follower_ids())
assert all(action.kind == "full_state" for action in full_state_plan_b.follower_actions)
assert {
    action.drone_id: action.position.tolist() for action in full_state_plan_b.follower_actions
} == {
    drone_id: position.tolist()
    for drone_id, position in full_state_commands_b.target_positions.items()
}

fsm.transition(MissionState.HOLD)
plan3 = scheduler.plan(
    snapshot,
    MissionState.HOLD,
    leader_ref,
    commands,
    safety,
    parked_follower_ids=fleet.follower_ids(),
)
assert len(plan3.follower_actions) == 0
assert len(plan3.hold_actions) == len(fleet.follower_ids())
assert sorted(plan3.diagnostics["hold_tx_groups_sent"]) == [0, 1, 2]

fsm.transition(MissionState.LAND)
scheduler.last_hold_tx_time = {}
plan4 = scheduler.plan(
    snapshot,
    MissionState.LAND,
    leader_ref,
    commands,
    safety,
    parked_follower_ids=fleet.follower_ids(),
)
assert len(plan4.follower_actions) == 0
assert len(plan4.hold_actions) == len(fleet.follower_ids())

fsm_group_mix = MissionFSM()
fsm_group_mix.transition(MissionState.CONNECT)
fsm_group_mix.transition(MissionState.POSE_READY)
fsm_group_mix.transition(MissionState.PREFLIGHT)
fsm_group_mix.transition(MissionState.TAKEOFF)
fsm_group_mix.transition(MissionState.SETTLE)
fsm_group_mix.transition(MissionState.RUN)
scheduler_group_mix = CommandScheduler(config.comm, fsm_group_mix, fleet)
scheduler_group_mix.last_follower_tx_time = {0: 0.0, 1: 0.0, 2: 0.0}
scheduler_group_mix.last_pose_seq_by_group = {0: -1, 1: -1, 2: -1}
scheduler_group_mix.last_hold_tx_time = {}
partial_commands = FollowerCommandSet(
    commands={5: np.array([0.1, 0.0, 0.0]), 6: np.array([0.1, 0.0, 0.0])},
    diagnostics={},
)
group2_only = scheduler_group_mix.plan(
    snapshot_newer,
    MissionState.RUN,
    leader_ref,
    partial_commands,
    safety,
    parked_follower_ids=[5],
)
assert [action.drone_id for action in group2_only.follower_actions] == [6]
assert [action.drone_id for action in group2_only.hold_actions] == [5]
assert group2_only.diagnostics["parked_group_counts"][1] == 1

abort_safety = SafetyDecision(action="ABORT", reasons=["test"])
plan5 = scheduler.plan(snapshot, MissionState.RUN, leader_ref, commands, abort_safety)
assert len(plan5.leader_actions) == 0
assert len(plan5.follower_actions) == 0

fsm_recover = MissionFSM()
fsm_recover.transition(MissionState.CONNECT)
fsm_recover.transition(MissionState.POSE_READY)
fsm_recover.transition(MissionState.PREFLIGHT)
fsm_recover.transition(MissionState.TAKEOFF)
fsm_recover.transition(MissionState.SETTLE)
fsm_recover.transition(MissionState.RUN)
fsm_recover.transition(MissionState.HOLD)
fsm_recover.transition(MissionState.RUN)
assert fsm_recover.state() == MissionState.RUN

print("[OK] Scheduler blocks follower resend on unchanged seq")
