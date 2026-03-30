"""Leader trajectory dispatch tests"""

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.domain.mission_profile import MissionProfile
from src.domain.leader_reference import LeaderReferenceGenerator
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.scheduler import CommandScheduler
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision
import numpy as np


config = ConfigLoader.load("config")
config.mission.leader_motion.trajectory_enabled = True
fleet = FleetModel(config.fleet)
formation = FormationModel(
    np.array(config.mission.nominal_positions), fleet.leader_ids(), fleet
)
mission_profile = MissionProfile(config.mission)
leader_gen = LeaderReferenceGenerator(mission_profile, formation, fleet)

fsm = MissionFSM()
fsm.transition(MissionState.CONNECT)
fsm.transition(MissionState.POSE_READY)
fsm.transition(MissionState.PREFLIGHT)
fsm.transition(MissionState.TAKEOFF)
fsm.transition(MissionState.SETTLE)
fsm.transition(MissionState.RUN)

scheduler = CommandScheduler(config.comm, fsm)
snapshot = PoseSnapshot(
    seq=1,
    t_meas=0.0,
    positions=np.array(config.mission.nominal_positions),
    fresh_mask=np.ones(len(config.mission.nominal_positions), dtype=bool),
    disconnected_ids=[],
)
leader_ref = leader_gen.reference_at(0.0)
plan = scheduler.plan(
    snapshot, MissionState.RUN, leader_ref, None, SafetyDecision("EXECUTE", [])
)

assert len(plan.leader_actions) == 1
assert plan.leader_actions[0].kind == "start_trajectory"

print("[OK] Leader trajectory dispatch verified")
