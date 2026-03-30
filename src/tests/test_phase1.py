"""测试第一阶段代码"""

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.domain.formation_model import FormationModel
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.pose_bus import PoseBus
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.safety_manager import SafetyManager
from src.runtime.scheduler import CommandScheduler
import numpy as np

print("=== 测试配置加载 ===")
config = ConfigLoader.load("config")
print(f"[OK] 加载了 {len(config.fleet.drones)} 架无人机")

print("\n=== 测试FleetModel ===")
fleet = FleetModel(config.fleet)
print(f"[OK] Leaders: {fleet.leader_ids()}")
print(f"[OK] Followers: {fleet.follower_ids()}")
print(f"[OK] ID 1 -> Index {fleet.id_to_index(1)}")

print("\n=== 测试FormationModel ===")
nominal = np.array(config.mission.nominal_positions)
formation = FormationModel(nominal, fleet.leader_ids(), fleet)
report = formation.check_affine_span(fleet.leader_ids())
print(f"[OK] Affine span valid: {report.valid}, rank: {report.rank}")

print("\n=== 测试PoseSnapshot + PoseBus ===")
pose_bus = PoseBus(fleet, config.safety.pose_timeout)
snapshot = PoseSnapshot(
    seq=0,
    t_meas=0.0,
    positions=nominal,
    fresh_mask=np.ones(len(nominal), dtype=bool),
    disconnected_ids=[],
)
for drone_id in fleet.all_ids():
    pose_bus.update_agent(drone_id, nominal[fleet.id_to_index(drone_id)], 0.0)
latest = pose_bus.latest()
assert latest is not None
print(f"[OK] Pose seq: {latest.seq}")

print("\n=== 测试MissionFSM ===")
fsm = MissionFSM()
print(f"[OK] 初始状态: {fsm.state()}")
fsm.transition(MissionState.CONNECT)
fsm.transition(MissionState.POSE_READY)
fsm.transition(MissionState.PREFLIGHT)
fsm.transition(MissionState.TAKEOFF)
fsm.transition(MissionState.SETTLE)
fsm.transition(MissionState.RUN)
policy = fsm.allowed_command_policy()
print(f"[OK] RUN状态策略: leader={policy.leader_mode}, follower={policy.follower_mode}")

print("\n=== 测试SafetyManager ===")
safety = SafetyManager(config.safety, fleet)
decision = safety.evaluate(snapshot)
print(f"[OK] 安全决策: {decision.action}")

print("\n=== 测试CommandScheduler ===")
scheduler = CommandScheduler(config.comm, fsm)
from src.runtime.follower_controller import FollowerCommandSet

plan = scheduler.plan(
    latest,
    MissionState.RUN,
    leader_ref=None,
    follower_cmds=FollowerCommandSet(
        commands={5: np.array([0.1, 0, 0])}, diagnostics={}
    ),
    safety=decision,
)
print(f"[OK] 生成了 {len(plan.follower_actions)} 个follower动作")

print("\n[SUCCESS] 第一阶段测试通过！")
