"""CommandScheduler link_quality 自适应：低链路质量时组级降频 + deadband 放大。"""

import time

import numpy as np

from src.runtime.scheduler import CommandScheduler
from src.runtime.safety_manager import SafetyDecision
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.follower_controller import FollowerCommandSet
from src.runtime.pose_snapshot import PoseSnapshot
from src.config.schema import CommConfig


class FakeFleet:
    def __init__(self):
        self._f = [5, 6]
        self._l = [1, 2, 3, 4]
        self._map = {1: 0, 2: 0, 3: 1, 4: 1, 5: 0, 6: 1}

    def follower_ids(self):
        return self._f.copy()

    def leader_ids(self):
        return self._l.copy()

    def get_radio_group(self, drone_id):
        return self._map[drone_id]


fleet = FakeFleet()
fsm = MissionFSM()
for s in (
    MissionState.CONNECT,
    MissionState.POSE_READY,
    MissionState.PREFLIGHT,
    MissionState.TAKEOFF,
    MissionState.SETTLE,
    MissionState.RUN,
):
    fsm.transition(s)

comm = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
    follower_cmd_deadband=0.0,
    link_quality_soft_floor=60.0,
    link_quality_backoff_scale=2.0,
    link_quality_deadband_scale=3.0,
)

# group 0: 链路质量差 (40)，应被降频；group 1: 好 (95)，正常
link_quality_by_group = {0: 40.0, 1: 95.0}


def provider(group_id: int) -> float | None:
    return link_quality_by_group.get(group_id)


scheduler = CommandScheduler(
    comm, fsm, fleet, link_quality_provider=provider
)

base_interval = 1.0 / comm.follower_tx_freq  # 0.125s
assert base_interval == 0.125

# Seed scheduler: 两个 group 都刚刚发过
now = time.monotonic()
scheduler.last_follower_tx_time = {0: now - 0.15, 1: now - 0.15}
scheduler.last_pose_seq_by_group = {0: -1, 1: -1}

snapshot = PoseSnapshot(
    seq=10,
    t_meas=0.0,
    positions=np.zeros((6, 3)),
    fresh_mask=np.ones(6, dtype=bool),
    disconnected_ids=[],
)
commands = FollowerCommandSet(
    commands={5: np.array([0.3, 0.0, 0.0]), 6: np.array([0.3, 0.0, 0.0])},
    diagnostics={},
)
safety = SafetyDecision(action="EXECUTE", reasons=[])
plan = scheduler.plan(
    snapshot, MissionState.RUN, None, commands, safety
)

# group 1（质量好）：间隔 0.15 > 0.125 -> 应 sent
# group 0（质量差）：scaled interval = 0.25，间隔 0.15 < 0.25 -> blocked
sent_drones = [a.drone_id for a in plan.follower_actions]
assert sent_drones == [6], f"仅 group 1 (drone 6) 应发包，实得 {sent_drones}"
assert 1 in plan.diagnostics["follower_tx_groups_sent"]
assert 0 in plan.diagnostics["follower_tx_groups_blocked"]
assert 0 in plan.diagnostics.get("link_quality_backoff_groups", []), (
    "坏链路的 group 应当出现在 link_quality_backoff_groups 里"
)

# ---- link_quality 足够好时不触发 backoff --------------------------------

link_quality_by_group[0] = 80.0  # 抬回到阈值之上
scheduler.last_follower_tx_time = {0: now - 0.15, 1: now - 0.15}
scheduler.last_pose_seq_by_group = {0: -1, 1: -1}
plan2 = scheduler.plan(
    snapshot, MissionState.RUN, None, commands, safety
)
sent2 = sorted(a.drone_id for a in plan2.follower_actions)
assert sent2 == [5, 6]
assert plan2.diagnostics.get("link_quality_backoff_groups", []) == []

# ---- provider=None / soft_floor=0：完全沿用旧路径 ------------------------

comm_off = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
    link_quality_soft_floor=0.0,  # 关闭
)
scheduler_off = CommandScheduler(comm_off, fsm, fleet, link_quality_provider=provider)
scheduler_off.last_follower_tx_time = {0: now - 0.15, 1: now - 0.15}
scheduler_off.last_pose_seq_by_group = {0: -1, 1: -1}
link_quality_by_group[0] = 10.0  # 即使很差
plan3 = scheduler_off.plan(
    snapshot, MissionState.RUN, None, commands, safety
)
assert sorted(a.drone_id for a in plan3.follower_actions) == [5, 6], (
    "soft_floor=0 时 link_quality 不应影响发包"
)
assert plan3.diagnostics.get("link_quality_backoff_groups", []) == []

# ---- deadband scale ----------------------------------------------------

comm_db = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
    follower_cmd_deadband=0.05,
    link_quality_soft_floor=60.0,
    link_quality_backoff_scale=1.0,  # 不改频率
    link_quality_deadband_scale=3.0,  # 坏链路：deadband 变成 0.15
)
link_quality_by_group[0] = 30.0
link_quality_by_group[1] = 95.0
scheduler_db = CommandScheduler(
    comm_db, fsm, fleet, link_quality_provider=provider
)
scheduler_db.last_follower_tx_time = {0: now - 1.0, 1: now - 1.0}
scheduler_db.last_pose_seq_by_group = {0: -1, 1: -1}
# 让每架机都有"上一次命令" -> 本次 delta = 0.1
scheduler_db.last_follower_cmd = {
    5: np.array([0.0, 0.0, 0.0]),
    6: np.array([0.0, 0.0, 0.0]),
}
cmds_mid = FollowerCommandSet(
    commands={5: np.array([0.1, 0.0, 0.0]), 6: np.array([0.1, 0.0, 0.0])},
    diagnostics={},
)
plan_db = scheduler_db.plan(
    snapshot, MissionState.RUN, None, cmds_mid, safety
)
# drone 5 属于 group 0（差），deadband = 0.05 * 3 = 0.15 > 0.1 -> 被滤掉
# drone 6 属于 group 1（好），deadband = 0.05 -> 0.1 >= 0.05 -> 通过
passed = sorted(a.drone_id for a in plan_db.follower_actions)
assert passed == [6], f"坏链路的 drone 5 应被放大后的 deadband 拦截，实得 {passed}"

# ---- group health score provider ---------------------------------------

health_by_group = {
    0: {"score": 35.0, "reason": "congestion"},
    1: {"score": 90.0, "reason": "healthy"},
}


def health_provider(group_id: int):
    return health_by_group.get(group_id)


comm_health = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
    follower_cmd_deadband=0.0,
    link_quality_soft_floor=60.0,
    link_quality_backoff_scale=2.0,
    link_quality_deadband_scale=3.0,
    min_stream_keepalive_hz=2.0,
)
scheduler_health = CommandScheduler(
    comm_health, fsm, fleet, link_quality_provider=health_provider
)
scheduler_health.last_follower_tx_time = {0: now - 0.15, 1: now - 0.15}
scheduler_health.last_pose_seq_by_group = {0: -1, 1: -1}
plan_health = scheduler_health.plan(
    snapshot, MissionState.RUN, None, commands, safety
)
assert [action.drone_id for action in plan_health.follower_actions] == [6]
assert plan_health.diagnostics["group_health_scores"][0] == 35.0
assert 0 in plan_health.diagnostics["link_quality_backoff_groups"]

print("[OK] CommandScheduler link_quality backoff + deadband scale verified")
