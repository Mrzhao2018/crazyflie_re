"""scheduler._group_drone_ids: 全集输入时返回与慢路径相等的结果"""

from src.config.loader import ConfigLoader
from src.domain.fleet_model import FleetModel
from src.runtime.scheduler import CommandScheduler


config = ConfigLoader.load("config")
fleet = FleetModel(config.fleet)
scheduler = CommandScheduler(config.comm, fleet_model=fleet)

# 传全集 follower_ids
full_follower = list(fleet.follower_ids())
grouped_full = scheduler._group_drone_ids(full_follower)
# 应当与预构建 _follower_groups 一致
assert {k: sorted(v) for k, v in grouped_full.items()} == {
    k: sorted(v) for k, v in scheduler._follower_groups.items()
}

# 传子集
subset = full_follower[:2]
grouped_sub = scheduler._group_drone_ids(subset)
for gid, members in grouped_sub.items():
    assert set(members).issubset(set(full_follower))
    for m in members:
        assert fleet.get_radio_group(m) == gid

# 空集
assert scheduler._group_drone_ids([]) == {}

print("[OK] Scheduler _group_drone_ids fast/slow path parity verified")
