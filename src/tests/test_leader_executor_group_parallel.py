"""LeaderExecutor batch_goto 跨组并行契约：

当 batch_goto 涉及多个 radio_group 的 leader，使用 GroupExecutorPool 时应跨组
并行、组内保序。takeoff / land / start_trajectory 这些对时序更敏感的动作在
真机上默认不要求并行化——测试保证这些路径旧行为不变。
"""

import threading
import time

import numpy as np

from src.adapters.leader_executor import LeaderExecutor
from src.adapters.group_executor_pool import GroupExecutorPool
from src.runtime.command_plan import LeaderAction


class RecordingTransport:
    def __init__(self, radio_group_map: dict[int, int], per_send_s: float = 0.03):
        self._radio_groups = radio_group_map
        self._per_send_s = per_send_s
        self.calls: list[tuple[float, int, int, str]] = []  # (t, group, drone, kind)
        self._lock = threading.Lock()

    def radio_group(self, drone_id: int) -> int:
        return self._radio_groups[drone_id]

    def classify_command_failure(self, *, drone_id, command_kind, exception):
        return {
            "drone_id": drone_id,
            "radio_group": self._radio_groups.get(drone_id),
            "command_kind": command_kind,
            "error": str(exception),
            "error_type": type(exception).__name__,
            "failure_category": "timeout",
            "retryable": True,
        }

    def hl_go_to(self, drone_id, x, y, z, duration):
        time.sleep(self._per_send_s)
        with self._lock:
            self.calls.append((time.time(), self._radio_groups[drone_id], drone_id, "goto"))

    def hl_takeoff(self, drone_id, height, duration):
        time.sleep(self._per_send_s)
        with self._lock:
            self.calls.append((time.time(), self._radio_groups[drone_id], drone_id, "takeoff"))

    def hl_land(self, drone_id, height, duration):
        time.sleep(self._per_send_s)
        with self._lock:
            self.calls.append((time.time(), self._radio_groups[drone_id], drone_id, "land"))

    def hl_start_trajectory(
        self,
        drone_id,
        trajectory_id,
        time_scale=1.0,
        relative_position=False,
        relative_yaw=False,
        reversed=False,
    ):
        time.sleep(self._per_send_s)
        with self._lock:
            self.calls.append((time.time(), self._radio_groups[drone_id], drone_id, "traj"))


# ---- batch_goto 跨组并行 ------------------------------------------------

radio_groups = {1: 0, 4: 1, 7: 2, 8: 2}
transport = RecordingTransport(radio_groups)
pool = GroupExecutorPool(group_ids=[0, 1, 2])
executor = LeaderExecutor(transport, group_executor_pool=pool)

positions = {1: np.array([0.0, 0.0, 0.5]), 4: np.array([1.0, 0.0, 0.5]),
             7: np.array([0.0, 1.0, 0.5]), 8: np.array([1.0, 1.0, 0.5])}
t0 = time.time()
results = executor.execute(
    [
        LeaderAction(
            kind="batch_goto",
            drone_ids=[1, 4, 7, 8],
            payload={"positions": positions, "duration": 1.0},
        )
    ]
)
elapsed = time.time() - t0

assert results[0]["kind"] == "batch_goto"
assert set(results[0]["successes"]) == {1, 4, 7, 8}
assert results[0]["failures"] == []

# 4 架 leader 跨 3 个 group：group 2 有 2 个 leader 串行（~60ms），其它两组各
# ~30ms 并行跑。整体应接近 group 2 的耗时（~60-90ms），远小于 4*30ms=120ms。
assert elapsed < 0.1, f"跨组 batch_goto 应并行（实测 {elapsed:.3f}s）"

# group 2 内部两次发包应保持 drone_id 顺序
group2_order = [drone for _, gid, drone, _ in transport.calls if gid == 2]
assert group2_order == [7, 8]

# ---- takeoff / land / trajectory 不受 pool 影响（保持旧同步行为） --------

transport2 = RecordingTransport(radio_groups, per_send_s=0.01)
executor2 = LeaderExecutor(transport2, group_executor_pool=pool)
executor2.execute(
    [
        LeaderAction(
            kind="takeoff",
            drone_ids=[1, 4, 7, 8],
            payload={"height": 0.5, "duration": 2.0},
        )
    ]
)
takeoff_order = [d for _, _, d, kind in transport2.calls if kind == "takeoff"]
assert takeoff_order == [1, 4, 7, 8], "takeoff 保留旧的同步按序行为"

# ---- 无 pool 时，batch_goto 走旧同步路径 ----------------------------------

transport_nopool = RecordingTransport(radio_groups)
executor_nopool = LeaderExecutor(transport_nopool)
t0 = time.time()
executor_nopool.execute(
    [
        LeaderAction(
            kind="batch_goto",
            drone_ids=[1, 4, 7, 8],
            payload={"positions": positions, "duration": 1.0},
        )
    ]
)
nopool_elapsed = time.time() - t0
assert nopool_elapsed >= 0.11, (
    f"无 pool 时 batch_goto 应串行（预期 ≥0.11s，实测 {nopool_elapsed:.3f}s）"
)

pool.shutdown(wait=True)
print("[OK] LeaderExecutor batch_goto cross-group parallel + legacy fallback verified")
