"""FollowerExecutor 组间并行契约：

当传入 GroupExecutorPool + transport.radio_group(drone_id) 能归组时，
execute_velocity / execute_hold 应把 actions 按组分派，组间真正并行、
组内按提交顺序串行。不传 pool 时保留旧的同步串行行为。
"""

import threading
import time

import numpy as np

from src.adapters.follower_executor import FollowerExecutor
from src.adapters.group_executor_pool import GroupExecutorPool
from src.runtime.command_plan import FollowerAction, HoldAction


class RecordingTransport:
    def __init__(self, radio_group_map: dict[int, int], per_send_s: float = 0.03):
        self._radio_groups = radio_group_map
        self._per_send_s = per_send_s
        self.order: list[tuple[float, int, int, str]] = []  # (t, group, drone, kind)
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

    def cmd_velocity_world(self, drone_id, vx, vy, vz):
        time.sleep(self._per_send_s)
        with self._lock:
            self.order.append((time.time(), self._radio_groups[drone_id], drone_id, "vel"))


# ---- 旧路径：无 pool，串行 ------------------------------------------------

transport_serial = RecordingTransport({1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2})
serial_executor = FollowerExecutor(transport_serial)
t0 = time.time()
actions = [
    FollowerAction(kind="velocity", drone_id=did, velocity=np.zeros(3))
    for did in (1, 3, 5)
]
result = serial_executor.execute_velocity(actions)
serial_elapsed = time.time() - t0
assert set(result["successes"]) == {1, 3, 5}
assert result["failures"] == []
assert serial_elapsed >= 0.08, f"无 pool 时应串行（预期 ≥0.09s，实测 {serial_elapsed:.3f}s）"

# ---- 新路径：带 pool，跨组并行 -------------------------------------------

transport = RecordingTransport({1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2})
pool = GroupExecutorPool(group_ids=[0, 1, 2])
parallel_executor = FollowerExecutor(transport, group_executor_pool=pool)

actions = [
    FollowerAction(kind="velocity", drone_id=did, velocity=np.zeros(3))
    for did in (1, 3, 5)
]
t0 = time.time()
result = parallel_executor.execute_velocity(actions)
parallel_elapsed = time.time() - t0
assert set(result["successes"]) == {1, 3, 5}
assert result["failures"] == []
assert parallel_elapsed < 0.07, (
    f"带 pool 时跨组应并行（预期 <0.07s，实测 {parallel_elapsed:.3f}s）"
)
# 各组的第一次 send 应几乎同时开始
group_first_send: dict[int, float] = {}
for t, gid, _, _ in transport.order:
    group_first_send.setdefault(gid, t)
starts = sorted(group_first_send.values())
assert starts[-1] - starts[0] < 0.03, "不同 group 的第一次发包应几乎同时"

# ---- 同组多 drone 依然按提交顺序串行 --------------------------------------

transport2 = RecordingTransport({1: 0, 2: 0}, per_send_s=0.02)
pool2 = GroupExecutorPool(group_ids=[0])
executor2 = FollowerExecutor(transport2, group_executor_pool=pool2)
executor2.execute_velocity(
    [
        FollowerAction(kind="velocity", drone_id=1, velocity=np.zeros(3)),
        FollowerAction(kind="velocity", drone_id=2, velocity=np.zeros(3)),
    ]
)
same_group_ids = [d for _, _, d, _ in transport2.order]
assert same_group_ids == [1, 2], "同组发包必须保持原提交顺序"

# ---- 失败汇总：某组异常不影响其它组 ---------------------------------------


class PartialFailTransport(RecordingTransport):
    def cmd_velocity_world(self, drone_id, vx, vy, vz):
        if drone_id == 3:
            raise TimeoutError("radio timeout for drone 3")
        time.sleep(0.01)
        with self._lock:
            self.order.append((time.time(), self._radio_groups[drone_id], drone_id, "vel"))


transport_fail = PartialFailTransport({1: 0, 2: 0, 3: 1, 4: 1, 5: 2})
pool_fail = GroupExecutorPool(group_ids=[0, 1, 2])
exec_fail = FollowerExecutor(transport_fail, group_executor_pool=pool_fail)
result = exec_fail.execute_velocity(
    [
        FollowerAction(kind="velocity", drone_id=1, velocity=np.zeros(3)),
        FollowerAction(kind="velocity", drone_id=3, velocity=np.zeros(3)),
        FollowerAction(kind="velocity", drone_id=5, velocity=np.zeros(3)),
    ]
)
assert set(result["successes"]) == {1, 5}
assert len(result["failures"]) == 1
failure = result["failures"][0]
assert failure["drone_id"] == 3
assert failure["radio_group"] == 1
assert failure["failure_category"] == "timeout"

# ---- execute_hold 等价并行化 ----------------------------------------------

transport_hold = RecordingTransport({1: 0, 3: 1, 5: 2})
pool_hold = GroupExecutorPool(group_ids=[0, 1, 2])
exec_hold = FollowerExecutor(transport_hold, group_executor_pool=pool_hold)
t0 = time.time()
result = exec_hold.execute_hold([HoldAction(drone_id=1), HoldAction(drone_id=3), HoldAction(drone_id=5)])
elapsed = time.time() - t0
assert set(result["successes"]) == {1, 3, 5}
assert elapsed < 0.07

pool.shutdown(wait=True)
pool2.shutdown(wait=True)
pool_fail.shutdown(wait=True)
pool_hold.shutdown(wait=True)

print("[OK] FollowerExecutor cross-group parallel / intra-group serial verified")
