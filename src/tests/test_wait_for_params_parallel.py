"""按 group 并行执行 wait_for_params 的契约。

通过把 wait_for_params 提交到 GroupExecutorPool：跨组并行、组内按 drone 顺序串行。
"""

import threading
import time

from src.adapters.group_executor_pool import GroupExecutorPool
from src.adapters.wait_for_params import wait_for_params_per_group


class SleepTransport:
    def __init__(self, per_call_s: float = 0.05):
        self._per_call_s = per_call_s
        self.order: list[tuple[float, int]] = []
        self._lock = threading.Lock()

    def wait_for_params(self, drone_id: int) -> None:
        time.sleep(self._per_call_s)
        with self._lock:
            self.order.append((time.time(), drone_id))


class FakeFleet:
    def __init__(self, drone_to_group: dict[int, int]):
        self._drone_to_group = drone_to_group

    def all_ids(self):
        return list(self._drone_to_group.keys())

    def get_radio_group(self, drone_id):
        return self._drone_to_group[drone_id]


# 3 组，每组 3 机，每次 wait 50ms -> 串行 450ms，并行 ~150ms
fleet = FakeFleet({
    1: 0, 2: 0, 3: 0,
    4: 1, 5: 1, 6: 1,
    7: 2, 8: 2, 9: 2,
})
transport = SleepTransport(per_call_s=0.05)
pool = GroupExecutorPool(group_ids=[0, 1, 2])

completed: list[int] = []
lock = threading.Lock()


def _on_done(drone_id: int) -> None:
    with lock:
        completed.append(drone_id)


t0 = time.time()
wait_for_params_per_group(transport, fleet, pool, on_done=_on_done)
elapsed = time.time() - t0

assert set(completed) == set(fleet.all_ids())
assert elapsed < 0.25, f"跨组并行 wait_for_params 应 <0.25s，实测 {elapsed:.3f}s"

# 组内按 drone_id 顺序串行
group0_order = [d for _, d in transport.order if fleet.get_radio_group(d) == 0]
assert group0_order == [1, 2, 3]

# ---- 异常透传 -----------------------------------------------------------


class FailingTransport(SleepTransport):
    def wait_for_params(self, drone_id):
        if drone_id == 5:
            raise TimeoutError(f"wait timeout {drone_id}")
        super().wait_for_params(drone_id)


failing = FailingTransport(per_call_s=0.02)
pool2 = GroupExecutorPool(group_ids=[0, 1, 2])
try:
    wait_for_params_per_group(failing, fleet, pool2, on_done=lambda _: None)
except TimeoutError as exc:
    assert "5" in str(exc)
else:
    raise AssertionError("异常应当被重新抛出")

pool.shutdown(wait=True)
pool2.shutdown(wait=True)

print("[OK] wait_for_params_per_group parallel + failure contracts verified")
