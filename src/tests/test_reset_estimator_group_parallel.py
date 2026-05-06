"""reset_estimator readiness helper runs across radio groups in parallel."""

import threading
import time

from src.adapters.group_executor_pool import GroupExecutorPool
from src.adapters.wait_for_params import reset_estimator_per_group


class _Fleet:
    def __init__(self):
        self._ids = [1, 2, 4, 5, 7, 8]
        self._groups = {1: 0, 2: 0, 4: 1, 5: 1, 7: 2, 8: 2}

    def all_ids(self):
        return list(self._ids)

    def get_radio_group(self, drone_id):
        return self._groups[drone_id]


class _Transport:
    def __init__(self, per_reset_s=0.03):
        self.per_reset_s = per_reset_s
        self.calls = []
        self._lock = threading.Lock()

    def reset_estimator_and_wait(self, drone_id):
        time.sleep(self.per_reset_s)
        with self._lock:
            self.calls.append((time.time(), drone_id))


fleet = _Fleet()
transport = _Transport(per_reset_s=0.03)
pool = GroupExecutorPool(group_ids=[0, 1, 2])
done = []

try:
    started = time.time()
    reset_estimator_per_group(
        transport,
        fleet,
        pool,
        drone_ids=fleet.all_ids(),
        on_done=done.append,
    )
    elapsed = time.time() - started
finally:
    pool.shutdown(wait=True)

assert elapsed < 0.11, f"reset estimator should run across groups ({elapsed:.3f}s)"
assert set(done) == set(fleet.all_ids())
assert [drone_id for _t, drone_id in transport.calls if fleet.get_radio_group(drone_id) == 0] == [1, 2]
assert [drone_id for _t, drone_id in transport.calls if fleet.get_radio_group(drone_id) == 1] == [4, 5]
assert [drone_id for _t, drone_id in transport.calls if fleet.get_radio_group(drone_id) == 2] == [7, 8]
