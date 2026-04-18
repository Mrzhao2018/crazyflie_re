"""GroupExecutorPool 契约：按 radio_group 建立持久化线程池，组间任务并行、组内串行。"""

import threading
import time

from src.adapters.group_executor_pool import GroupExecutorPool


# ---- 基本提交 ------------------------------------------------------------

pool = GroupExecutorPool(group_ids=[0, 1, 2])
assert set(pool.group_ids) == {0, 1, 2}

# 跨组并行：三个组各跑一个 100ms 任务，总耗时应接近 100ms 而非 300ms
thread_ids_by_group: dict[int, int] = {}
barrier = threading.Barrier(3)


def _slow_task(group_id: int) -> int:
    barrier.wait()  # 三个线程同时起步，证明它们确实并行而非串行
    thread_ids_by_group[group_id] = threading.get_ident()
    time.sleep(0.1)
    return group_id


t0 = time.time()
futures = {gid: pool.submit(gid, _slow_task, gid) for gid in (0, 1, 2)}
results = {gid: fut.result(timeout=2.0) for gid, fut in futures.items()}
elapsed = time.time() - t0
assert results == {0: 0, 1: 1, 2: 2}
assert elapsed < 0.25, f"三个组并行下耗时应 <0.25s，实测 {elapsed:.3f}s"

# 三个任务应来自三条不同线程
assert len(set(thread_ids_by_group.values())) == 3

# ---- 组内串行 -----------------------------------------------------------

ordered: list[int] = []


def _record(tag: int) -> None:
    time.sleep(0.02)
    ordered.append(tag)


futs = [pool.submit(0, _record, i) for i in range(5)]
for fut in futs:
    fut.result(timeout=2.0)
assert ordered == [0, 1, 2, 3, 4], "同组任务必须按提交顺序串行执行"

# ---- 未知 group_id 触发错误 ---------------------------------------------

try:
    pool.submit(99, lambda: None)
except KeyError:
    pass
else:
    raise AssertionError("未知 group_id 应该抛 KeyError")

# ---- shutdown 释放线程 --------------------------------------------------

pool.shutdown(wait=True)
# shutdown 之后再提交应报错
try:
    pool.submit(0, lambda: None)
except RuntimeError:
    pass
else:
    raise AssertionError("shutdown 后 submit 应抛 RuntimeError")

print("[OK] GroupExecutorPool parallel-across-groups / serial-within-group verified")
