"""按 radio_group 组织的持久化执行线程池。

每个 radio_group 有且仅有一条工作线程 —— 组间任务真正并行，组内任务按提交
顺序串行执行。这样做的原因：

* 同一个 cflib dongle（对应一个 radio_group）内部发包是串行的，强行多线程
  发送同一 dongle 只会增加锁争用；
* 但不同 dongle（不同 radio_group）的发包完全独立，当前代码却在 Python 主
  线程里串行发到所有 follower 上，跨组发包被完全串行化；
* 这个 pool 的职责就是把"组内必须串行"这条约束保留，把"组间可以并行"的空
  间释放出来。
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from typing import Callable, Iterable


class GroupExecutorPool:
    def __init__(self, group_ids: Iterable[int]):
        self._executors: dict[int, ThreadPoolExecutor] = {}
        self._shutdown = False
        for gid in group_ids:
            if gid in self._executors:
                continue
            self._executors[int(gid)] = ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix=f"group-{int(gid)}",
            )

    @property
    def group_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self._executors))

    def submit(self, group_id: int, fn: Callable, /, *args, **kwargs) -> Future:
        if self._shutdown:
            raise RuntimeError("GroupExecutorPool has been shut down")
        if group_id not in self._executors:
            raise KeyError(f"Unknown radio_group: {group_id}")
        return self._executors[group_id].submit(fn, *args, **kwargs)

    def shutdown(self, *, wait: bool = True) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        for executor in self._executors.values():
            executor.shutdown(wait=wait)
