"""按 radio_group 并行执行 wait_for_params 的工具函数。

cflib 的 ``wait_for_params`` 会等当前 TOC 下载完成（首连需要拉一整份 param
TOC + log TOC，单机可能达 1–2 秒）。10 机串行加起来就是十几秒的 readiness
time。按 radio_group 并行后，组间时间重叠；组内仍串行，避免同一 dongle
上的 TOC 请求互相打架。
"""

from __future__ import annotations

from concurrent.futures import Future
from typing import Callable

from .group_executor_pool import GroupExecutorPool


def wait_for_params_per_group(
    transport,
    fleet,
    pool: GroupExecutorPool,
    *,
    on_done: Callable[[int], None] | None = None,
) -> None:
    """按 group 并行调用 ``transport.wait_for_params(drone_id)``。

    on_done 会在每架 drone 成功 wait 后被调用（从对应组的 worker 线程里）。
    任何组内异常会被记录并在所有组完成后原样重抛。
    """

    grouped: dict[int, list[int]] = {}
    for drone_id in fleet.all_ids():
        group_id = fleet.get_radio_group(drone_id)
        grouped.setdefault(group_id, []).append(drone_id)

    futures: list[tuple[int, list[int], Future]] = []
    for group_id, drones in grouped.items():
        if group_id not in pool.group_ids:
            # 未知 group：fallback 到 inline 执行
            for drone_id in drones:
                transport.wait_for_params(drone_id)
                if on_done is not None:
                    on_done(drone_id)
            continue

        def _run(drones=drones):
            for drone_id in drones:
                transport.wait_for_params(drone_id)
                if on_done is not None:
                    on_done(drone_id)

        futures.append((group_id, drones, pool.submit(group_id, _run)))

    first_exc: BaseException | None = None
    for _, _, fut in futures:
        exc = fut.exception()
        if exc is not None and first_exc is None:
            first_exc = exc
    if first_exc is not None:
        raise first_exc
