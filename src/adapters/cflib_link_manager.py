"""Cflib连接管理器"""

from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Callable
from ..domain.fleet_model import FleetModel
from ..runtime.link_quality_bus import LinkQualityBus
from ..runtime.link_state_bus import LinkStateBus
from .radio_driver_select import RadioDriverMode, select_radio_driver

cflib: Any | None = None
Crazyflie: Any | None = None
SyncCrazyflie: Any | None = None

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
except ImportError:
    print("[WARNING] cflib未安装，适配层将无法使用")

logger = logging.getLogger(__name__)


ConnectEventCallback = Callable[[dict[str, Any]], None]


_LINK_METRIC_CALLERS = (
    ("link_quality", "link_quality_updated"),
    ("uplink_rssi", "uplink_rssi_updated"),
    ("uplink_rate", "uplink_rate_updated"),
    ("downlink_rate", "downlink_rate_updated"),
    ("uplink_congestion", "uplink_congestion_updated"),
    ("downlink_congestion", "downlink_congestion_updated"),
)


def wire_link_quality_callbacks(
    cf: Any,
    *,
    drone_id: int,
    bus: LinkQualityBus | None,
    time_fn: Callable[[], float] = time.time,
) -> None:
    """把 cflib ``cf.link_statistics`` 的 6 个 Caller 挂到 ``bus`` 上。

    ``bus=None`` 时不做任何挂接；此时也不应启动 link_statistics 采集，避免
    cflib 在后台持续消耗 CPU / 网络但无人消费。

    注意：cflib 0.1.31 已在 ``Crazyflie.__init__`` 里把
    ``self.connected.add_callback(lambda uri: self.link_statistics.start())``
    与对应的 ``disconnected → stop()`` 挂好（见 cflib
    ``crazyflie/__init__.py`` 约 L159–L162），所以此函数**不**需要再手动
    ``link_statistics.start()``——那是一次多余调用，对应 ``stop()`` 路径也由
    cflib 自己管理。
    """
    if bus is None:
        return
    link_stats = getattr(cf, "link_statistics", None)
    if link_stats is None:
        return
    for metric, caller_name in _LINK_METRIC_CALLERS:
        caller = getattr(link_stats, caller_name, None)
        if caller is None:
            continue

        def _make_cb(metric=metric):
            def _cb(value):
                bus.update(drone_id, metric, value, time_fn())
            return _cb

        caller.add_callback(_make_cb())


class CflibLinkManager:
    """管理所有Crazyflie连接"""

    def __init__(
        self,
        fleet: FleetModel,
        *,
        link_quality_bus: LinkQualityBus | None = None,
        link_state_bus: LinkStateBus | None = None,
        connect_pace_s: float = 0.2,
        connect_timeout_s: float = 5.0,
        radio_driver: RadioDriverMode | str = RadioDriverMode.AUTO,
    ):
        self.fleet = fleet
        self._scfs = {}  # {drone_id: SyncCrazyflie}
        self._initialized = False
        self._last_connect_report: dict[str, Any] | None = None
        self._link_quality_bus = link_quality_bus
        self._link_state_bus = link_state_bus
        self._connect_pace_s = float(connect_pace_s)
        self._connect_timeout_s = float(connect_timeout_s)
        self._radio_driver = radio_driver

    def _grouped_drone_ids(self) -> dict[int, list[int]]:
        grouped: dict[int, list[int]] = {}
        for drone_id in self.fleet.all_ids():
            group_id = self.fleet.get_radio_group(drone_id)
            grouped.setdefault(group_id, []).append(drone_id)
        return dict(sorted(grouped.items()))

    def _connect_drone(self, drone_id: int) -> None:
        assert Crazyflie is not None
        assert SyncCrazyflie is not None
        uri = self.fleet.get_uri(drone_id)
        logger.info("Connecting to drone %s at %s", drone_id, uri)

        # 创建Crazyflie实例。ro_cache 跨机共享只读 TOC，rw_cache 保留原 cache/
        # 兼容旧目录：同时设置 ro_cache=cache/ro、rw_cache=cache/rw。
        cf = Crazyflie(ro_cache="./cache/ro", rw_cache="./cache/rw")
        # 在 open_link 之前挂上 link_quality 回调：cflib 的 RadioLinkStatistics 在
        # open_link 触发 get_link_driver 时开始产生数据，太晚挂就会漏掉最开始几秒。
        wire_link_quality_callbacks(
            cf, drone_id=drone_id, bus=self._link_quality_bus
        )
        if self._link_state_bus is not None:
            cf.connection_lost.add_callback(
                lambda _uri, errmsg, _drone_id=drone_id: self._link_state_bus.mark_disconnected(
                    _drone_id, error=str(errmsg)
                )
            )
            cf.disconnected.add_callback(
                lambda _uri, _drone_id=drone_id: self._link_state_bus.mark_disconnected(
                    _drone_id
                )
            )
        scf = SyncCrazyflie(uri, cf=cf)
        scf.open_link()

        # 等待fully_connected
        timeout = self._connect_timeout_s
        start = time.time()
        while not scf.cf.is_connected():
            if time.time() - start > timeout:
                raise TimeoutError(f"Drone {drone_id} connection timeout")
            time.sleep(0.1)

        self._scfs[drone_id] = scf
        if self._link_state_bus is not None:
            self._link_state_bus.mark_connected(drone_id)
        logger.info("Connected to drone %s", drone_id)
        if self._connect_pace_s > 0:
            time.sleep(self._connect_pace_s)  # 多机连接间隔

    def _connect_group(self, group_id: int, drone_ids: list[int]) -> dict[str, Any]:
        started_at = time.time()
        group_result: dict[str, Any] = {
            "group_id": group_id,
            "drone_ids": list(drone_ids),
            "connected": [],
            "failures": [],
            "ok": True,
        }
        for drone_id in drone_ids:
            try:
                self._connect_drone(drone_id)
                group_result["connected"].append(drone_id)
            except Exception as exc:
                group_result["ok"] = False
                group_result["failures"].append(
                    {
                        "drone_id": drone_id,
                        "group_id": group_id,
                        "error": str(exc),
                        "exception_type": type(exc).__name__,
                    }
                )
                break
        group_result["duration_s"] = time.time() - started_at
        return group_result

    def last_connect_report(self) -> dict[str, Any] | None:
        return self._last_connect_report

    def connect_all(
        self,
        *,
        on_group_start: ConnectEventCallback | None = None,
        on_group_result: ConnectEventCallback | None = None,
        parallel_groups: bool = False,
    ):
        """连接所有无人机"""
        if cflib is None or Crazyflie is None or SyncCrazyflie is None:
            raise RuntimeError(
                "cflib 未安装或当前解释器不可用，无法建立 Crazyflie 连接"
            )

        if not self._initialized:
            select_radio_driver(self._radio_driver)
            self._initialized = True

        started_at = time.time()
        report: dict[str, Any] = {
            "ok": True,
            "connected": [],
            "failures": [],
            "radio_groups": {},
            "parallel": bool(parallel_groups),
            "per_group_duration_s": {},
        }
        self._last_connect_report = report

        grouped_drone_ids = self._grouped_drone_ids()
        if not parallel_groups:
            for group_id, drone_ids in grouped_drone_ids.items():
                if on_group_start is not None:
                    on_group_start({"group_id": group_id, "drone_ids": list(drone_ids)})

                group_result = self._connect_group(group_id, drone_ids)
                report["radio_groups"][group_id] = group_result
                report["per_group_duration_s"][group_id] = float(
                    group_result.get("duration_s", 0.0)
                )
                report["connected"].extend(group_result["connected"])
                report["failures"].extend(group_result["failures"])
                report["ok"] = report["ok"] and group_result["ok"]

                if on_group_result is not None:
                    on_group_result(group_result)

                if not group_result["ok"]:
                    failure = group_result["failures"][0]
                    logger.error(
                        "Failed to connect radio group %s at drone %s: %s",
                        group_id,
                        failure["drone_id"],
                        failure["error"],
                    )
                    report["duration_s"] = time.time() - started_at
                    self.close_all()
                    raise RuntimeError(failure["error"])

            report["duration_s"] = time.time() - started_at
            return report

        for group_id, drone_ids in grouped_drone_ids.items():
            if on_group_start is not None:
                on_group_start({"group_id": group_id, "drone_ids": list(drone_ids)})

        future_to_group = {}
        with ThreadPoolExecutor(max_workers=len(grouped_drone_ids) or 1) as executor:
            for group_id, drone_ids in grouped_drone_ids.items():
                future_to_group[
                    executor.submit(self._connect_group, group_id, drone_ids)
                ] = group_id

            for future in as_completed(future_to_group):
                group_id = future_to_group[future]
                group_result = future.result()
                report["radio_groups"][group_id] = group_result
                report["per_group_duration_s"][group_id] = float(
                    group_result.get("duration_s", 0.0)
                )
                report["connected"].extend(group_result["connected"])
                report["failures"].extend(group_result["failures"])
                report["ok"] = report["ok"] and group_result["ok"]

                if on_group_result is not None:
                    on_group_result(group_result)

        if not report["ok"]:
            first_failure = next(
                failure for failure in report["failures"] if isinstance(failure, dict)
            )
            logger.error(
                "Failed to connect radio group %s at drone %s: %s",
                first_failure.get("group_id"),
                first_failure.get("drone_id"),
                first_failure.get("error"),
            )
            report["duration_s"] = time.time() - started_at
            self.close_all()
            raise RuntimeError(str(first_failure.get("error")))

        report["duration_s"] = time.time() - started_at
        return report

    def close_all(self):
        """关闭所有连接"""
        # 逐机关闭时如果某一个 close_link 抛异常，不能让剩下的链路泄漏。
        for drone_id, scf in list(self._scfs.items()):
            logger.info("Closing connection to drone %s", drone_id)
            try:
                scf.close_link()
            except Exception:
                logger.exception("Failed to close link for drone %s", drone_id)
        self._scfs.clear()

    def get(self, drone_id: int) -> Any:
        """获取指定无人机的连接"""
        if drone_id not in self._scfs:
            raise KeyError(f"Drone {drone_id} is not connected")
        return self._scfs[drone_id]

    def reconnect(
        self,
        drone_id: int,
        *,
        attempts: int,
        backoff_s: float,
        timeout_s: float,
    ) -> dict[str, Any]:
        """对已断开的 drone 做有限次数重连。

        每次尝试会先 close 旧链路、再 open 新链路，期间等待 ``backoff_s``。
        成功时把新的 SyncCrazyflie 覆盖到 ``_scfs`` 中；失败时保留最后一次异常
        的 error 字段。返回值结构可直接被 failure_policy 做事件记录。
        """

        if attempts <= 0:
            raise ValueError("reconnect attempts 必须 >= 1")

        assert Crazyflie is not None
        assert SyncCrazyflie is not None

        uri = self.fleet.get_uri(drone_id)
        old_scf = self._scfs.pop(drone_id, None)
        if old_scf is not None:
            try:
                old_scf.close_link()
            except Exception:
                logger.exception("Failed to close stale link for drone %s", drone_id)

        last_error: str | None = None
        for attempt_idx in range(1, attempts + 1):
            if backoff_s > 0 and attempt_idx > 1:
                time.sleep(backoff_s)

            try:
                cf = Crazyflie(ro_cache="./cache/ro", rw_cache="./cache/rw")
                wire_link_quality_callbacks(
                    cf, drone_id=drone_id, bus=self._link_quality_bus
                )
                if self._link_state_bus is not None:
                    cf.connection_lost.add_callback(
                        lambda _uri, errmsg, _drone_id=drone_id: self._link_state_bus.mark_disconnected(
                            _drone_id, error=str(errmsg)
                        )
                    )
                    cf.disconnected.add_callback(
                        lambda _uri, _drone_id=drone_id: self._link_state_bus.mark_disconnected(
                            _drone_id
                        )
                    )
                scf = SyncCrazyflie(uri, cf=cf)
                scf.open_link()

                start = time.time()
                while not scf.cf.is_connected():
                    if time.time() - start > timeout_s:
                        raise TimeoutError(
                            f"Drone {drone_id} reconnect timeout"
                        )
                    time.sleep(0.05)

                self._scfs[drone_id] = scf
                if self._link_state_bus is not None:
                    self._link_state_bus.mark_connected(drone_id)
                return {
                    "ok": True,
                    "drone_id": drone_id,
                    "attempt_count": attempt_idx,
                    "radio_group": self.fleet.get_radio_group(drone_id),
                }
            except Exception as exc:
                last_error = str(exc)
                logger.warning(
                    "Reconnect attempt %d/%d for drone %s failed: %s",
                    attempt_idx,
                    attempts,
                    drone_id,
                    exc,
                )

        return {
            "ok": False,
            "drone_id": drone_id,
            "attempt_count": attempts,
            "error": last_error,
            "radio_group": self.fleet.get_radio_group(drone_id),
        }
