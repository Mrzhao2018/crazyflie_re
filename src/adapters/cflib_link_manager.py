"""Cflib连接管理器"""

from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Callable
from ..domain.fleet_model import FleetModel

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


class CflibLinkManager:
    """管理所有Crazyflie连接"""

    def __init__(self, fleet: FleetModel):
        self.fleet = fleet
        self._scfs = {}  # {drone_id: SyncCrazyflie}
        self._initialized = False
        self._last_connect_report: dict[str, Any] | None = None

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
        logger.info(f"Connecting to drone {drone_id} at {uri}")

        # 创建Crazyflie实例
        cf = Crazyflie(rw_cache="./cache")
        scf = SyncCrazyflie(uri, cf=cf)
        scf.open_link()

        # 等待fully_connected
        timeout = 5.0
        start = time.time()
        while not scf.cf.is_connected():
            if time.time() - start > timeout:
                raise TimeoutError(f"Drone {drone_id} connection timeout")
            time.sleep(0.1)

        self._scfs[drone_id] = scf
        logger.info(f"Connected to drone {drone_id}")
        time.sleep(0.2)  # 多机连接间隔

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
            cflib.crtp.init_drivers()
            self._initialized = True

        started_at = time.time()
        report: dict[str, Any] = {
            "ok": True,
            "connected": [],
            "failures": [],
            "radio_groups": {},
        }
        self._last_connect_report = report

        grouped_drone_ids = self._grouped_drone_ids()
        if not parallel_groups:
            for group_id, drone_ids in grouped_drone_ids.items():
                if on_group_start is not None:
                    on_group_start({"group_id": group_id, "drone_ids": list(drone_ids)})

                group_result = self._connect_group(group_id, drone_ids)
                report["radio_groups"][group_id] = group_result
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
