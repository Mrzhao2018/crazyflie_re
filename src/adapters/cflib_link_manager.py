"""Cflib连接管理器"""

import logging
import time
from typing import Any
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


class CflibLinkManager:
    """管理所有Crazyflie连接"""

    def __init__(self, fleet: FleetModel):
        self.fleet = fleet
        self._scfs = {}  # {drone_id: SyncCrazyflie}
        self._initialized = False

    def connect_all(self):
        """连接所有无人机"""
        if cflib is None or Crazyflie is None or SyncCrazyflie is None:
            raise RuntimeError(
                "cflib 未安装或当前解释器不可用，无法建立 Crazyflie 连接"
            )

        if not self._initialized:
            cflib.crtp.init_drivers()
            self._initialized = True

        for drone_id in self.fleet.all_ids():
            uri = self.fleet.get_uri(drone_id)
            logger.info(f"Connecting to drone {drone_id} at {uri}")

            try:
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

            except Exception as e:
                logger.error(f"Failed to connect drone {drone_id}: {e}")
                self.close_all()
                raise

    def close_all(self):
        """关闭所有连接"""
        for drone_id, scf in self._scfs.items():
            logger.info(f"Closing connection to drone {drone_id}")
            scf.close_link()
        self._scfs.clear()

    def get(self, drone_id: int) -> Any:
        """获取指定无人机的连接"""
        if drone_id not in self._scfs:
            raise KeyError(f"Drone {drone_id} is not connected")
        return self._scfs[drone_id]
