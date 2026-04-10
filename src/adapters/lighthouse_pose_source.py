"""Lighthouse定位数据源"""

import logging
import threading
import time
import numpy as np

logger = logging.getLogger(__name__)


class LighthousePoseSource:
    """从Lighthouse读取定位数据"""

    def __init__(self, link_manager, fleet_model, log_freq_hz=10.0):
        self.link_manager = link_manager
        self.fleet = fleet_model
        self.log_period_ms = int(1000.0 / log_freq_hz)
        self._callbacks = []
        self._health_callbacks = []
        self._log_configs = {}
        self._sync_loggers = {}
        self._threads = []
        self._running = False

    def start(self):
        """启动定位数据流"""
        self._threads = []
        self._running = True

        for drone_id in self.fleet.all_ids():
            scf = self.link_manager.get(drone_id)

            from cflib.crazyflie.log import LogConfig
            from cflib.crazyflie.syncLogger import SyncLogger

            pose_conf = LogConfig(
                name=f"pose_{drone_id}", period_in_ms=self.log_period_ms
            )
            pose_conf.add_variable("stateEstimate.x", "float")
            pose_conf.add_variable("stateEstimate.y", "float")
            pose_conf.add_variable("stateEstimate.z", "float")

            health_conf = LogConfig(
                name=f"health_{drone_id}", period_in_ms=max(100, self.log_period_ms * 5)
            )
            health_conf.add_variable("pm.vbat", "float")

            sync_logger = SyncLogger(scf, [pose_conf, health_conf])
            try:
                sync_logger.connect()
            except RuntimeError as exc:
                logger.error("Lighthouse logger connect failed for drone %s: %s", drone_id, exc)
                raise
            except Exception as exc:
                logger.error("Unexpected logger connect failure for drone %s: %s", drone_id, exc)
                raise
            worker = threading.Thread(
                target=self._logger_worker,
                args=(drone_id, sync_logger),
                daemon=True,
            )
            worker.start()
            self._sync_loggers[drone_id] = sync_logger
            self._log_configs[drone_id] = [pose_conf, health_conf]
            self._threads.append(worker)

        logger.info("Lighthouse pose source started")

    def stop(self):
        """停止定位数据流"""
        self._running = False
        for sync_logger in self._sync_loggers.values():
            try:
                sync_logger.disconnect()
            except RuntimeError as exc:
                logger.warning("Disconnect logger failed: %s", exc)
            except Exception as exc:
                logger.warning("Unexpected disconnect failure: %s", exc)
        for worker in self._threads:
            worker.join(timeout=1.0)
        self._threads.clear()
        self._sync_loggers.clear()
        self._log_configs.clear()

    def register_callback(self, callback):
        """注册回调函数"""
        self._callbacks.append(callback)

    def register_health_callback(self, callback):
        """注册健康回调函数"""
        self._health_callbacks.append(callback)

    def _on_pose_data(self, drone_id, data):
        """接收到定位数据"""
        if not self._running:
            return

        pos = np.array(
            [data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"]]
        )

        for cb in self._callbacks:
            cb(drone_id, pos, time.time())

    def _on_health_data(self, drone_id, data):
        if not self._running:
            return

        health = {"pm.vbat": float(data["pm.vbat"])}
        for cb in self._health_callbacks:
            cb(drone_id, health, time.time())

    def _logger_worker(self, drone_id, sync_logger):
        while self._running:
            try:
                _ts, data, _logblock = next(sync_logger)
            except StopIteration:
                logger.info("Lighthouse logger stopped for drone %s", drone_id)
                break
            except RuntimeError as exc:
                logger.warning("Lighthouse logger runtime failure for drone %s: %s", drone_id, exc)
                break
            except Exception as exc:
                logger.exception("Unexpected lighthouse logger failure for drone %s: %s", drone_id, exc)
                break

            if "stateEstimate.x" in data:
                self._on_pose_data(drone_id, data)
            if "pm.vbat" in data:
                self._on_health_data(drone_id, data)
