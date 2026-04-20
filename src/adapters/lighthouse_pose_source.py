"""Lighthouse定位数据源

单 drone 三路 LogConfig（cflib 每块 payload ≤ 26B 限制）：

* pose (10 Hz, 24B) — stateEstimate.x/y/z + vx/vy/vz
* health (2 Hz, 24B) — pm.vbat + kalman.varPX/Y/Z + motor.m1~m4
* attitude (5 Hz, 14B) — stateEstimate.roll/pitch/yaw + stabilizer.thrust

pose 推 PoseBus；health / attitude 合并推 HealthBus（后者 update 是 merge 语义）。
"""

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
            pose_conf.add_variable("stateEstimate.vx", "float")
            pose_conf.add_variable("stateEstimate.vy", "float")
            pose_conf.add_variable("stateEstimate.vz", "float")

            health_conf = LogConfig(
                name=f"health_{drone_id}", period_in_ms=max(200, self.log_period_ms * 5)
            )
            health_conf.add_variable("pm.vbat", "float")
            health_conf.add_variable("kalman.varPX", "float")
            health_conf.add_variable("kalman.varPY", "float")
            health_conf.add_variable("kalman.varPZ", "float")
            health_conf.add_variable("motor.m1", "uint16_t")
            health_conf.add_variable("motor.m2", "uint16_t")
            health_conf.add_variable("motor.m3", "uint16_t")
            health_conf.add_variable("motor.m4", "uint16_t")

            attitude_conf = LogConfig(
                name=f"attitude_{drone_id}", period_in_ms=max(200, self.log_period_ms * 2)
            )
            attitude_conf.add_variable("stateEstimate.roll", "float")
            attitude_conf.add_variable("stateEstimate.pitch", "float")
            attitude_conf.add_variable("stateEstimate.yaw", "float")
            attitude_conf.add_variable("stabilizer.thrust", "float")

            sync_logger = SyncLogger(scf, [pose_conf, health_conf, attitude_conf])
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
            self._log_configs[drone_id] = [pose_conf, health_conf, attitude_conf]
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
        """注册回调函数 —— 接受 (drone_id, pos, ts, velocity=None)"""
        self._callbacks.append(callback)

    def register_health_callback(self, callback):
        """注册健康回调函数 —— 接受 (drone_id, values_dict, ts)"""
        self._health_callbacks.append(callback)

    def _on_pose_data(self, drone_id, data):
        """pose block 触发"""
        if not self._running:
            return

        pos = np.array(
            [data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"]]
        )
        vel = None
        if "stateEstimate.vx" in data:
            vel = np.array(
                [
                    data["stateEstimate.vx"],
                    data["stateEstimate.vy"],
                    data["stateEstimate.vz"],
                ]
            )

        ts = time.time()
        for cb in self._callbacks:
            try:
                cb(drone_id, pos, ts, vel)
            except TypeError:
                cb(drone_id, pos, ts)

    def _on_health_data(self, drone_id, data):
        """health block 触发 —— pm.vbat / kalman.var* / motor.m1~m4"""
        if not self._running:
            return
        values = {k: _coerce_scalar(v) for k, v in data.items()}
        ts = time.time()
        for cb in self._health_callbacks:
            cb(drone_id, values, ts)

    def _on_attitude_data(self, drone_id, data):
        """attitude block 触发 —— roll/pitch/yaw/thrust"""
        if not self._running:
            return
        values = {k: _coerce_scalar(v) for k, v in data.items()}
        ts = time.time()
        for cb in self._health_callbacks:
            cb(drone_id, values, ts)

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
            elif "stateEstimate.roll" in data:
                self._on_attitude_data(drone_id, data)
            elif "pm.vbat" in data or "kalman.varPX" in data:
                self._on_health_data(drone_id, data)


def _coerce_scalar(v):
    """把 cflib 返回的 numpy 标量 / python number 统一成 float。"""
    try:
        return float(v)
    except (TypeError, ValueError):
        return v
