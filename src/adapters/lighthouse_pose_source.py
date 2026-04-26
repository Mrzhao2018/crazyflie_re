"""Lighthouse定位数据源。

单 drone 三路 LogConfig（cflib 每块 payload ≤ 26B 限制）：

* pose (10 Hz, 24B) — stateEstimate.x/y/z + vx/vy/vz
* health (2 Hz, 24B) — pm.vbat + kalman.varPX/Y/Z + motor.m1~m4
* attitude (5 Hz, 14B) — stateEstimate.roll/pitch/yaw + stabilizer.thrust

历史实现使用 ``SyncLogger`` + 每机一条 worker 线程。当前实现改为直接按
Bitcraze 官方 Python API 文档的推荐范式挂 ``LogConfig.data_received_cb``：

    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(cb)
    log_conf.start()

这样 cflib 的 incoming 线程收到 log packet 后会直接调我们的 callback，不再额外
经过 ``SyncLogger`` 队列和 Python worker 线程。外部 API 保持不变：

* pose 推 PoseBus
* health / attitude 合并推 HealthBus（merge 语义）
* ``snapshot.t_meas`` 仍继续使用 ``time.time()`` 的 wall-clock

同时把 cflib 回调里的 ``timestamp``（firmware log tick，毫秒）作为
``cf_log_timestamp_ms`` 附带到 health/attitude payload，便于离线诊断链路拥塞
或 host 回调延迟。
"""

from __future__ import annotations

import logging
import time

import numpy as np

logger = logging.getLogger(__name__)


class LighthousePoseSource:
    """从Lighthouse读取定位数据"""

    def __init__(
        self,
        link_manager,
        fleet_model,
        log_freq_hz=10.0,
        *,
        attitude_log_enabled: bool = True,
        motor_log_enabled: bool = True,
    ):
        self.link_manager = link_manager
        self.fleet = fleet_model
        self.log_period_ms = int(1000.0 / log_freq_hz)
        self.attitude_log_enabled = bool(attitude_log_enabled)
        self.motor_log_enabled = bool(motor_log_enabled)
        self._callbacks = []
        self._health_callbacks = []
        self._log_configs: dict[int, list[object]] = {}
        self._data_callbacks: dict[int, list[tuple[object, object]]] = {}
        self._running = False

    def start(self):
        """启动定位数据流"""
        self._running = True
        self._log_configs.clear()
        self._data_callbacks.clear()

        for drone_id in self.fleet.all_ids():
            try:
                self._attach_drone(drone_id)
            except RuntimeError as exc:
                logger.error("Lighthouse logger connect failed for drone %s: %s", drone_id, exc)
                self.stop()
                raise
            except Exception as exc:
                logger.error("Unexpected logger connect failure for drone %s: %s", drone_id, exc)
                self.stop()
                raise

        logger.info("Lighthouse pose source started")

    def stop(self):
        """停止定位数据流"""
        self._running = False
        for drone_id in list(self._data_callbacks):
            self._detach_drone(drone_id)
        self._data_callbacks.clear()
        self._log_configs.clear()

    def reattach_drone(self, drone_id: int) -> bool:
        """Reconnect 成功后，把该 drone 的 LogConfig 重新绑定到新的 ``scf.cf``。"""
        if not self._running:
            return False
        self._detach_drone(drone_id)
        self._attach_drone(drone_id)
        return True

    def register_callback(self, callback):
        """注册回调函数 —— 接受 (drone_id, pos, ts, velocity=None)"""
        self._callbacks.append(callback)

    def register_health_callback(self, callback):
        """注册健康回调函数 —— 接受 (drone_id, values_dict, ts)"""
        self._health_callbacks.append(callback)

    def _build_configs_and_callbacks(self, drone_id: int):
        from cflib.crazyflie.log import LogConfig

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
        if self.motor_log_enabled:
            health_conf.add_variable("motor.m1", "uint16_t")
            health_conf.add_variable("motor.m2", "uint16_t")
            health_conf.add_variable("motor.m3", "uint16_t")
            health_conf.add_variable("motor.m4", "uint16_t")

        configs_and_callbacks = [
            (pose_conf, self._make_log_callback(drone_id, "pose")),
            (health_conf, self._make_log_callback(drone_id, "health")),
        ]

        if self.attitude_log_enabled:
            attitude_conf = LogConfig(
                name=f"attitude_{drone_id}",
                period_in_ms=max(200, self.log_period_ms * 2),
            )
            attitude_conf.add_variable("stateEstimate.roll", "float")
            attitude_conf.add_variable("stateEstimate.pitch", "float")
            attitude_conf.add_variable("stateEstimate.yaw", "float")
            attitude_conf.add_variable("stabilizer.thrust", "float")
            configs_and_callbacks.append(
                (attitude_conf, self._make_log_callback(drone_id, "attitude"))
            )

        return configs_and_callbacks

    def _attach_drone(self, drone_id: int) -> None:
        scf = self.link_manager.get(drone_id)
        cf = scf.cf
        configs_and_callbacks = self._build_configs_and_callbacks(drone_id)
        self._log_configs[drone_id] = [
            log_conf for log_conf, _callback in configs_and_callbacks
        ]
        self._data_callbacks[drone_id] = configs_and_callbacks
        for log_conf, callback in configs_and_callbacks:
            cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(callback)
            log_conf.start()

    def _detach_drone(self, drone_id: int) -> None:
        bindings = self._data_callbacks.pop(drone_id, [])
        for log_conf, callback in bindings:
            try:
                log_conf.stop()
            except RuntimeError as exc:
                logger.warning(
                    "Stopping lighthouse log config failed for drone %s: %s",
                    drone_id,
                    exc,
                )
            except Exception as exc:
                logger.warning(
                    "Unexpected lighthouse log stop failure for drone %s: %s",
                    drone_id,
                    exc,
                )
            try:
                log_conf.data_received_cb.remove_callback(callback)
            except Exception:
                logger.exception(
                    "Failed to detach lighthouse log callback for drone %s",
                    drone_id,
                )
            try:
                log_conf.delete()
            except RuntimeError as exc:
                logger.warning(
                    "Deleting lighthouse log config failed for drone %s: %s",
                    drone_id,
                    exc,
                )
            except Exception as exc:
                logger.warning(
                    "Unexpected lighthouse log delete failure for drone %s: %s",
                    drone_id,
                    exc,
                )
        self._log_configs.pop(drone_id, None)

    def _make_log_callback(self, drone_id: int, stream_kind: str):
        def _callback(timestamp, data, logconf):
            try:
                self._dispatch_log_packet(
                    drone_id=drone_id,
                    stream_kind=stream_kind,
                    timestamp=timestamp,
                    data=data,
                    logconf=logconf,
                )
            except Exception:
                logger.exception(
                    "Lighthouse %s callback failed for drone %s",
                    stream_kind,
                    drone_id,
                )

        return _callback

    def _dispatch_log_packet(self, *, drone_id, stream_kind, timestamp, data, logconf):
        if not self._running:
            return
        if stream_kind == "pose":
            self._on_pose_data(drone_id, data)
        elif stream_kind == "health":
            self._on_health_data(drone_id, data, timestamp)
        elif stream_kind == "attitude":
            self._on_attitude_data(drone_id, data, timestamp)
        else:
            logger.warning(
                "Unexpected lighthouse log stream %s for drone %s (%s)",
                stream_kind,
                drone_id,
                getattr(logconf, "name", None),
            )

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

    def _on_health_data(self, drone_id, data, log_timestamp_ms):
        """health block 触发 —— pm.vbat / kalman.var* / motor.m1~m4"""
        if not self._running:
            return
        values = {k: _coerce_scalar(v) for k, v in data.items()}
        values["cf_log_timestamp_ms"] = _coerce_scalar(log_timestamp_ms)
        ts = time.time()
        for cb in self._health_callbacks:
            cb(drone_id, values, ts)

    def _on_attitude_data(self, drone_id, data, log_timestamp_ms):
        """attitude block 触发 —— roll/pitch/yaw/thrust"""
        if not self._running:
            return
        values = {k: _coerce_scalar(v) for k, v in data.items()}
        values["cf_log_timestamp_ms"] = _coerce_scalar(log_timestamp_ms)
        ts = time.time()
        for cb in self._health_callbacks:
            cb(drone_id, values, ts)


def _coerce_scalar(v):
    """把 cflib 返回的 numpy 标量 / python number 统一成 float。"""
    try:
        return float(v)
    except (TypeError, ValueError):
        return v
