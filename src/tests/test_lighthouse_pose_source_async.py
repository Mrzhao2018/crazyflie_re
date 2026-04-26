from __future__ import annotations

import numpy as np

from src.adapters.lighthouse_pose_source import LighthousePoseSource


class _FakeCaller:
    def __init__(self):
        self._callbacks = []

    def add_callback(self, cb):
        self._callbacks.append(cb)

    def remove_callback(self, cb):
        self._callbacks.remove(cb)

    def call(self, *args):
        for cb in list(self._callbacks):
            cb(*args)


class _FakeLogConfig:
    def __init__(self, name, period_in_ms):
        self.name = name
        self.period_in_ms = period_in_ms
        self.variables = []
        self.data_received_cb = _FakeCaller()
        self.started = False
        self.stopped = False
        self.deleted = False

    def add_variable(self, name, fetch_as):
        self.variables.append((name, fetch_as))

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def delete(self):
        self.deleted = True


class _FakeLogSubsystem:
    def __init__(self):
        self.configs = []

    def add_config(self, log_conf):
        self.configs.append(log_conf)


class _FakeCF:
    def __init__(self):
        self.log = _FakeLogSubsystem()


class _FakeSCF:
    def __init__(self):
        self.cf = _FakeCF()


class _FakeLinkManager:
    def __init__(self):
        self._scf = _FakeSCF()

    def get(self, drone_id):
        return self._scf

    def replace_scf(self):
        self._scf = _FakeSCF()
        return self._scf


class _FakeFleet:
    def all_ids(self):
        return [7]


def test_lighthouse_pose_source_async(monkeypatch):
    monkeypatch.setattr("cflib.crazyflie.log.LogConfig", _FakeLogConfig)

    link_manager = _FakeLinkManager()
    pose_source = LighthousePoseSource(link_manager, _FakeFleet(), log_freq_hz=10.0)

    pose_updates = []
    health_updates = []
    pose_source.register_callback(
        lambda drone_id, pos, ts, velocity=None: pose_updates.append(
            (drone_id, pos, ts, velocity)
        )
    )
    pose_source.register_health_callback(
        lambda drone_id, values, ts: health_updates.append((drone_id, values, ts))
    )
    # 异步 callback 内部必须吞掉异常，不能把 cflib incoming 线程打挂
    pose_source.register_health_callback(
        lambda drone_id, values, ts: (_ for _ in ()).throw(RuntimeError("boom"))
    )

    pose_source.start()

    configs = link_manager._scf.cf.log.configs
    assert [cfg.name for cfg in configs] == ["pose_7", "health_7", "attitude_7"]
    assert all(cfg.started is True for cfg in configs)
    pose_conf, health_conf, attitude_conf = configs

    pose_conf.data_received_cb.call(
        123,
        {
            "stateEstimate.x": 1.0,
            "stateEstimate.y": 2.0,
            "stateEstimate.z": 3.0,
            "stateEstimate.vx": 0.1,
            "stateEstimate.vy": 0.2,
            "stateEstimate.vz": 0.3,
        },
        pose_conf,
    )
    health_conf.data_received_cb.call(
        456,
        {
            "pm.vbat": 4.1,
            "kalman.varPX": 0.001,
            "kalman.varPY": 0.002,
            "kalman.varPZ": 0.003,
        },
        health_conf,
    )
    attitude_conf.data_received_cb.call(
        789,
        {
            "stateEstimate.roll": 1.0,
            "stateEstimate.pitch": 2.0,
            "stateEstimate.yaw": 3.0,
            "stabilizer.thrust": 40000.0,
        },
        attitude_conf,
    )

    assert len(pose_updates) == 1
    drone_id, pos, _ts, vel = pose_updates[0]
    assert drone_id == 7
    assert isinstance(pos, np.ndarray)
    assert pos.tolist() == [1.0, 2.0, 3.0]
    assert isinstance(vel, np.ndarray)
    assert vel.tolist() == [0.1, 0.2, 0.3]

    assert len(health_updates) == 2
    assert health_updates[0][0] == 7
    assert health_updates[0][1]["pm.vbat"] == 4.1
    assert health_updates[0][1]["cf_log_timestamp_ms"] == 456.0
    assert health_updates[1][1]["stateEstimate.roll"] == 1.0
    assert health_updates[1][1]["cf_log_timestamp_ms"] == 789.0

    pose_source.stop()
    assert all(cfg.stopped is True for cfg in configs)
    assert all(cfg.deleted is True for cfg in configs)
    assert all(len(cfg.data_received_cb._callbacks) == 0 for cfg in configs)


def test_lighthouse_pose_source_reattach_drone_rebinds_log_configs(monkeypatch):
    monkeypatch.setattr("cflib.crazyflie.log.LogConfig", _FakeLogConfig)

    link_manager = _FakeLinkManager()
    pose_source = LighthousePoseSource(link_manager, _FakeFleet(), log_freq_hz=10.0)

    pose_source.start()
    old_configs = list(link_manager._scf.cf.log.configs)
    assert [cfg.name for cfg in old_configs] == ["pose_7", "health_7", "attitude_7"]

    new_scf = link_manager.replace_scf()
    assert pose_source.reattach_drone(7) is True

    assert all(cfg.stopped is True for cfg in old_configs)
    assert all(cfg.deleted is True for cfg in old_configs)
    assert all(len(cfg.data_received_cb._callbacks) == 0 for cfg in old_configs)
    assert [cfg.name for cfg in new_scf.cf.log.configs] == [
        "pose_7",
        "health_7",
        "attitude_7",
    ]
    assert all(cfg.started is True for cfg in new_scf.cf.log.configs)

    pose_source.stop()


def test_lighthouse_pose_source_can_disable_diagnostic_streams(monkeypatch):
    monkeypatch.setattr("cflib.crazyflie.log.LogConfig", _FakeLogConfig)

    link_manager = _FakeLinkManager()
    pose_source = LighthousePoseSource(
        link_manager,
        _FakeFleet(),
        log_freq_hz=10.0,
        attitude_log_enabled=False,
        motor_log_enabled=False,
    )

    pose_source.start()

    configs = link_manager._scf.cf.log.configs
    assert [cfg.name for cfg in configs] == ["pose_7", "health_7"]
    health_conf = configs[1]
    assert ("pm.vbat", "float") in health_conf.variables
    assert ("kalman.varPX", "float") in health_conf.variables
    assert not any(name.startswith("motor.") for name, _ in health_conf.variables)

    pose_source.stop()
