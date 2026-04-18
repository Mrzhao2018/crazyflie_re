"""CflibLinkManager ↔ LinkQualityBus 接入契约：
cflib 的 cf.link_statistics 提供 6 个 Caller —— wire_link_quality_callbacks
必须把它们挂到 LinkQualityBus 对应字段上。
"""

from src.adapters.cflib_link_manager import wire_link_quality_callbacks
from src.runtime.link_quality_bus import LinkQualityBus


class FakeCaller:
    def __init__(self):
        self._callbacks = []

    def add_callback(self, cb):
        self._callbacks.append(cb)

    def call(self, value):
        for cb in self._callbacks:
            cb(value)


class FakeLinkStatistics:
    def __init__(self):
        self.link_quality_updated = FakeCaller()
        self.uplink_rssi_updated = FakeCaller()
        self.uplink_rate_updated = FakeCaller()
        self.downlink_rate_updated = FakeCaller()
        self.uplink_congestion_updated = FakeCaller()
        self.downlink_congestion_updated = FakeCaller()
        self.started = False

    def start(self):
        self.started = True


class FakeCF:
    def __init__(self):
        self.link_statistics = FakeLinkStatistics()


# ---- 正常接入 --------------------------------------------------------------

bus = LinkQualityBus()
cf = FakeCF()
now = [1000.0]

def fake_time():
    now[0] += 0.1
    return now[0]

wire_link_quality_callbacks(cf, drone_id=7, bus=bus, time_fn=fake_time)

# link_statistics.start 必须被调用，否则 cflib 不会触发回调
assert cf.link_statistics.started is True

# 模拟 cflib 触发每一个 Caller
cf.link_statistics.link_quality_updated.call(93.0)
cf.link_statistics.uplink_rssi_updated.call(-52.5)
cf.link_statistics.uplink_rate_updated.call(40.0)
cf.link_statistics.downlink_rate_updated.call(7.5)
cf.link_statistics.uplink_congestion_updated.call(10.0)
cf.link_statistics.downlink_congestion_updated.call(2.0)

latest = bus.latest()
assert 7 in latest
sample = latest[7]
assert sample.link_quality == 93.0
assert sample.uplink_rssi == -52.5
assert sample.uplink_rate == 40.0
assert sample.downlink_rate == 7.5
assert sample.uplink_congestion == 10.0
assert sample.downlink_congestion == 2.0
assert sample.last_update_t is not None

# ---- bus=None 时 start 仍被调用，回调不挂（避免无意义开销） ------------------

cf2 = FakeCF()
wire_link_quality_callbacks(cf2, drone_id=8, bus=None, time_fn=fake_time)
assert cf2.link_statistics.started is False  # bus=None 时不启动，避免白白采集
assert len(cf2.link_statistics.link_quality_updated._callbacks) == 0

# ---- 多 drone 隔离 -------------------------------------------------------

cf3 = FakeCF()
wire_link_quality_callbacks(cf3, drone_id=9, bus=bus, time_fn=fake_time)
cf3.link_statistics.link_quality_updated.call(75.0)

latest = bus.latest()
assert latest[7].link_quality == 93.0, "drone 7 应不受 drone 9 的回调影响"
assert latest[9].link_quality == 75.0

print("[OK] wire_link_quality_callbacks contracts verified")
