"""LinkQualityBus contract tests"""

import threading

from src.runtime.link_quality_bus import LinkQualityBus, LinkQualitySample


bus = LinkQualityBus()
assert bus.latest() == {}, "empty bus should return empty dict"

# 单指标更新
bus.update(1, "link_quality", 92.5, 1000.0)
latest = bus.latest()
assert 1 in latest, "drone_id 1 missing after update"
sample = latest[1]
assert isinstance(sample, LinkQualitySample), "latest() values must be LinkQualitySample"
assert sample.link_quality == 92.5
assert sample.last_update_t == 1000.0
assert sample.uplink_rssi is None, "uninitialized metrics should be None"

# 同一架机多指标叠加 —— 保留已有指标，不覆盖
bus.update(1, "uplink_rssi", -55.0, 1000.5)
bus.update(1, "uplink_rate", 42.0, 1001.0)
bus.update(1, "downlink_rate", 8.0, 1001.2)
bus.update(1, "uplink_congestion", 12.5, 1001.3)
bus.update(1, "downlink_congestion", 3.0, 1001.4)
latest = bus.latest()
sample = latest[1]
assert sample.link_quality == 92.5, "先前的 link_quality 应当保留"
assert sample.uplink_rssi == -55.0
assert sample.uplink_rate == 42.0
assert sample.downlink_rate == 8.0
assert sample.uplink_congestion == 12.5
assert sample.downlink_congestion == 3.0
assert sample.last_update_t == 1001.4, "last_update_t 应为最新一次 update"

# 新指标数值覆盖旧数值
bus.update(1, "link_quality", 78.0, 1002.0)
assert bus.latest()[1].link_quality == 78.0
assert bus.latest()[1].last_update_t == 1002.0

# 多机互不干扰
bus.update(2, "link_quality", 60.0, 1002.5)
bus.update(2, "uplink_congestion", 85.0, 1002.6)
bus.update(2, "latency_ms", 90.0, 1002.7)
bus.update(2, "uplink_congestion", 10.0, 1015.0)
latest = bus.latest()
assert latest[1].link_quality == 78.0, "drone 1 应不受 drone 2 更新影响"
assert latest[2].link_quality == 60.0
assert latest[2].uplink_rssi is None
assert latest[2].latency_ms == 90.0

# latest() 返回快照，外部修改不影响内部
snap = bus.latest()
snap[99] = LinkQualitySample(link_quality=1.0)
assert 99 not in bus.latest(), "latest() 返回的 dict 应与内部隔离"

# 未知 metric 名称被忽略（不抛错，保持健壮）
bus.update(1, "unknown_metric", 42.0, 1003.0)
latest = bus.latest()
assert latest[1].link_quality == 78.0, "未知 metric 不应影响已有值"

# 线程安全 —— 回调可能从多个 radio driver 线程同时到达
def writer(drone_id: int):
    for i in range(200):
        bus.update(drone_id, "link_quality", float(i), 2000.0 + i / 1000.0)


threads = [threading.Thread(target=writer, args=(d,)) for d in (10, 11, 12)]
for t in threads:
    t.start()
for t in threads:
    t.join()

latest = bus.latest()
for d in (10, 11, 12):
    assert d in latest, f"并发写入后 drone {d} 丢失"
    assert latest[d].link_quality == 199.0, f"drone {d} 最终 link_quality 应为最后写入值"


class FakeFleet:
    def get_radio_group(self, drone_id):
        return {1: 0, 2: 0, 10: 1, 11: 1, 12: 1}[drone_id]


summary = bus.group_health_summary(
    FakeFleet(),
    window_s=2000.0,
    congestion_soft_floor=60.0,
    latency_p95_soft_limit_ms=50.0,
)
assert 0 in summary
assert summary[0]["drone_count"] == 2

fresh_summary = bus.group_health_summary(
    type("FreshFleet", (), {"get_radio_group": lambda self, drone_id: 2 if drone_id == 2 else 0})(),
    window_s=1000.0,
    congestion_soft_floor=60.0,
    latency_p95_soft_limit_ms=50.0,
)
assert fresh_summary[2]["min_link_quality"] is None
assert fresh_summary[2]["max_congestion"] == 10.0

print("[OK] LinkQualityBus contracts verified")
