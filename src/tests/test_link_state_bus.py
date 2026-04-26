from src.runtime.link_state_bus import LinkStateBus


bus = LinkStateBus()
bus.mark_connected(1, t_wall=1.0)
assert bus.latest()[1].state == "connected"
assert bus.disconnected_ids() == []

events = bus.drain_events()
assert events == [
    {"drone_id": 1, "state": "connected", "t_wall": 1.0, "error": None}
]
assert bus.drain_events() == []

# 重复 connected 不应重复产生日志洪水
bus.mark_connected(1, t_wall=2.0)
assert bus.drain_events() == []

bus.mark_disconnected(1, error="radio lost", t_wall=3.0)
assert bus.latest()[1].state == "disconnected"
assert bus.latest()[1].error == "radio lost"
assert bus.disconnected_ids() == [1]

events = bus.drain_events()
assert events == [
    {
        "drone_id": 1,
        "state": "disconnected",
        "t_wall": 3.0,
        "error": "radio lost",
    }
]

# 同状态但不同 error 应记录一次新事件，避免丢失错误上下文
bus.mark_disconnected(1, error="timeout", t_wall=4.0)
assert bus.drain_events()[0]["error"] == "timeout"

print("[OK] LinkStateBus contracts verified")
