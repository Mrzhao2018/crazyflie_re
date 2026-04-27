"""analyze_records 新增 radio_link_summary：
当 record 含 radio_link_quality 字段时，按 drone 聚合 link_quality min/p5/mean/count。
"""

from src.runtime.telemetry_replay import analyze_records


def _record(seq: int, link: dict | None) -> dict:
    return {
        "snapshot_seq": seq,
        "snapshot_t_meas": 0.1 * seq,
        "safety_action": "EXECUTE",
        "scheduler_reason": "execute",
        "measured_positions": {},
        "leader_reference_positions": {},
        "follower_reference_positions": {},
        "fresh_mask": {},
        "mission_state": "RUN",
        "phase_label": "formation_run",
        "follower_command_norms": {},
        "radio_link_quality": link if link is not None else {},
        "scheduler_diagnostics": {
            "group_health_scores": {0: 85.0 - seq, 1: 45.0 + seq},
            "link_quality_backoff_groups": [1] if seq % 2 == 0 else [],
        },
    }


records = [
    _record(0, {1: {"link_quality": 95.0}, 2: {"link_quality": 80.0}}),
    _record(1, {1: {"link_quality": 90.0}, 2: {"link_quality": 50.0}}),
    _record(2, {1: {"link_quality": 92.0}, 2: {"link_quality": 70.0}}),
    _record(3, {1: {"link_quality": 88.0}, 2: {"link_quality": 60.0}}),
]

summary = analyze_records(records)
assert "radio_link_summary" in summary, "analyze_records 必须暴露 radio_link_summary"

link_summary = summary["radio_link_summary"]
# per_drone 必须按 drone_id 字符串（JSON 字典键统一成 str）聚合
per_drone = link_summary["per_drone"]
assert "1" in per_drone and "2" in per_drone
assert per_drone["1"]["count"] == 4
assert per_drone["1"]["mean"] == (95.0 + 90.0 + 92.0 + 88.0) / 4
assert per_drone["1"]["min"] == 88.0
assert per_drone["2"]["min"] == 50.0
assert link_summary["per_group"]["1"]["min_score"] == 45.0
assert link_summary["per_group"]["1"]["backoff_count"] == 2

# 整体 min / mean 应覆盖所有 drone 的样本
overall = link_summary["overall"]
assert overall["count"] == 8, "overall count 应为所有 drone 样本数之和"
assert overall["min"] == 50.0

# 无 link_quality 数据的 run 不应崩
empty_summary = analyze_records([_record(0, None)])
assert empty_summary["radio_link_summary"]["overall"]["count"] == 0
assert empty_summary["radio_link_summary"]["per_drone"] == {}

# 旧格式：不含 radio_link_quality 字段的 record 向后兼容
legacy_records = [
    {
        "snapshot_seq": 0,
        "snapshot_t_meas": 0.0,
        "safety_action": "EXECUTE",
        "scheduler_reason": "execute",
        "measured_positions": {},
        "leader_reference_positions": {},
        "follower_reference_positions": {},
        "fresh_mask": {},
        "mission_state": "RUN",
        "phase_label": "formation_run",
        "follower_command_norms": {},
    }
]
legacy_summary = analyze_records(legacy_records)
assert legacy_summary["radio_link_summary"]["overall"]["count"] == 0

print("[OK] analyze_records.radio_link_summary contracts verified")
