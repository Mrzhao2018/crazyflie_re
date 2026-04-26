"""TelemetryLoader contracts —— 复用 telemetry_replay + LRU 缓存."""

import json
import tempfile
from pathlib import Path

from src.web.data_access.telemetry_loader import LoadedRun, TelemetryLoader


HEADER_LINE = {
    "kind": "header",
    "schema_version": 2,
    "t_wall": 0.0,
    "config_fingerprint": {"config_sha256": "abc"},
}

RECORD_TEMPLATE = {
    "kind": "record",
    "t_wall": 0.1,
    "mission_state": "formation_run",
    "startup_mode": "auto",
    "mission_elapsed": 0.1,
    "trajectory_state": None,
    "trajectory_terminal_reason": None,
    "snapshot_seq": 1,
    "snapshot_t_meas": 0.0,
    "measured_positions": {"1": [0.0, 0.0, 1.0]},
    "fresh_mask": {"1": True},
    "disconnected_ids": [],
    "health": {},
    "frame_valid": True,
    "frame_condition_number": 1.2,
    "phase_label": "formation_run",
    "leader_mode": "hold",
    "leader_reference_positions": {"1": [0.0, 0.0, 1.0]},
    "follower_reference_positions": {},
    "safety_action": "EXECUTE",
    "safety_reasons": [],
    "safety_reason_codes": [],
    "scheduler_reason": None,
    "scheduler_diagnostics": {},
    "leader_reference_source": None,
    "manual_axis": None,
    "manual_input_age": None,
    "leader_action_count": 0,
    "follower_action_count": 0,
    "follower_command_norms": {},
    "radio_link_quality": {},
}


def _write_jsonl(path: Path, records: int, phase: str = "formation_run") -> None:
    lines = [HEADER_LINE]
    for i in range(records):
        record = dict(RECORD_TEMPLATE)
        record["snapshot_seq"] = i + 1
        record["snapshot_t_meas"] = 0.05 * i
        record["phase_label"] = phase
        lines.append(record)
    path.write_text("\n".join(json.dumps(line) for line in lines) + "\n")


# --- load() ---

with tempfile.TemporaryDirectory() as tmp:
    jsonl = Path(tmp) / "run.jsonl"
    _write_jsonl(jsonl, records=5)

    loader = TelemetryLoader(cache_size=2)
    run = loader.load(jsonl)
    assert isinstance(run, LoadedRun)
    assert run.summary["record_count"] == 5
    assert run.summary["last_mission_state"] == "formation_run"
    # 缓存命中 (同一对象)
    again = loader.load(jsonl)
    assert again is run


# --- cache eviction ---

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    p1 = root / "run1.jsonl"
    p2 = root / "run2.jsonl"
    p3 = root / "run3.jsonl"
    _write_jsonl(p1, records=1)
    _write_jsonl(p2, records=2)
    _write_jsonl(p3, records=3)

    loader = TelemetryLoader(cache_size=2)
    r1 = loader.load(p1)
    r2 = loader.load(p2)
    r3 = loader.load(p3)  # evicts r1
    # r2, r3 仍缓存; r1 被挤出, 再 load 会是新对象
    assert loader.load(p2) is r2
    assert loader.load(p3) is r3
    new_r1 = loader.load(p1)
    assert new_r1 is not r1
    assert new_r1.summary["record_count"] == 1


# --- summary_for_phase ---

with tempfile.TemporaryDirectory() as tmp:
    jsonl = Path(tmp) / "mixed.jsonl"
    lines = [HEADER_LINE]
    for i, phase in enumerate(["settle", "formation_run", "formation_run", "land"]):
        record = dict(RECORD_TEMPLATE)
        record["snapshot_seq"] = i + 1
        record["snapshot_t_meas"] = 0.05 * i
        record["phase_label"] = phase
        lines.append(record)
    jsonl.write_text("\n".join(json.dumps(line) for line in lines) + "\n")

    loader = TelemetryLoader()
    run = loader.load(jsonl)

    # None: 原始 summary
    assert loader.summary_for_phase(run, None) is run.summary

    # 指定 phase: formation_error 等被替换为该 phase 切片
    formation = loader.summary_for_phase(run, "formation_run")
    assert formation["filtered_phase"] == "formation_run"
    # formation_error 来自 phase_tracking_error["formation_run"], 至少 count>0
    assert formation["formation_error"]["count"] >= 1

    # 不存在的 phase: 返回空 stats
    missing = loader.summary_for_phase(run, "nonexistent_phase")
    assert missing["filtered_phase"] == "nonexistent_phase"
    assert missing["formation_error"]["count"] == 0
    assert missing["role_tracking_error"]["leader"]["count"] == 0


# --- cache_size 边界 ---

try:
    TelemetryLoader(cache_size=0)
except ValueError:
    pass
else:
    raise AssertionError("cache_size=0 must raise ValueError")

print("OK test_web_telemetry_loader")
