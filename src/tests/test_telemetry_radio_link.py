"""TelemetryRecord.radio_link_quality 契约：
新增字段必须 round-trip 通过 JSONL，且缺省值（空 dict）也能被写入。
"""

import json
import tempfile
from pathlib import Path

from src.runtime.telemetry import TelemetryRecorder, TelemetryRecord


def _base_record(seq: int, radio_link_quality=None) -> TelemetryRecord:
    return TelemetryRecord(
        t_wall=0.0,
        mission_state="RUN",
        startup_mode="auto",
        mission_elapsed=0.1 * seq,
        trajectory_state="running",
        trajectory_terminal_reason=None,
        snapshot_seq=seq,
        snapshot_t_meas=0.0,
        measured_positions={1: [0.0, 0.0, 0.5]},
        fresh_mask={1: True},
        disconnected_ids=[],
        health={1: {"pm.vbat": 4.0}},
        frame_valid=True,
        frame_condition_number=1.0,
        phase_label="formation_run",
        leader_mode="batch_goto",
        leader_reference_positions={1: [0.1, 0.0, 0.5]},
        follower_reference_positions={5: [0.0, 0.2, 0.5]},
        safety_action="EXECUTE",
        safety_reasons=[],
        safety_reason_codes=[],
        scheduler_reason="execute",
        scheduler_diagnostics={"fresh": True},
        leader_reference_source="L",
        manual_axis=None,
        manual_input_age=None,
        leader_action_count=1,
        follower_action_count=2,
        follower_command_norms={5: 0.1},
        radio_link_quality=radio_link_quality if radio_link_quality is not None else {},
    )


with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp) / "t.jsonl"
    rec = TelemetryRecorder()
    rec.open(str(path))
    rec.write_header(config_fingerprint={"x": 1})

    # 携带 radio_link_quality 的 record
    link_payload = {
        1: {
            "link_quality": 92.5,
            "uplink_rssi": -55.0,
            "uplink_rate": 42.0,
            "downlink_rate": 8.0,
            "uplink_congestion": 12.5,
            "downlink_congestion": 3.0,
            "last_update_t": 1001.4,
        },
        2: {
            "link_quality": 60.0,
        },
    }
    rec.log(_base_record(0, radio_link_quality=link_payload))
    # 缺省（空 dict）的 record
    rec.log(_base_record(1))
    rec.close()

    lines = path.read_text(encoding="utf-8").splitlines()
    records = [json.loads(line) for line in lines if json.loads(line)["kind"] == "record"]
    assert len(records) == 2

    first = records[0]
    assert "radio_link_quality" in first, "radio_link_quality 字段应出现在 record 中"
    # dict 键在 JSON 序列化后会变成 string
    link_in = first["radio_link_quality"]
    assert link_in["1"]["link_quality"] == 92.5
    assert link_in["1"]["uplink_rssi"] == -55.0
    assert link_in["2"]["link_quality"] == 60.0
    assert "uplink_rssi" not in link_in["2"] or link_in["2"].get("uplink_rssi") is None

    second = records[1]
    assert second["radio_link_quality"] == {}, "缺省值应当是空 dict"

print("[OK] TelemetryRecord.radio_link_quality round-trip verified")
