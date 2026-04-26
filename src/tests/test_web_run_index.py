"""RunIndex contracts —— 扫描 telemetry_dir 配对 artifacts_dir."""

import json
import tempfile
from datetime import datetime
from pathlib import Path

from src.web.data_access.run_index import RunArtifact, RunIndex, _parse_timestamp


# --- _parse_timestamp ---

assert _parse_timestamp("run_real_20260414_150922") == datetime(
    2026, 4, 14, 15, 9, 22
)
assert _parse_timestamp("run_real") is None
assert _parse_timestamp("anything_20260414_150922_suffix") == datetime(
    2026, 4, 14, 15, 9, 22
)
assert _parse_timestamp("bad_99999999_999999") is None  # invalid date


# --- RunIndex.list_runs ---

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    telemetry_dir = root / "telemetry"
    artifacts_dir = root / "artifacts"
    telemetry_dir.mkdir()
    artifacts_dir.mkdir()

    # 不存在的目录返回空
    missing_index = RunIndex(root / "missing", artifacts_dir)
    assert missing_index.list_runs() == []

    # 创建 3 个 run, 其中 2 个时间戳可解析 + 1 个无时间戳
    (telemetry_dir / "run_real_20260414_150922.jsonl").write_text("{}\n")
    (telemetry_dir / "run_real_20260414_163307.jsonl").write_text("{}\n")
    (telemetry_dir / "run_real.jsonl").write_text("{}\n")

    # 给其中一个 run 建 artifacts 目录
    (artifacts_dir / "run_real_20260414_150922").mkdir()
    (artifacts_dir / "run_real_20260414_150922" / "overview.png").write_text("x")

    index = RunIndex(telemetry_dir, artifacts_dir)
    runs = index.list_runs()

    assert len(runs) == 3
    # 时间倒序: 163307 > 150922 > run_real (后者用文件 mtime)
    # 注意 run_real 无时间戳用 mtime, 可能会在任意位置, 只要前两个顺序对即可
    timestamped = [r for r in runs if r.run_id.endswith(("150922", "163307"))]
    assert timestamped[0].run_id == "run_real_20260414_163307"
    assert timestamped[1].run_id == "run_real_20260414_150922"

    # 配对验证
    with_artifacts = [r for r in runs if r.run_id == "run_real_20260414_150922"][0]
    assert with_artifacts.has_artifacts
    assert with_artifacts.artifacts_dir == artifacts_dir / "run_real_20260414_150922"

    without_artifacts = [r for r in runs if r.run_id == "run_real_20260414_163307"][0]
    assert not without_artifacts.has_artifacts
    assert without_artifacts.artifacts_dir is None


# --- RunIndex.get ---

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    telemetry_dir = root / "telemetry"
    artifacts_dir = root / "artifacts"
    telemetry_dir.mkdir()
    artifacts_dir.mkdir()

    (telemetry_dir / "run_real_20260414_150922.jsonl").write_text("{}\n")

    index = RunIndex(telemetry_dir, artifacts_dir)

    found = index.get("run_real_20260414_150922")
    assert isinstance(found, RunArtifact)
    assert found.run_id == "run_real_20260414_150922"
    assert found.t_start == datetime(2026, 4, 14, 15, 9, 22)

    missing = index.get("does_not_exist")
    assert missing is None

print("OK test_web_run_index")
