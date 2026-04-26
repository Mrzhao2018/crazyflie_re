"""TelemetryRecorder queue/flush config wiring contracts."""

import tempfile
from pathlib import Path

import yaml

from src.app.bootstrap import build_core_app
from src.runtime.telemetry import TelemetryRecorder


def _copy_config_tree(src_dir: Path, dst_dir: Path, *, comm_overrides: dict) -> None:
    for name in ("fleet.yaml", "mission.yaml", "safety.yaml", "startup.yaml"):
        (dst_dir / name).write_text(
            (src_dir / name).read_text(encoding="utf-8"), encoding="utf-8"
        )
    comm = yaml.safe_load((src_dir / "comm.yaml").read_text(encoding="utf-8"))
    comm.update(comm_overrides)
    (dst_dir / "comm.yaml").write_text(yaml.safe_dump(comm), encoding="utf-8")


rec = TelemetryRecorder(flush_every_n=3, queue_max_size=7)
assert rec._flush_every_n == 3
assert rec._queue_max_size == 7
assert rec._queue.maxsize == 7
rec.close()

with tempfile.TemporaryDirectory() as tmp_str:
    tmp = Path(tmp_str)
    _copy_config_tree(
        Path("config"),
        tmp,
        comm_overrides={"telemetry_queue_max": 32, "telemetry_flush_every_n": 3},
    )
    components = build_core_app(str(tmp))
    telemetry = components["telemetry"]
    assert isinstance(telemetry, TelemetryRecorder)
    assert telemetry._flush_every_n == 3
    assert telemetry._queue_max_size == 32
    assert telemetry._queue.maxsize == 32

print("[OK] TelemetryRecorder queue/flush config wiring verified")
