"""ConfigLoader full_state invariants."""

import tempfile
from pathlib import Path

import yaml

from src.config.loader import ConfigLoader


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    for config_path in Path("config").glob("*.yaml"):
        root.joinpath(config_path.name).write_text(
            config_path.read_text(encoding="utf-8"),
            encoding="utf-8",
        )

    fleet_data = yaml.safe_load((root / "fleet.yaml").read_text(encoding="utf-8"))
    fleet_data["control"]["output_mode"] = "full_state"
    fleet_data["control"]["onboard_controller"] = "mellinger"
    fleet_data["control"]["dynamics_model_order"] = 1
    (root / "fleet.yaml").write_text(
        yaml.safe_dump(fleet_data, sort_keys=False),
        encoding="utf-8",
    )

    try:
        ConfigLoader.load(str(root))
    except ValueError as exc:
        assert "full_state" in str(exc)
        assert "dynamics_model_order" in str(exc)
    else:
        raise AssertionError("Expected full_state controller-path validation to fail")

with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    for config_path in Path("config").glob("*.yaml"):
        root.joinpath(config_path.name).write_text(
            config_path.read_text(encoding="utf-8"),
            encoding="utf-8",
        )

    safety_data = yaml.safe_load((root / "safety.yaml").read_text(encoding="utf-8"))
    safety_data["pose_timeout"] = 0.15
    (root / "safety.yaml").write_text(
        yaml.safe_dump(safety_data, sort_keys=False),
        encoding="utf-8",
    )

    try:
        ConfigLoader.load(str(root))
    except ValueError as exc:
        assert "health" in str(exc).lower() or "pose_timeout" in str(exc)
    else:
        raise AssertionError("Expected health freshness cadence validation to fail")

print("[OK] Full-state config validation verified")
