"""Crazyswarm2 sim YAML generation contracts."""

import tempfile
from pathlib import Path

import yaml

from src.adapters.crazyswarm_sim_config import (
    build_crazyswarm2_yaml_data,
    generate_crazyswarm2_yaml,
)

data = build_crazyswarm2_yaml_data("config")

assert data["fileversion"] == 3
assert list(data["robots"].keys()) == [f"cf{i}" for i in range(1, 11)]
assert data["robots"]["cf1"]["uri"].endswith("01")
assert data["robots"]["cf10"]["uri"].endswith("0A")
assert data["robots"]["cf1"]["initial_position"] == [0.0, 1.05, 0.0]
assert data["robots"]["cf10"]["initial_position"] == [0.455, -0.262, 0.0]
assert data["all"]["firmware_params"]["commander"]["enHighLevel"] == 1
assert data["all"]["firmware_params"]["stabilizer"]["controller"] == 2

with tempfile.TemporaryDirectory() as tmp_dir:
    output = Path(tmp_dir) / "generated_crazyflies.yaml"
    written = generate_crazyswarm2_yaml("config", output)
    loaded = yaml.safe_load(written.read_text(encoding="utf-8"))
    assert loaded == data

print("[OK] Crazyswarm2 sim YAML generation verified")
