"""MissionProfile tests"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.mission_profile import MissionProfile
from src.config.schema import MissionConfig, MissionPhaseConfig


config = ConfigLoader.load("config")
profile = MissionProfile(config.mission)

assert profile.total_time() == config.mission.duration
phase0 = profile.phase_at(0.5)
assert phase0.mode == "settle"

phase1 = profile.phase_at(5.0)
assert phase1.mode == "formation_run"

transform = profile.affine_transform_at(5.0)
assert transform.A.shape == (3, 3)
assert transform.b.shape == (3,)
assert not np.isnan(transform.A).any()

config.mission.leader_motion.trajectory_enabled = True
config.mission.leader_motion.trajectory_pieces = []
config.mission.leader_motion.trajectory_sample_dt = 10.0
trajectory_profile = MissionProfile(config.mission)
trajectory_spec = trajectory_profile.trajectory_spec()
expected_piece_count = 0
for phase in config.mission.phases:
    phase_duration = phase.t_end - phase.t_start
    expected_piece_count += max(
        1,
        int(
            np.ceil(phase_duration / config.mission.leader_motion.trajectory_sample_dt)
        ),
    )
assert len(trajectory_spec["pieces"]) == expected_piece_count
for generated_piece in trajectory_spec["pieces"]:
    assert generated_piece.duration > 0
    assert len(generated_piece.x) == 8
    assert len(generated_piece.y) == 8
    assert len(generated_piece.z) == 8
    assert len(generated_piece.yaw) == 8
    assert any(abs(coeff) > 0 for coeff in generated_piece.x[:4])
    assert any(abs(coeff) > 0 for coeff in generated_piece.y[:4])

leader_a = trajectory_profile.trajectory_spec_for_nominal(np.array([1.0, 0.0, 0.5]))
leader_b = trajectory_profile.trajectory_spec_for_nominal(np.array([-1.0, 0.0, 0.5]))
assert leader_a["nominal_position"] != leader_b["nominal_position"]
assert any(
    piece_a.x != piece_b.x or piece_a.y != piece_b.y or piece_a.z != piece_b.z
    for piece_a, piece_b in zip(leader_a["pieces"], leader_b["pieces"])
)
assert any(
    any(abs(coeff) > 0 for coeff in piece.x[2:4]) for piece in leader_a["pieces"]
)

print("[OK] MissionProfile contracts verified")


broken_config = ConfigLoader.load("config")
broken_config.mission = MissionConfig(
    duration=10.0,
    formation_type=broken_config.mission.formation_type,
    nominal_positions=broken_config.mission.nominal_positions,
    leader_motion=broken_config.mission.leader_motion,
    phases=[
        MissionPhaseConfig(name="settle", t_start=1.0, t_end=3.0, mode="settle"),
        MissionPhaseConfig(
            name="formation_run", t_start=3.0, t_end=12.0, mode="formation_run"
        ),
    ],
)

try:
    ConfigLoader._validate(broken_config)
    raise AssertionError("Expected invalid mission timing to be rejected")
except ValueError as exc:
    assert "duration" in str(exc) or "t_start=0.0" in str(exc)


gap_config = ConfigLoader.load("config")
gap_config.mission = MissionConfig(
    duration=12.0,
    formation_type=gap_config.mission.formation_type,
    nominal_positions=gap_config.mission.nominal_positions,
    leader_motion=gap_config.mission.leader_motion,
    phases=[
        MissionPhaseConfig(name="settle", t_start=0.0, t_end=3.0, mode="settle"),
        MissionPhaseConfig(
            name="formation_run", t_start=4.0, t_end=12.0, mode="formation_run"
        ),
    ],
)

try:
    ConfigLoader._validate(gap_config)
    raise AssertionError("Expected phase gap to be rejected")
except ValueError as exc:
    assert "连续" in str(exc)
