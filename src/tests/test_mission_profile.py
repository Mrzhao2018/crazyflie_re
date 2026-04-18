"""MissionProfile tests"""

import numpy as np

from src.config.loader import ConfigLoader
from src.domain.mission_profile import MissionProfile
from src.config.schema import MissionConfig, MissionPhaseConfig


config = ConfigLoader.load("config")
config.mission.phases = [
    MissionPhaseConfig(name="settle", t_start=0.0, t_end=3.0, mode="settle"),
    MissionPhaseConfig(
        name="trajectory_entry", t_start=3.0, t_end=5.0, mode="trajectory_entry"
    ),
    MissionPhaseConfig(
        name="formation_run",
        t_start=5.0,
        t_end=config.mission.duration,
        mode="formation_run",
    ),
]
profile = MissionProfile(config.mission)

assert profile.total_time() == config.mission.duration
phase0 = profile.phase_at(0.5)
assert phase0.mode == "settle"

phase_entry = profile.phase_at(3.5)
assert phase_entry.mode == "trajectory_entry"

phase1 = profile.phase_at(5.0)
assert phase1.mode == "formation_run"

transform = profile.affine_transform_at(5.0)
assert transform.A.shape == (3, 3)
assert transform.b.shape == (3,)
assert not np.isnan(transform.A).any()
entry_transform = profile.affine_transform_at(3.5)
assert np.allclose(entry_transform.A, np.eye(3))

config.mission.leader_motion.trajectory_enabled = True
config.mission.leader_motion.trajectory_pieces = []
config.mission.leader_motion.trajectory_sample_dt = 10.0
trajectory_profile = MissionProfile(config.mission)
trajectory_spec = trajectory_profile.trajectory_spec()
expected_piece_count = 0
for phase in config.mission.phases:
    if phase.mode == "formation_run":
        phase_duration = phase.t_end - phase.t_start
        expected_piece_count += max(
            1,
            int(
                np.ceil(
                    phase_duration / config.mission.leader_motion.trajectory_sample_dt
                )
            ),
        )
assert len(trajectory_spec["pieces"]) == expected_piece_count
assert trajectory_profile.trajectory_start_time() == 5.0
for generated_piece in trajectory_spec["pieces"]:
    assert generated_piece.duration > 0
    assert len(generated_piece.x) == 8
    assert len(generated_piece.y) == 8
    assert len(generated_piece.z) == 8
    assert len(generated_piece.yaw) == 8
    assert any(abs(coeff) > 0 for coeff in generated_piece.x[:4])
    assert any(abs(coeff) > 0 for coeff in generated_piece.y[:4])
first_piece = trajectory_spec["pieces"][0]
assert abs(first_piece.x[1]) > 0 or abs(first_piece.y[1]) > 0

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
quality_summary = trajectory_profile.trajectory_quality_summary()
assert quality_summary["condition_number_max"] is not None
assert quality_summary["condition_number_min"] is not None
assert quality_summary["penalized_samples"] >= 0

penalty_config = ConfigLoader.load("config")
penalty_config.mission.phases = config.mission.phases
penalty_config.mission.leader_motion.trajectory_enabled = True
penalty_config.mission.leader_motion.condition_penalty_enabled = True
penalty_config.mission.leader_motion.condition_stress_enabled = True
penalty_config.mission.leader_motion.condition_stress_min_scale = 0.15
penalty_config.mission.leader_motion.condition_soft_limit = 3.05
penalty_config.mission.leader_motion.condition_penalty_scale = 1.0
penalty_profile = MissionProfile(penalty_config.mission)
penalty_quality_summary = penalty_profile.trajectory_quality_summary()
assert penalty_quality_summary["penalty_active"] is True
assert penalty_quality_summary["penalized_samples"] > 0
assert (
    penalty_quality_summary["condition_number_max"]
    < penalty_quality_summary["raw_condition_number_max"]
)

stress_config = ConfigLoader.load("config")
stress_config.mission.phases = config.mission.phases
stress_config.mission.leader_motion.trajectory_enabled = True
stress_config.mission.leader_motion.condition_stress_enabled = True
stress_config.mission.leader_motion.condition_stress_min_scale = 0.15
stress_profile = MissionProfile(stress_config.mission)
stress_quality_summary = stress_profile.trajectory_quality_summary()
assert stress_quality_summary["raw_condition_number_max"] is not None
assert stress_quality_summary["raw_condition_number_max"] > quality_summary["condition_number_max"]

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
