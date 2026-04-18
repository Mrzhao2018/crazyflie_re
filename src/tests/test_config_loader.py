"""ConfigLoader cross-config validation tests."""

from src.config.loader import ConfigLoader


config = ConfigLoader.load("config")
ConfigLoader._validate(config)
assert config.comm.connect_groups_in_parallel is False
assert config.comm.trajectory_upload_groups_in_parallel is False
assert config.control.time_delay_compensation_enabled is False
assert config.control.estimated_total_delay_ms == 0.0
assert config.control.dynamics_model_order == 1
assert config.control.velocity_feedback_gain == 0.8
assert config.control.acceleration_feedforward_gain == 1.0
assert config.control.mass_kg == 0.033
assert config.control.damping_coeff == 0.15
assert config.control.max_acceleration == 2.0

bad_model_order = ConfigLoader.load("config")
bad_model_order.control.dynamics_model_order = 3
try:
    ConfigLoader._validate(bad_model_order)
    raise AssertionError("Expected dynamics_model_order validation to fail")
except ValueError as exc:
    assert "dynamics_model_order" in str(exc)

bad_mass = ConfigLoader.load("config")
bad_mass.control.mass_kg = 0.0
try:
    ConfigLoader._validate(bad_mass)
    raise AssertionError("Expected mass_kg validation to fail")
except ValueError as exc:
    assert "mass_kg" in str(exc)

bad_max_acc = ConfigLoader.load("config")
bad_max_acc.control.max_acceleration = 0.0
try:
    ConfigLoader._validate(bad_max_acc)
    raise AssertionError("Expected max_acceleration validation to fail")
except ValueError as exc:
    assert "max_acceleration" in str(exc)

bad_delay = ConfigLoader.load("config")
bad_delay.control.estimated_total_delay_ms = -1.0
try:
    ConfigLoader._validate(bad_delay)
    raise AssertionError("Expected delay compensation validation to fail")
except ValueError as exc:
    assert "estimated_total_delay_ms" in str(exc)

bad_pose_timeout = ConfigLoader.load("config")
bad_pose_timeout.safety.pose_timeout = 1.0 / bad_pose_timeout.comm.pose_log_freq
try:
    ConfigLoader._validate(bad_pose_timeout)
    raise AssertionError("Expected pose timeout consistency check to fail")
except ValueError as exc:
    assert "pose_timeout" in str(exc)

bad_leader_rate = ConfigLoader.load("config")
bad_leader_rate.comm.leader_update_freq = bad_leader_rate.comm.follower_tx_freq + 1.0
try:
    ConfigLoader._validate(bad_leader_rate)
    raise AssertionError("Expected leader/follower rate consistency check to fail")
except ValueError as exc:
    assert "leader_update_freq" in str(exc)

bad_sample_dt = ConfigLoader.load("config")
bad_sample_dt.mission.leader_motion.trajectory_enabled = True
bad_sample_dt.mission.leader_motion.trajectory_sample_dt = 0.5
try:
    ConfigLoader._validate(bad_sample_dt)
    raise AssertionError("Expected trajectory sample dt consistency check to fail")
except ValueError as exc:
    assert "trajectory_sample_dt" in str(exc)

bad_watchdog_action = ConfigLoader.load("config")
bad_watchdog_action.safety.velocity_stream_watchdog_action = "invalid"
try:
    ConfigLoader._validate(bad_watchdog_action)
    raise AssertionError("Expected watchdog action validation to fail")
except ValueError as exc:
    assert "velocity_stream_watchdog_action" in str(exc)

print("[OK] ConfigLoader cross-config checks verified")