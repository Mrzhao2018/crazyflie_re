"""ConfigLoader cross-config validation tests."""

from src.config.loader import ConfigLoader


config = ConfigLoader.load("config")
ConfigLoader._validate(config)
assert config.comm.connect_groups_in_parallel is True
assert config.comm.trajectory_upload_groups_in_parallel is True
assert config.control.time_delay_compensation_enabled is False
assert config.control.estimated_total_delay_ms == 0.0
assert config.control.dynamics_model_order == 2
assert config.control.output_mode == "full_state"
assert config.control.onboard_controller == "mellinger"
assert config.control.velocity_feedback_gain == 1.2
assert config.control.acceleration_feedforward_gain == 0.5
assert config.control.mass_kg == 0.033
assert config.control.damping_coeff == 0.05
assert config.control.max_acceleration == 2.0
assert config.control.full_state_position_smoothing_alpha == 0.45
assert config.control.full_state_max_position_step == 0.04
assert config.control.full_state_warmup_s == 1.0
assert config.control.full_state_warmup_rate_hz == 20.0
assert config.control.active_follower_ids == [5]
assert config.comm.telemetry_queue_max == 4096
assert config.comm.telemetry_flush_every_n == 50
assert config.comm.radio_health_window_s == 2.0
assert config.comm.congestion_soft_floor == 60.0
assert config.comm.latency_p95_soft_limit_ms == 50.0
assert config.comm.min_stream_keepalive_hz == 2.0
assert config.safety.executor_group_failure_streak == 2
assert config.safety.min_vbat == 0.0
assert config.safety.min_vbat_abort_samples == 5
assert config.safety.min_vbat_window_s == 3.0
assert config.safety.min_vbat_critical == 0.0
assert config.safety.fast_gate_group_degrade_enabled is True
assert config.safety.pose_jitter_threshold == 0.05
assert config.safety.estimator_variance_window_s == 1.0
assert config.safety.lighthouse_required_method is None
assert config.safety.runtime_pose_jump_threshold == 0.35
assert config.safety.runtime_pose_speed_threshold == 3.0
assert config.safety.runtime_vertical_speed_threshold == 1.5
assert config.mission.leader_motion.trajectory_type == "poly4d_compressed"
assert config.mission.leader_motion.trajectory_sample_dt == 0.1334

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

bad_full_state_alpha = ConfigLoader.load("config")
bad_full_state_alpha.control.full_state_position_smoothing_alpha = 0.0
try:
    ConfigLoader._validate(bad_full_state_alpha)
    raise AssertionError("Expected full_state_position_smoothing_alpha validation to fail")
except ValueError as exc:
    assert "full_state_position_smoothing_alpha" in str(exc)

bad_full_state_step = ConfigLoader.load("config")
bad_full_state_step.control.full_state_max_position_step = 0.0
try:
    ConfigLoader._validate(bad_full_state_step)
    raise AssertionError("Expected full_state_max_position_step validation to fail")
except ValueError as exc:
    assert "full_state_max_position_step" in str(exc)

bad_full_state_warmup = ConfigLoader.load("config")
bad_full_state_warmup.control.full_state_warmup_rate_hz = 0.0
try:
    ConfigLoader._validate(bad_full_state_warmup)
    raise AssertionError("Expected full_state_warmup_rate_hz validation to fail")
except ValueError as exc:
    assert "full_state_warmup_rate_hz" in str(exc)

bad_pose_jump = ConfigLoader.load("config")
bad_pose_jump.safety.runtime_pose_jump_threshold = -1.0
try:
    ConfigLoader._validate(bad_pose_jump)
    raise AssertionError("Expected runtime_pose_jump_threshold validation to fail")
except ValueError as exc:
    assert "runtime_pose_jump_threshold" in str(exc)

bad_active_followers = ConfigLoader.load("config")
bad_active_followers.control.active_follower_ids = [1]
try:
    ConfigLoader._validate(bad_active_followers)
    raise AssertionError("Expected active_follower_ids validation to fail")
except ValueError as exc:
    assert "active_follower_ids" in str(exc)

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
bad_sample_dt.mission.leader_motion.trajectory_type = "poly4d"
bad_sample_dt.mission.leader_motion.trajectory_sample_dt = 0.5
try:
    ConfigLoader._validate(bad_sample_dt)
    raise AssertionError("Expected trajectory sample dt consistency check to fail")
except ValueError as exc:
    assert "trajectory_sample_dt" in str(exc)

compressed_ok = ConfigLoader.load("config")
compressed_ok.mission.leader_motion.trajectory_type = "poly4d_compressed"
compressed_ok.mission.leader_motion.trajectory_sample_dt = 0.1334
ConfigLoader._validate(compressed_ok)

compressed_reversed = ConfigLoader.load("config")
compressed_reversed.mission.leader_motion.trajectory_type = "poly4d_compressed"
compressed_reversed.mission.leader_motion.trajectory_reversed = True
try:
    ConfigLoader._validate(compressed_reversed)
    raise AssertionError("Expected compressed reversed trajectory validation to fail")
except ValueError as exc:
    assert "poly4d_compressed" in str(exc)

bad_watchdog_action = ConfigLoader.load("config")
bad_watchdog_action.safety.velocity_stream_watchdog_action = "invalid"
try:
    ConfigLoader._validate(bad_watchdog_action)
    raise AssertionError("Expected watchdog action validation to fail")
except ValueError as exc:
    assert "velocity_stream_watchdog_action" in str(exc)

bad_executor_streak = ConfigLoader.load("config")
bad_executor_streak.safety.executor_group_failure_streak = 0
try:
    ConfigLoader._validate(bad_executor_streak)
    raise AssertionError("Expected executor_group_failure_streak validation to fail")
except ValueError as exc:
    assert "executor_group_failure_streak" in str(exc)

bad_pose_jitter = ConfigLoader.load("config")
bad_pose_jitter.safety.pose_jitter_threshold = -0.01
try:
    ConfigLoader._validate(bad_pose_jitter)
    raise AssertionError("Expected pose_jitter_threshold validation to fail")
except ValueError as exc:
    assert "pose_jitter_threshold" in str(exc)

bad_variance_window = ConfigLoader.load("config")
bad_variance_window.safety.estimator_variance_window_s = 0.0
try:
    ConfigLoader._validate(bad_variance_window)
    raise AssertionError("Expected estimator variance window validation to fail")
except ValueError as exc:
    assert "estimator_variance_window_s" in str(exc)

bad_vbat_samples = ConfigLoader.load("config")
bad_vbat_samples.safety.min_vbat_abort_samples = 0
try:
    ConfigLoader._validate(bad_vbat_samples)
    raise AssertionError("Expected min_vbat_abort_samples validation to fail")
except ValueError as exc:
    assert "min_vbat_abort_samples" in str(exc)

bad_vbat_critical = ConfigLoader.load("config")
bad_vbat_critical.safety.min_vbat_critical = -0.1
try:
    ConfigLoader._validate(bad_vbat_critical)
    raise AssertionError("Expected min_vbat_critical validation to fail")
except ValueError as exc:
    assert "min_vbat_critical" in str(exc)

bad_vbat_window = ConfigLoader.load("config")
bad_vbat_window.safety.min_vbat_window_s = 0.0
try:
    ConfigLoader._validate(bad_vbat_window)
    raise AssertionError("Expected min_vbat_window_s validation to fail")
except ValueError as exc:
    assert "min_vbat_window_s" in str(exc)

print("[OK] ConfigLoader cross-config checks verified")