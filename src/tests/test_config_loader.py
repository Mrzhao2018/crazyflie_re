"""ConfigLoader cross-config validation tests."""

from src.config.loader import ConfigLoader


config = ConfigLoader.load("config")
ConfigLoader._validate(config)

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

print("[OK] ConfigLoader cross-config checks verified")