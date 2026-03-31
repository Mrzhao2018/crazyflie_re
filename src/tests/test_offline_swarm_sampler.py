"""Offline swarm sampler contracts."""

from src.app.bootstrap import build_core_app
from src.runtime.offline_swarm_sampler import sample_offline_swarm


components = build_core_app("config")
replay = sample_offline_swarm(components, dt=5.0)

assert replay.drone_ids == [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
assert len(replay.times) == len(replay.phase_labels)
assert replay.phase_labels[0] == "settle"
assert replay.phase_labels[-1] == "formation_run"
assert all(replay.frame_valid)
assert all(replay.follower_valid)
assert len(replay.leader_positions[1]) == len(replay.times)
assert len(replay.leader_positions[4]) == len(replay.times)
assert len(replay.leader_positions[7]) == len(replay.times)
assert len(replay.leader_positions[8]) == len(replay.times)
assert len(replay.follower_positions[5]) == len(replay.times)
assert len(replay.follower_positions[10]) == len(replay.times)
assert len(replay.positions[10]) == len(replay.times)
assert replay.roles[1] == "leader"
assert replay.roles[4] == "leader"
assert replay.roles[7] == "leader"
assert replay.roles[8] == "leader"
assert replay.roles[2] == "follower"
assert replay.roles[10] == "follower"
assert replay.leader_positions[1][0] == [1.05, 0.0, 0.35]
assert replay.positions[1][0] == [1.05, 0.0, 0.35]
assert replay.follower_positions[5][0] is not None
assert replay.positions[5][0] == replay.follower_positions[5][0]
assert all(value > 0 for value in replay.frame_condition_numbers)

print("[OK] Offline swarm sampler verified")
