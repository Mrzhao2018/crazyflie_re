"""MissionFSM contract tests"""

from src.runtime.mission_fsm import MissionFSM, MissionState


fsm = MissionFSM()

assert fsm.allowed_command_policy().leader_mode == "none"
assert fsm.allowed_command_policy().follower_mode == "none"

fsm.transition(MissionState.CONNECT)
fsm.transition(MissionState.POSE_READY)
fsm.transition(MissionState.PREFLIGHT)
fsm.transition(MissionState.TAKEOFF)
fsm.transition(MissionState.SETTLE)

policy = fsm.allowed_command_policy()
assert policy.leader_mode == "none"
assert policy.follower_mode == "hold"

fsm.transition(MissionState.RUN)
policy = fsm.allowed_command_policy()
assert policy.leader_mode == "batch_goto"
assert policy.follower_mode == "velocity"

fsm.transition(MissionState.HOLD)
policy = fsm.allowed_command_policy()
assert policy.leader_mode == "none"
assert policy.follower_mode == "hold"

fsm.transition(MissionState.ABORT)
policy = fsm.allowed_command_policy()
assert policy.leader_mode == "none"
assert policy.follower_mode == "hold"

fsm2 = MissionFSM()
try:
    fsm2.transition(MissionState.RUN)
    raise AssertionError("Expected illegal transition to raise")
except ValueError:
    pass

print("[OK] MissionFSM policy contracts verified")
