"""任务状态机"""

from enum import Enum
from dataclasses import dataclass
from typing import Literal


class MissionState(Enum):
    INIT = "init"
    CONNECT = "connect"
    POSE_READY = "pose_ready"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    SETTLE = "settle"
    LEADER_ARMED = "leader_armed"
    RUN = "run"
    HOLD = "hold"
    LAND = "land"
    ABORT = "abort"


@dataclass
class CommandPolicy:
    leader_mode: Literal["none", "trajectory", "batch_goto"]
    follower_mode: Literal["none", "velocity", "hold"]


ALLOWED_TRANSITIONS = {
    MissionState.INIT: {MissionState.CONNECT},
    MissionState.CONNECT: {MissionState.POSE_READY, MissionState.ABORT},
    MissionState.POSE_READY: {MissionState.PREFLIGHT, MissionState.ABORT},
    MissionState.PREFLIGHT: {MissionState.TAKEOFF, MissionState.ABORT},
    MissionState.TAKEOFF: {MissionState.SETTLE, MissionState.ABORT, MissionState.LAND},
    MissionState.SETTLE: {
        MissionState.RUN,
        MissionState.HOLD,
        MissionState.ABORT,
        MissionState.LAND,
    },
    MissionState.LEADER_ARMED: {
        MissionState.RUN,
        MissionState.ABORT,
        MissionState.LAND,
    },
    MissionState.RUN: {MissionState.HOLD, MissionState.LAND, MissionState.ABORT},
    MissionState.HOLD: {MissionState.RUN, MissionState.LAND, MissionState.ABORT},
    MissionState.LAND: {MissionState.INIT, MissionState.ABORT},
    MissionState.ABORT: set(),
}


class MissionFSM:
    def __init__(self):
        self._state = MissionState.INIT

    def state(self) -> MissionState:
        return self._state

    def transition(self, target: MissionState):
        if target == self._state:
            return self._state

        allowed = ALLOWED_TRANSITIONS.get(self._state, set())
        if target not in allowed:
            raise ValueError(
                f"Illegal mission state transition: {self._state.value} -> {target.value}"
            )

        self._state = target
        return self._state

    def allowed_command_policy(self) -> CommandPolicy:
        if self._state == MissionState.RUN:
            return CommandPolicy(leader_mode="trajectory", follower_mode="velocity")
        if self._state in {MissionState.HOLD, MissionState.SETTLE}:
            return CommandPolicy(leader_mode="none", follower_mode="hold")
        if self._state == MissionState.TAKEOFF:
            return CommandPolicy(leader_mode="none", follower_mode="none")
        if self._state == MissionState.LAND:
            return CommandPolicy(leader_mode="none", follower_mode="hold")
        if self._state == MissionState.ABORT:
            return CommandPolicy(leader_mode="none", follower_mode="hold")
        return CommandPolicy(leader_mode="none", follower_mode="none")
