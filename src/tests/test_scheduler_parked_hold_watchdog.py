"""Parked follower hold cadence should stay watchdog-safe."""

import time

import numpy as np

from src.config.schema import CommConfig
from src.runtime.follower_controller import FollowerCommandSet
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision
from src.runtime.scheduler import CommandScheduler


class FakeFleet:
    def __init__(self):
        self._f = [5]
        self._l = [1]
        self._map = {1: 0, 5: 0}

    def follower_ids(self):
        return self._f.copy()

    def leader_ids(self):
        return self._l.copy()

    def get_radio_group(self, drone_id):
        return self._map[drone_id]


def test_parked_followers_keep_watchdog_safe_hold_cadence():
    fleet = FakeFleet()
    fsm = MissionFSM()
    for state in (
        MissionState.CONNECT,
        MissionState.POSE_READY,
        MissionState.PREFLIGHT,
        MissionState.TAKEOFF,
        MissionState.SETTLE,
        MissionState.RUN,
    ):
        fsm.transition(state)

    comm = CommConfig(
        pose_log_freq=10.0,
        follower_tx_freq=8.0,
        leader_update_freq=1.0,
        parked_hold_freq=0.5,
    )
    scheduler = CommandScheduler(comm, fsm, fleet)

    snapshot = PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=np.zeros((2, 3)),
        fresh_mask=np.ones(2, dtype=bool),
        disconnected_ids=[],
    )
    safety = SafetyDecision(action="EXECUTE", reasons=[])
    commands = FollowerCommandSet(commands={}, diagnostics={})

    # Simulate a parked follower that already received one hold 0.2s ago.
    now = time.monotonic()
    scheduler.last_hold_tx_time = {0: now - 0.2}
    plan = scheduler.plan(
        snapshot,
        MissionState.RUN,
        None,
        commands,
        safety,
        parked_follower_ids=[5],
    )

    assert [action.drone_id for action in plan.hold_actions] == [5], (
        "Parked followers should continue receiving hold packets fast enough "
        "to stay inside the velocity watchdog window."
    )
