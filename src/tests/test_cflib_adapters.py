"""cflib adapter behavior tests with fakes"""

import numpy as np

from src.adapters.cflib_command_transport import CflibCommandTransport
from src.adapters.leader_executor import LeaderExecutor
from src.runtime.command_plan import LeaderAction


class FakeHighLevelCommander:
    def __init__(self):
        self.calls = []

    def takeoff(self, height, duration):
        self.calls.append(("takeoff", height, duration))

    def land(self, height, duration):
        self.calls.append(("land", height, duration))

    def go_to(self, x, y, z, yaw, duration):
        self.calls.append(("go_to", x, y, z, yaw, duration))

    def define_trajectory(self, trajectory_id, offset, n_pieces):
        self.calls.append(("define_trajectory", trajectory_id, offset, n_pieces))

    def start_trajectory(
        self,
        trajectory_id,
        time_scale=1.0,
        relative_position=False,
        relative_yaw=False,
        reversed=False,
    ):
        self.calls.append(
            (
                "start_trajectory",
                trajectory_id,
                time_scale,
                relative_position,
                relative_yaw,
                reversed,
            )
        )


class FakeCommander:
    def __init__(self):
        self.calls = []

    def send_velocity_world_setpoint(self, vx, vy, vz, yaw_rate):
        self.calls.append(("velocity", vx, vy, vz, yaw_rate))

    def send_notify_setpoint_stop(self):
        self.calls.append(("notify_stop",))


class FakeCF:
    def __init__(self):
        self.high_level_commander = FakeHighLevelCommander()
        self.commander = FakeCommander()


class FakeSCF:
    def __init__(self):
        self.cf = FakeCF()
        self.waited = False
        self.uploaded = None

    def wait_for_params(self):
        self.waited = True


class FakeLinkManager:
    def __init__(self):
        self.scfs = {1: FakeSCF()}

    def get(self, drone_id):
        return self.scfs[drone_id]


link_manager = FakeLinkManager()
transport = CflibCommandTransport(link_manager)
transport.wait_for_params(1)
assert link_manager.scfs[1].waited is True

transport.hl_start_trajectory(
    1, 7, time_scale=1.2, relative_position=True, relative_yaw=False, reversed=True
)
calls = link_manager.scfs[1].cf.high_level_commander.calls
assert calls[-1][0] == "start_trajectory"
assert calls[-1][1] == 7

executor = LeaderExecutor(transport)
executor.execute(
    [
        LeaderAction(
            kind="start_trajectory",
            drone_ids=[1],
            payload={
                "trajectory_id": 3,
                "time_scale": 1.0,
                "relative_position": False,
                "relative_yaw": False,
                "reversed": False,
            },
        )
    ]
)
assert link_manager.scfs[1].cf.high_level_commander.calls[-1][0] == "start_trajectory"

print("[OK] cflib transport/executor trajectory support verified")
