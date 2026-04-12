"""cflib adapter behavior tests with fakes"""

import numpy as np

from src.adapters.cflib_command_transport import CflibCommandTransport
from src.adapters.follower_executor import FollowerExecutor
from src.adapters.leader_executor import LeaderExecutor
from src.runtime.command_plan import FollowerAction, HoldAction, LeaderAction


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
        self.mem = type(
            "FakeMem",
            (),
            {
                "get_mems": lambda self, mem_type: [
                    type(
                        "FakeTrajectoryMem",
                        (),
                        {
                            "trajectory": [],
                            "write_data_sync": lambda self, start_addr=0: True,
                        },
                    )()
                ]
            },
        )()


class FakeSCF:
    def __init__(self):
        self.cf = FakeCF()
        self.waited = False
        self.uploaded = None

    def wait_for_params(self):
        self.waited = True


class FakeLinkManager:
    def __init__(self):
        self.fleet = type(
            "FakeFleet",
            (),
            {"get_radio_group": lambda self, drone_id: {1: 0, 2: 1}.get(drone_id, 2)},
        )()
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
assert transport.last_high_level_command_time(1) is not None

transport.cmd_velocity_world(1, 0.1, 0.2, 0.3)
assert link_manager.scfs[1].cf.commander.calls[-1] == ("velocity", 0.1, 0.2, 0.3, 0)
assert transport.last_velocity_command_time(1) is not None

executor = LeaderExecutor(transport)
results = executor.execute(
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
assert results[0]["successes"] == [1]
assert results[0]["failures"] == []

failure = transport.classify_command_failure(
    drone_id=1,
    command_kind="start_trajectory",
    exception=TimeoutError("radio timeout"),
)
assert failure["drone_id"] == 1
assert failure["radio_group"] == 0
assert failure["command_kind"] == "start_trajectory"
assert failure["error_type"] == "TimeoutError"
assert failure["failure_category"] == "timeout"
assert failure["retryable"] is True

non_retryable = transport.classify_command_failure(
    drone_id=1,
    command_kind="upload_trajectory",
    exception=RuntimeError("Trajectory too large for drone 1"),
)
assert non_retryable["failure_category"] == "transport_runtime"
assert non_retryable["retryable"] is False

uploads = transport.upload_trajectories_by_group(
    {
        1: {"pieces": [], "start_addr": 0, "trajectory_id": 1},
    },
    parallel_groups=True,
)
assert uploads[1]["piece_count"] == 0
assert link_manager.scfs[1].cf.high_level_commander.calls[-1][0] == "define_trajectory"


class BrokenTransport:
    def __init__(self):
        self.fleet = type(
            "FakeFleet",
            (),
            {"get_radio_group": lambda self, drone_id: {1: 0, 2: 1}.get(drone_id, 2)},
        )()

    def classify_command_failure(self, *, drone_id, command_kind, exception):
        return {
            "drone_id": drone_id,
            "radio_group": self.fleet.get_radio_group(drone_id),
            "command_kind": command_kind,
            "error": str(exception),
            "error_type": type(exception).__name__,
            "failure_category": "timeout"
            if isinstance(exception, TimeoutError)
            else "link_lookup",
            "retryable": isinstance(exception, TimeoutError),
        }

    def hl_takeoff(self, drone_id, height, duration):
        raise TimeoutError(f"takeoff timeout {drone_id}")

    def hl_go_to(self, drone_id, x, y, z, duration):
        raise KeyError(f"missing {drone_id}")

    def hl_land(self, drone_id, height, duration):
        raise TimeoutError(f"land timeout {drone_id}")

    def hl_start_trajectory(
        self,
        drone_id,
        trajectory_id,
        time_scale=1.0,
        relative_position=False,
        relative_yaw=False,
        reversed=False,
    ):
        raise TimeoutError(f"traj timeout {drone_id}")

    def cmd_velocity_world(self, drone_id, vx, vy, vz):
        raise TimeoutError(f"velocity timeout {drone_id}")

    def notify_setpoint_stop(self, drone_id):
        raise KeyError(f"notify missing {drone_id}")


broken_transport = BrokenTransport()
leader_executor = LeaderExecutor(broken_transport)
leader_failure = leader_executor.execute(
    [
        LeaderAction(
            kind="batch_goto",
            drone_ids=[2],
            payload={"positions": {2: np.array([0.0, 0.0, 0.5])}, "duration": 1.0},
        )
    ]
)[0]["failures"][0]
assert leader_failure["drone_id"] == 2
assert leader_failure["radio_group"] == 1
assert leader_failure["command_kind"] == "batch_goto"
assert leader_failure["failure_category"] == "link_lookup"
assert leader_failure["retryable"] is False

follower_executor = FollowerExecutor(broken_transport)
velocity_failure = follower_executor.execute_velocity(
    [FollowerAction(kind="velocity", drone_id=2, velocity=np.array([0.1, 0.0, 0.0]))]
)["failures"][0]
assert velocity_failure["command_kind"] == "velocity"
assert velocity_failure["radio_group"] == 1
assert velocity_failure["failure_category"] == "timeout"
assert velocity_failure["retryable"] is True

hold_failure = follower_executor.execute_hold([HoldAction(drone_id=2)])["failures"][0]
assert hold_failure["command_kind"] == "hold"

notify_failure = follower_executor.stop_velocity_mode([2])["failures"][0]
assert notify_failure["command_kind"] == "notify_stop"
assert notify_failure["failure_category"] == "link_lookup"
assert notify_failure["retryable"] is False


class FakePiece:
    def __init__(self):
        self.duration = 1.0
        self.x = [0.0] * 8
        self.y = [0.0] * 8
        self.z = [0.0] * 8
        self.yaw = [0.0] * 8


try:
    transport.upload_trajectory(1, [FakePiece() for _ in range(40)], start_addr=0)
    raise AssertionError("Expected oversized trajectory upload to fail")
except RuntimeError as exc:
    assert "Trajectory too large" in str(exc)

print("[OK] cflib transport/executor trajectory support verified")
