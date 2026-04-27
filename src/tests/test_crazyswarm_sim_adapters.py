"""Crazyswarm2 sim adapters with fake swarm objects."""

import time

import numpy as np

import src.adapters.crazyswarm_sim as sim


class FakeFleet:
    def __init__(self):
        self._ids = [1, 2]

    def all_ids(self):
        return tuple(self._ids)

    def get_radio_group(self, drone_id):
        return {1: 0, 2: 1}[int(drone_id)]


class FakeCF:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.calls = []
        self.position = [float(drone_id), 0.0, 0.0]
        self.status = {"battery": 4.1, "rssi": -42}

    def takeoff(self, targetHeight, duration):
        self.calls.append(("takeoff", targetHeight, duration))

    def land(self, targetHeight, duration):
        self.calls.append(("land", targetHeight, duration))

    def goTo(self, goal, yaw, duration):
        self.calls.append(("goTo", tuple(goal), yaw, duration))

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.calls.append(("cmdFullState", tuple(pos), tuple(vel), tuple(acc), yaw, tuple(omega)))

    def notifySetpointsStop(self):
        self.calls.append(("notifySetpointsStop",))

    def setParam(self, name, value):
        self.calls.append(("setParam", name, value))

    def uploadTrajectory(self, trajectory_id, piece_offset, trajectory):
        self.calls.append(("uploadTrajectory", trajectory_id, piece_offset, trajectory))

    def startTrajectory(self, trajectory_id, timescale=1.0, reverse=False, relative=True):
        self.calls.append(("startTrajectory", trajectory_id, timescale, reverse, relative))

    def get_position(self):
        return self.position

    def get_status(self):
        return self.status


class FakeAllCfs:
    def __init__(self):
        self.crazyfliesById = {1: FakeCF(1), 2: FakeCF(2)}
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


class FakeSwarm:
    def __init__(self):
        self.allcfs = FakeAllCfs()


fleet = FakeFleet()
swarm = FakeSwarm()
link_manager = sim.CrazyswarmSimLinkManager(fleet, swarm_factory=lambda: swarm)
started = []
finished = []
report = link_manager.connect_all(
    on_group_start=lambda event: started.append(event),
    on_group_result=lambda event: finished.append(event),
)
assert report["ok"] is True
assert report["connected"] == [1, 2]
assert [event["group_id"] for event in started] == [0, 1]
assert [event["group_id"] for event in finished] == [0, 1]

transport = sim.CrazyswarmSimCommandTransport(link_manager)
transport.hl_takeoff(1, 0.5, 2.0)
transport.hl_go_to(1, 1.0, 2.0, 0.5, 1.5)
transport.set_onboard_controller(1, "mellinger")
transport.cmd_full_state(1, (0.1, 0.2, 0.3), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
transport.cmd_velocity_world(1, 1.0, 0.0, 0.0)
transport.notify_setpoint_stop(1)
assert ("takeoff", 0.5, 2.0) in link_manager.get(1).calls
assert ("goTo", (1.0, 2.0, 0.5), 0.0, 1.5) in link_manager.get(1).calls
assert ("setParam", "stabilizer.controller", 2) in link_manager.get(1).calls
assert link_manager.get(1).calls[-2][0] == "cmdFullState"
assert link_manager.get(1).calls[-1] == ("notifySetpointsStop",)
assert transport.last_velocity_command_time(1) is not None

failure = transport.classify_command_failure(
    drone_id=1,
    command_kind="full_state",
    exception=ValueError("bad setpoint"),
)
assert failure["failure_category"] == "invalid_command"
assert failure["retryable"] is False


class FakeTrajectoryModule:
    class Polynomial4D:
        def __init__(self, duration, px, py, pz, pyaw):
            self.duration = duration
            self.px = type("P", (), {"p": px})()
            self.py = type("P", (), {"p": py})()
            self.pz = type("P", (), {"p": pz})()
            self.pyaw = type("P", (), {"p": pyaw})()

    class Trajectory:
        pass


piece = type(
    "Piece",
    (),
    {
        "duration": 1.25,
        "x": [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "y": [0.0] * 8,
        "z": [0.5] + [0.0] * 7,
        "yaw": [0.0] * 8,
    },
)()
trajectory = sim.pieces_to_crazyswarm_trajectory([piece], FakeTrajectoryModule)
assert trajectory.duration == 1.25
assert len(trajectory.polynomials) == 1
assert trajectory.polynomials[0].px.p.tolist() == piece.x

original_converter = sim.pieces_to_crazyswarm_trajectory
try:
    sim.pieces_to_crazyswarm_trajectory = lambda pieces: {"pieces": list(pieces)}
    results = transport.upload_trajectories_by_group(
        {1: {"pieces": [piece], "start_addr": 0, "trajectory_id": 7, "trajectory_type": "poly4d_compressed"}}
    )
finally:
    sim.pieces_to_crazyswarm_trajectory = original_converter
assert results[1]["piece_count"] == 1
assert results[1]["trajectory_id"] == 7
assert link_manager.get(1).calls[-1][0] == "uploadTrajectory"
assert link_manager.get(1).calls[-1][1] == 7

pose_events = []
health_events = []
pose_source = sim.CrazyswarmSimPoseSource(link_manager, fleet, log_freq_hz=100.0)
pose_source.register_callback(lambda drone_id, pos, ts, velocity=None: pose_events.append((drone_id, pos, velocity)))
pose_source.register_health_callback(lambda drone_id, values, ts: health_events.append((drone_id, values)))
pose_source.start()
time.sleep(0.05)
pose_source.stop()
assert {event[0] for event in pose_events} == {1, 2}
assert {event[0] for event in health_events} == {1, 2}
assert all("pm.vbat" in event[1] for event in health_events)

print("[OK] Crazyswarm2 sim adapters verified with fakes")
