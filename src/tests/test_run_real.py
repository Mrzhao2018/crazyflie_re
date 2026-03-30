"""RealMissionApp orchestration contract tests with fakes"""

import numpy as np

from src.app.run_real import RealMissionApp
from src.runtime.mission_fsm import MissionFSM, MissionState
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision
from src.runtime.follower_controller import FollowerCommandSet


class FakeFleet:
    def __init__(self):
        self._leaders = [1, 2, 3, 4]
        self._followers = [5, 6]
        self._all = self._leaders + self._followers
        self._id_to_index = {drone_id: i for i, drone_id in enumerate(self._all)}

    def leader_ids(self):
        return self._leaders.copy()

    def follower_ids(self):
        return self._followers.copy()

    def all_ids(self):
        return self._all.copy()

    def id_to_index(self, drone_id):
        return self._id_to_index[drone_id]


class FakeLinkManager:
    def __init__(self, should_fail=False):
        self.should_fail = should_fail
        self.closed = False

    def connect_all(self):
        if self.should_fail:
            raise RuntimeError("connect failed")

    def close_all(self):
        self.closed = True


class FakeTransport:
    def __init__(self):
        self.wait_calls = []
        self.reset_calls = []
        self.upload_calls = []
        self.define_calls = []

    def wait_for_params(self, drone_id):
        self.wait_calls.append(drone_id)

    def reset_estimator_and_wait(self, drone_id):
        self.reset_calls.append(drone_id)

    def upload_trajectory(self, drone_id, pieces, start_addr=0):
        self.upload_calls.append((drone_id, pieces, start_addr))
        return len(pieces)

    def hl_define_trajectory(self, drone_id, trajectory_id, offset, n_pieces):
        self.define_calls.append((drone_id, trajectory_id, offset, n_pieces))


class FakePoseSource:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.callback = None

    def register_callback(self, cb):
        self.callback = cb

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class FakePoseBus:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)
        self.index = 0
        self.update_calls = []

    def latest(self):
        if not self.snapshots:
            return None
        idx = min(self.index, len(self.snapshots) - 1)
        snapshot = self.snapshots[idx]
        if self.index < len(self.snapshots) - 1:
            self.index += 1
        return snapshot

    def update_agent(self, drone_id, pos, timestamp):
        self.update_calls.append((drone_id, pos, timestamp))


class FakePreflight:
    def __init__(self, ok=True, reasons=None):
        self.ok = ok
        self.reasons = reasons or []

    def run(self):
        return type(
            "PreflightReport",
            (),
            {
                "ok": self.ok,
                "reasons": self.reasons,
                "failed_codes": ["BAD"] if not self.ok else [],
                "checks": [],
            },
        )()


class FakeTelemetry:
    def __init__(self):
        self.opened = None
        self.records = []
        self.closed = False
        self.events = []

    def open(self, path):
        self.opened = path

    def log(self, record):
        self.records.append(record)

    def record_event(self, event, **details):
        self.events.append({"event": event, "details": details})

    def phase_events(self):
        return list(self.events)

    def summary(self):
        return {"event_counts": {event["event"]: 1 for event in self.events}}

    def export_replay(self):
        return {
            "phase_events": self.phase_events(),
            "records": self.records,
            "summary": self.summary(),
        }

    def close(self):
        self.closed = True


class FakeLeaderExecutor:
    def __init__(self):
        self.actions = []

    def execute(self, actions):
        self.actions.append(actions)


class FakeFollowerExecutor:
    def __init__(self):
        self.takeoff_calls = []
        self.hold_calls = []
        self.velocity_calls = []
        self.land_calls = []
        self.stop_calls = []

    def takeoff(self, drone_ids, height=0.5, duration=2.0):
        self.takeoff_calls.append((list(drone_ids), height, duration))

    def execute_hold(self, actions):
        self.hold_calls.append(actions)

    def execute_velocity(self, actions):
        self.velocity_calls.append(actions)

    def land(self, drone_ids, duration=2.0):
        self.land_calls.append((list(drone_ids), duration))

    def stop_velocity_mode(self, drone_ids):
        self.stop_calls.append(list(drone_ids))


class FakeFrameEstimator:
    def __init__(self):
        self.calls = 0

    def estimate(self, snapshot, leader_ids):
        self.calls += 1
        return type(
            "Frame",
            (),
            {
                "valid": True,
                "condition_number": 1.0,
                "leader_positions": {
                    lid: snapshot.positions[i] for i, lid in enumerate(leader_ids)
                },
            },
        )()


class FakeFollowerRefGen:
    def compute(self, leader_positions):
        return type(
            "FollowerRef",
            (),
            {
                "valid": True,
                "frame_condition_number": 1.0,
                "target_positions": {
                    5: np.array([0.0, 0.0, 1.0]),
                    6: np.array([0.0, 0.0, 1.0]),
                },
            },
        )()


class FakeFollowerController:
    def __init__(self):
        self.calls = 0

    def compute(self, snapshot, follower_ref, follower_ids, fleet):
        self.calls += 1
        return FollowerCommandSet(
            commands={fid: np.array([0.1, 0.0, 0.0]) for fid in follower_ids},
            diagnostics={},
        )


class FakeSafety:
    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.calls = 0

    def evaluate(
        self, snapshot, frame=None, commands=None, follower_ref=None, health=None
    ):
        idx = min(self.calls, len(self.decisions) - 1)
        self.calls += 1
        return self.decisions[idx]


class FakeScheduler:
    def __init__(self):
        self.calls = []

    def plan(
        self,
        snapshot,
        mission_state,
        leader_ref,
        commands,
        safety_decision,
        parked_follower_ids=None,
    ):
        self.calls.append(
            (snapshot.seq, mission_state, commands, safety_decision.action)
        )
        return type(
            "Plan",
            (),
            {
                "leader_actions": [],
                "follower_actions": [],
                "hold_actions": [],
                "diagnostics": {"reason": "fake_plan", "source": "test"},
            },
        )()


class FakeLeaderRefGen:
    def reference_at(self, t):
        return type(
            "LeaderRef",
            (),
            {
                "leader_ids": [1, 2, 3, 4],
                "mode": "trajectory",
                "trajectory": {
                    "per_leader": {
                        1: {
                            "trajectory_id": 1,
                            "start_addr": 0,
                            "pieces": [],
                            "nominal_position": [1.0, 0.0, 0.5],
                        },
                        2: {
                            "trajectory_id": 1,
                            "start_addr": 0,
                            "pieces": [],
                            "nominal_position": [0.0, 1.0, 0.5],
                        },
                        3: {
                            "trajectory_id": 1,
                            "start_addr": 0,
                            "pieces": [],
                            "nominal_position": [-1.0, 0.0, 0.5],
                        },
                        4: {
                            "trajectory_id": 1,
                            "start_addr": 0,
                            "pieces": [],
                            "nominal_position": [0.0, 0.0, 1.5],
                        },
                    }
                },
                "positions": {
                    1: np.zeros(3),
                    2: np.zeros(3),
                    3: np.zeros(3),
                    4: np.zeros(3),
                },
            },
        )()


def make_snapshot(seq, fresh=True, z=0.8):
    positions = np.array(
        [
            [1.0, 0.0, z],
            [0.0, 1.0, z],
            [-1.0, 0.0, z],
            [0.0, 0.0, z + 1.0],
            [0.0, 0.0, z],
            [0.5, 0.5, z],
        ],
        dtype=float,
    )
    return PoseSnapshot(
        seq=seq,
        t_meas=0.0,
        positions=positions,
        fresh_mask=np.ones(6, dtype=bool) if fresh else np.zeros(6, dtype=bool),
        disconnected_ids=[] if fresh else [1, 2, 3, 4, 5, 6],
    )


def build_components(
    start_snapshots, safety_decisions, connect_fail=False, preflight_ok=True
):
    fake_config = type(
        "FakeConfig",
        (),
        {
            "comm": type(
                "FakeComm",
                (),
                {
                    "readiness_wait_for_params": True,
                    "readiness_reset_estimator": True,
                },
            )(),
        },
    )()

    fake_health_bus = type(
        "FakeHealthBus",
        (),
        {
            "latest": lambda self: {
                did: type("Sample", (), {"values": {"pm.vbat": 4.0}})()
                for did in [1, 2, 3, 4, 5, 6]
            },
            "update": lambda self, drone_id, values, t_meas: None,
        },
    )()

    return {
        "config": fake_config,
        "fsm": MissionFSM(),
        "fleet": FakeFleet(),
        "link_manager": FakeLinkManager(should_fail=connect_fail),
        "transport": FakeTransport(),
        "pose_source": FakePoseSource(),
        "pose_bus": FakePoseBus(start_snapshots),
        "preflight": FakePreflight(
            ok=preflight_ok, reasons=[] if preflight_ok else ["bad"]
        ),
        "telemetry": FakeTelemetry(),
        "health_bus": fake_health_bus,
        "leader_executor": FakeLeaderExecutor(),
        "follower_executor": FakeFollowerExecutor(),
        "frame_estimator": FakeFrameEstimator(),
        "follower_ref_gen": FakeFollowerRefGen(),
        "follower_controller": FakeFollowerController(),
        "safety": FakeSafety(safety_decisions),
        "scheduler": FakeScheduler(),
        "leader_ref_gen": FakeLeaderRefGen(),
    }


# start() connect failure -> ABORT
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], connect_fail=True
)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT


# start() preflight failure -> ABORT
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], preflight_ok=False
)
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT


# start() success drives startup stages and telemetry open
components = build_components(
    [make_snapshot(1), make_snapshot(1)], [SafetyDecision("EXECUTE", [])]
)
app = RealMissionApp(components)
assert app.start() is True
assert components["fsm"].state() == MissionState.SETTLE
assert components["pose_source"].started is True
assert components["telemetry"].opened is not None
assert components["transport"].wait_calls == components["fleet"].all_ids()
assert components["transport"].reset_calls == components["fleet"].all_ids()
assert len(components["leader_executor"].actions) == 1
assert len(components["follower_executor"].takeoff_calls) == 1


# run() HOLD path executes hold and does not velocity-execute
components = build_components(
    [make_snapshot(1), make_snapshot(1)],
    [SafetyDecision("HOLD", ["frame"]), SafetyDecision("HOLD", ["frame"])],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app._running = True


def stop_after_hold(actions):
    components["follower_executor"].hold_calls.append(actions)
    app._running = False


components["follower_executor"].execute_hold = stop_after_hold
app.run()
assert len(components["follower_executor"].hold_calls) >= 1
assert len(components["follower_executor"].velocity_calls) == 0


# run() ABORT path triggers emergency land semantics
components = build_components(
    [make_snapshot(1), make_snapshot(1)],
    [SafetyDecision("ABORT", ["disconnect"]), SafetyDecision("ABORT", ["disconnect"])],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert components["fsm"].state() == MissionState.ABORT
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1


# no-new-pose should not recompute follower control twice
components = build_components(
    [make_snapshot(1), make_snapshot(1), make_snapshot(1)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert components["follower_controller"].calls == 1
assert len(components["telemetry"].records) >= 1
record = components["telemetry"].records[-1]
assert record.scheduler_reason == "fake_plan"
assert record.mission_state in {MissionState.RUN.value, MissionState.ABORT.value}
assert record.health
assert len(record.phase_events) >= 1
assert any(
    event["event"] in {"fsm_transition", "preflight", "trajectory_prepare"}
    for event in components["telemetry"].events
)
assert components["telemetry"].summary()["event_counts"]
assert components["telemetry"].export_replay()["phase_events"]
if any(
    event["event"] == "trajectory_prepare" for event in components["telemetry"].events
):
    assert len(components["transport"].upload_calls) >= 1
    assert len(components["transport"].define_calls) >= 1

print("[OK] RealMissionApp orchestration contracts verified")
