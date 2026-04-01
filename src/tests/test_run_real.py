"""RealMissionApp orchestration contract tests with fakes"""

import numpy as np
from pathlib import Path

from src.app.run_real import RealMissionApp
from src.runtime.manual_input_port import ManualLeaderIntent
from src.runtime.manual_leader_reference import ManualLeaderReferenceSource
from src.runtime.manual_leader_state import ManualLeaderState
from src.adapters.manual_input_keyboard import KeyboardManualInputSource
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

    def is_leader(self, drone_id):
        return drone_id in self._leaders

    def is_follower(self, drone_id):
        return drone_id in self._followers


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


class FakeManualInput:
    def __init__(self, intents=None):
        self.intents = list(intents or [])
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def poll(self):
        if self.intents:
            return self.intents.pop(0)
        return None


class FakeKeyReader:
    def __init__(self, keys):
        self.keys = list(keys)

    def __call__(self):
        if self.keys:
            return self.keys.pop(0)
        return None


class FakeLeaderRefGen:
    def __init__(self, mode="trajectory"):
        self.mode = mode

    def reference_at(self, t):
        return type(
            "LeaderRef",
            (),
            {
                "leader_ids": [1, 2, 3, 4],
                "mode": self.mode,
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
        t_meas=float(seq - 1),
        positions=positions,
        fresh_mask=np.ones(6, dtype=bool) if fresh else np.zeros(6, dtype=bool),
        disconnected_ids=[] if fresh else [1, 2, 3, 4, 5, 6],
    )


def build_components(
    start_snapshots,
    safety_decisions,
    connect_fail=False,
    preflight_ok=True,
    startup_mode="auto",
    leader_ref_mode=None,
    manual_input=None,
    manual_state=None,
):
    if leader_ref_mode is None:
        leader_ref_mode = "trajectory" if startup_mode == "auto" else "batch_goto"

    fake_config = type(
        "FakeConfig",
        (),
        {
            "startup": type(
                "FakeStartup",
                (),
                {
                    "mode": startup_mode,
                    "manual": type(
                        "FakeManualCfg",
                        (),
                        {"default_axis": "z"},
                    )()
                    if startup_mode == "manual_leader"
                    else None,
                },
            )(),
            "comm": type(
                "FakeComm",
                (),
                {
                    "readiness_wait_for_params": True,
                    "readiness_reset_estimator": True,
                },
            )(),
            "mission": type("FakeMission", (), {"duration": 20.0})(),
            "safety": type("FakeSafetyCfg", (), {"hold_auto_land_timeout": 0.2})(),
        },
    )()

    fake_health_bus = type(
        "FakeHealthBus",
        (),
        {
            "latest": lambda self: {
                did: type("Sample", (), {"values": {"pm.vbat": 4.0}, "t_meas": 0.0})()
                for did in [1, 2, 3, 4, 5, 6]
            },
            "update": lambda self, drone_id, values, t_meas: None,
        },
    )()

    return {
        "config": fake_config,
        "startup_mode": startup_mode,
        "fsm": MissionFSM(),
        "fleet": FakeFleet(),
        "mission_profile": type(
            "FakeMissionProfile",
            (),
            {
                "phase_at": lambda self, t: type(
                    "Phase", (), {"name": "formation_run"}
                )(),
                "total_time": lambda self: 20.0,
            },
        )(),
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
        "leader_ref_gen": FakeLeaderRefGen(mode=leader_ref_mode),
        "manual_input": manual_input,
        "manual_leader_state": manual_state,
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
assert Path(components["telemetry"].opened).name.startswith("run_real_")
assert Path(components["telemetry"].opened).suffix == ".jsonl"
assert components["telemetry_path"] == components["telemetry"].opened
assert components["transport"].wait_calls == components["fleet"].all_ids()
assert components["transport"].reset_calls == components["fleet"].all_ids()
assert any(event["event"] == "startup_mode" for event in components["telemetry"].events)
assert any(event["event"] == "health_ready" for event in components["telemetry"].events)
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "takeoff" for action in batch)
    for batch in components["leader_executor"].actions
)
assert any(
    event["event"] in {"trajectory_prepare", "formation_align"}
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "trajectory_budget_check"
    for event in components["telemetry"].events
)
budget_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "trajectory_budget_check"
)
assert budget_event["details"]["fits_memory"] is True
assert any(
    event["event"] == "trajectory_readiness_summary"
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "trajectory_state" and event["details"]["state"] == "ready"
    for event in components["telemetry"].events
)
readiness_event = next(
    event
    for event in components["telemetry"].events
    if event["event"] == "trajectory_readiness_summary"
)
assert readiness_event["details"]["ok"] is True
assert set(readiness_event["details"]["leaders"].keys()) == {1, 2, 3, 4}
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
assert components["fsm"].state() == MissionState.HOLD


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
abort_record = components["telemetry"].records[-1]
assert abort_record.safety_action == "ABORT"
assert abort_record.scheduler_reason == "emergency_land"
assert abort_record.trajectory_state == "terminated"
assert abort_record.trajectory_terminal_reason == "abort"
assert any(
    event["event"] == "emergency_land" for event in components["telemetry"].events
)


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
assert len(components["telemetry"].records) >= 2
record = components["telemetry"].records[-2]
terminal_record = components["telemetry"].records[-1]
assert record.scheduler_reason == "fake_plan"
assert terminal_record.scheduler_reason == "emergency_land"
assert terminal_record.trajectory_state == "terminated"
assert terminal_record.trajectory_terminal_reason == "abort"
assert record.mission_state in {MissionState.RUN.value, MissionState.ABORT.value}
assert record.phase_label == "formation_run"
assert record.measured_positions[1] == [1.0, 0.0, 0.8]
assert record.fresh_mask[1] is True
assert record.disconnected_ids == []
assert record.leader_mode == "trajectory"
assert record.health
assert len(record.phase_events) >= 1
assert record.startup_mode == "auto"
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

# run() can recover from HOLD when safety recovers
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3)],
    [
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "hold_recovered" for event in components["telemetry"].events
)
assert any(event["event"] == "hold_entered" for event in components["telemetry"].events)
assert components["follower_controller"].calls >= 1


# sustained HOLD auto-lands after timeout
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3), make_snapshot(4)],
    [
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
        SafetyDecision("HOLD", ["frame"]),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "hold_timeout_land" for event in components["telemetry"].events
)
timeout_record = components["telemetry"].records[-1]
assert timeout_record.safety_action == "HOLD_TIMEOUT"
assert timeout_record.scheduler_reason == "hold_timeout"
assert timeout_record.trajectory_terminal_reason == "hold_timeout"
assert len(components["follower_executor"].land_calls) == 1

# start() failure path cleans up resources
components = build_components(
    [make_snapshot(1)], [SafetyDecision("EXECUTE", [])], preflight_ok=False
)
app = RealMissionApp(components)
assert app.start() is False
assert components["telemetry"].closed is True
assert components["pose_source"].stopped is True


# start() fails if health is never ready
components = build_components([make_snapshot(1)], [SafetyDecision("EXECUTE", [])])
components["health_bus"] = type(
    "NoHealthBus",
    (),
    {
        "latest": lambda self: {},
        "update": lambda self, drone_id, values, t_meas: None,
    },
)()
app = RealMissionApp(components)
assert app.start() is False
assert components["fsm"].state() == MissionState.ABORT
assert any(
    event["event"] == "health_ready" and event["details"]["ok"] is False
    for event in components["telemetry"].events
)


# shutdown() from active mission performs symmetric landing and flushes telemetry
components = build_components(
    [make_snapshot(1), make_snapshot(1)], [SafetyDecision("EXECUTE", [])]
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.RUN
app.shutdown()
assert components["fsm"].state() == MissionState.LAND
assert len(components["leader_executor"].actions) >= 1
assert any(
    any(action.kind == "land" for action in batch)
    for batch in components["leader_executor"].actions
)
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1
assert components["telemetry"].closed is True
assert len(components["telemetry"].records) >= 1
shutdown_record = components["telemetry"].records[-1]
assert shutdown_record.safety_action == "SHUTDOWN"
assert shutdown_record.scheduler_reason == "shutdown"
assert shutdown_record.trajectory_state == "terminated"
assert shutdown_record.trajectory_terminal_reason == "shutdown"
assert any(
    event["event"] == "manual_shutdown_land" for event in components["telemetry"].events
)
assert any(event["event"] == "shutdown" for event in components["telemetry"].events)


# auto mode ends by mission duration and lands automatically
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(25)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
    ],
)
app = RealMissionApp(components)
components["fsm"]._state = MissionState.SETTLE
app.run()
assert any(
    event["event"] == "mission_complete" for event in components["telemetry"].events
)
assert any(
    event["event"] == "mission_complete_land"
    for event in components["telemetry"].events
)
assert len(components["follower_executor"].stop_calls) == 1
assert len(components["follower_executor"].land_calls) == 1
mission_complete_record = components["telemetry"].records[-1]
assert mission_complete_record.safety_action == "MISSION_COMPLETE"
assert mission_complete_record.scheduler_reason == "mission_complete"
assert mission_complete_record.trajectory_state == "terminated"
assert mission_complete_record.trajectory_terminal_reason == "mission_complete"


# manual startup mode skips auto-only leader startup behavior
manual_state = ManualLeaderState(default_axis="z")
manual_input = FakeManualInput(
    [
        ManualLeaderIntent(axis_switch=True),
        ManualLeaderIntent(target_switch=True),
        ManualLeaderIntent(translation_delta=(0.2, 0.0, 0.0)),
        ManualLeaderIntent(rotation_delta_deg=5.0),
    ]
)
manual_source = ManualLeaderReferenceSource(
    type(
        "Formation",
        (),
        {
            "nominal_position": lambda self, drone_id: np.array(
                [float(drone_id), 0.0, 0.5]
            )
        },
    )(),
    FakeFleet(),
    manual_state,
)
components = build_components(
    [make_snapshot(1), make_snapshot(2), make_snapshot(3)],
    [
        SafetyDecision("EXECUTE", []),
        SafetyDecision("EXECUTE", []),
        SafetyDecision("ABORT", ["stop"]),
    ],
    startup_mode="manual_leader",
    leader_ref_mode="batch_goto",
    manual_input=manual_input,
    manual_state=manual_state,
)
components["leader_ref_gen"] = manual_source
app = RealMissionApp(components)
assert app.start() is True
assert not any(
    event["event"] == "trajectory_prepare" for event in components["telemetry"].events
)
assert not any(
    event["event"] == "formation_align" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_structure_align"
    for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_mode_armed" for event in components["telemetry"].events
)
app.run()
assert manual_input.started is True
assert any(
    event["event"] == "manual_input_started" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_intent" for event in components["telemetry"].events
)
assert any(
    event["event"] == "manual_axis_switch" for event in components["telemetry"].events
)
manual_record = components["telemetry"].records[-1]
assert manual_record.startup_mode == "manual_leader"
assert manual_record.leader_reference_source == "ManualLeaderReferenceSource"
assert manual_record.manual_axis in {"x", "y", "z"}
assert any(
    event["event"] == "manual_intent" for event in components["telemetry"].events
)


# keyboard input adapter maps keys into ManualLeaderIntent
reader = FakeKeyReader(["w", "q", "x", "c", "v", "?"])
keyboard_input = KeyboardManualInputSource(
    translation_step=0.2,
    vertical_step=0.15,
    scale_step=0.05,
    rotation_step_deg=7.0,
    key_reader=reader,
)
keyboard_input.start()
intent = keyboard_input.poll()
assert intent is not None and intent.translation_delta == (0.2, 0.0, 0.0)
intent = keyboard_input.poll()
assert intent is not None and intent.scale_delta == 0.05
intent = keyboard_input.poll()
assert intent is not None and intent.rotation_delta_deg == 7.0
intent = keyboard_input.poll()
assert intent is not None and intent.axis_switch is True
intent = keyboard_input.poll()
assert intent is not None and intent.target_switch is True
assert keyboard_input.poll() is None


# manual leader state supports target switching between swarm and individual leaders
state = ManualLeaderState(default_axis="z")
state.apply_intent(ManualLeaderIntent(target_switch=True))
snap = state.snapshot()
assert snap.target_mode == "leader"
assert snap.selected_leader_id == 1
state.apply_intent(ManualLeaderIntent(translation_delta=(0.3, 0.0, 0.0)))
snap = state.snapshot()
assert np.allclose(snap.per_leader_offsets[1], np.array([0.3, 0.0, 0.0]))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
state.apply_intent(ManualLeaderIntent(target_switch=True))
snap = state.snapshot()
assert snap.target_mode == "swarm"
assert snap.selected_leader_id is None

print("[OK] RealMissionApp orchestration contracts verified")
