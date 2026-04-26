"""Shared fakes and builders for RealMissionApp tests."""

import time

import numpy as np

from src.runtime.follower_controller import FollowerCommandSet
from src.runtime.manual_input_port import ManualLeaderIntent
from src.runtime.mission_fsm import MissionFSM
from src.runtime.pose_snapshot import PoseSnapshot
from src.runtime.safety_manager import SafetyDecision


class FakeFleet:
    def __init__(self):
        self._leaders = [1, 2, 3, 4]
        self._followers = [5, 6]
        self._all = self._leaders + self._followers
        self._id_to_index = {drone_id: i for i, drone_id in enumerate(self._all)}
        self._id_to_radio = {1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2}

    def leader_ids(self):
        return self._leaders.copy()

    def follower_ids(self):
        return self._followers.copy()

    def all_ids(self):
        return self._all.copy()

    def id_to_index(self, drone_id):
        return self._id_to_index[drone_id]

    def get_radio_group(self, drone_id):
        return self._id_to_radio[drone_id]

    def is_leader(self, drone_id):
        return drone_id in self._leaders

    def is_follower(self, drone_id):
        return drone_id in self._followers


class FakeLinkManager:
    def __init__(self, fleet, should_fail=False, failed_drones=None):
        self.fleet = fleet
        self.should_fail = should_fail
        self.failed_drones = set(failed_drones or [])
        self.closed = False
        self._last_connect_report = None

    def last_connect_report(self):
        return self._last_connect_report

    def connect_all(self, *, on_group_start=None, on_group_result=None, parallel_groups=False):
        grouped = {}
        for drone_id in self.fleet.all_ids():
            group_id = self.fleet.get_radio_group(drone_id)
            grouped.setdefault(group_id, []).append(drone_id)

        report = {
            "ok": True,
            "connected": [],
            "failures": [],
            "radio_groups": {},
            "duration_s": 0.0,
        }
        self._last_connect_report = report

        for group_id, drone_ids in sorted(grouped.items()):
            if on_group_start is not None:
                on_group_start({"group_id": group_id, "drone_ids": list(drone_ids)})

            group_result = {
                "group_id": group_id,
                "drone_ids": list(drone_ids),
                "connected": [],
                "failures": [],
                "ok": True,
                "duration_s": 0.0,
            }
            for drone_id in drone_ids:
                if self.should_fail and not self.failed_drones:
                    self.failed_drones.add(drone_id)
                if drone_id in self.failed_drones:
                    group_result["ok"] = False
                    group_result["failures"].append(
                        {
                            "drone_id": drone_id,
                            "group_id": group_id,
                            "error": "connect failed",
                            "exception_type": "RuntimeError",
                        }
                    )
                    break
                group_result["connected"].append(drone_id)

            report["radio_groups"][group_id] = group_result
            report["connected"].extend(group_result["connected"])
            report["failures"].extend(group_result["failures"])
            report["ok"] = report["ok"] and group_result["ok"]

            if on_group_result is not None:
                on_group_result(group_result)

            if not group_result["ok"]:
                self.close_all()
                raise RuntimeError("connect failed")

        return report

    def close_all(self):
        self.closed = True


class FakeTransport:
    def __init__(self):
        self.wait_calls = []
        self.reset_calls = []
        self.controller_calls = []
        self.upload_calls = []
        self.define_calls = []
        self.velocity_calls = []
        self.high_level_calls = []
        self._last_velocity_command_time = {}

    def wait_for_params(self, drone_id):
        self.wait_calls.append(drone_id)

    def reset_estimator_and_wait(self, drone_id):
        self.reset_calls.append(drone_id)

    def set_onboard_controller(self, drone_id, controller):
        self.controller_calls.append((drone_id, controller))

    def upload_trajectory(self, drone_id, pieces, start_addr=0):
        self.upload_calls.append((drone_id, pieces, start_addr))
        return len(pieces)

    def upload_trajectories_by_group(self, uploads, *, parallel_groups=False):
        results = {}
        for drone_id, spec in uploads.items():
            piece_count = self.upload_trajectory(
                drone_id, spec.get("pieces", []), start_addr=spec.get("start_addr", 0)
            )
            self.hl_define_trajectory(
                drone_id,
                spec.get("trajectory_id", 1),
                spec.get("start_addr", 0),
                piece_count,
            )
            results[drone_id] = {
                "piece_count": piece_count,
                "trajectory_id": spec.get("trajectory_id", 1),
                "start_addr": spec.get("start_addr", 0),
                "parallel_groups": parallel_groups,
            }
        return results

    def hl_define_trajectory(self, drone_id, trajectory_id, offset, n_pieces):
        self.define_calls.append((drone_id, trajectory_id, offset, n_pieces))

    def hl_takeoff(self, drone_id, height, duration):
        self.high_level_calls.append(("takeoff", drone_id, height, duration))

    def hl_land(self, drone_id, height, duration):
        self.high_level_calls.append(("land", drone_id, height, duration))

    def hl_go_to(self, drone_id, x, y, z, duration):
        self.high_level_calls.append(("go_to", drone_id, x, y, z, duration))

    def hl_start_trajectory(
        self,
        drone_id,
        trajectory_id,
        time_scale=1.0,
        relative_position=False,
        relative_yaw=False,
        reversed=False,
    ):
        self.high_level_calls.append(
            (
                "start_trajectory",
                drone_id,
                trajectory_id,
                time_scale,
                relative_position,
                relative_yaw,
                reversed,
            )
        )

    def cmd_velocity_world(self, drone_id, vx, vy, vz):
        self.velocity_calls.append((drone_id, vx, vy, vz))
        self._last_velocity_command_time[drone_id] = time.monotonic()

    def notify_setpoint_stop(self, drone_id):
        self.high_level_calls.append(("notify_stop", drone_id))

    def last_velocity_command_time(self, drone_id):
        return self._last_velocity_command_time.get(drone_id)


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
        self.header = None

    def open(self, path):
        self.opened = path

    def write_header(self, *, config_fingerprint=None, readiness=None, fleet_meta=None):
        self.header = {
            "config_fingerprint": config_fingerprint or {},
            "readiness": readiness or {},
            "fleet": fleet_meta or {},
        }

    def log(self, record):
        self.records.append(record)

    def record_event(self, event, **details):
        self.events.append({"event": event, "details": details})

    def phase_events(self):
        return list(self.events)

    def summary(self):
        return {"event_counts": {event["event"]: 1 for event in self.events}}

    def close(self):
        self.closed = True


class FakeLeaderExecutor:
    def __init__(self):
        self.actions = []

    def execute(self, actions):
        self.actions.append(actions)
        return [
            {
                "kind": action.kind,
                "successes": list(action.drone_ids),
                "failures": [],
            }
            for action in actions
        ]


class FakeFollowerExecutor:
    def __init__(self):
        self.takeoff_calls = []
        self.hold_calls = []
        self.velocity_calls = []
        self.land_calls = []
        self.stop_calls = []
        self.velocity_result = None
        self.hold_result = None

    def takeoff(self, drone_ids, height=0.5, duration=2.0):
        self.takeoff_calls.append((list(drone_ids), height, duration))
        return {"kind": "takeoff", "successes": list(drone_ids), "failures": []}

    def execute_hold(self, actions):
        self.hold_calls.append(actions)
        if self.hold_result is not None:
            return self.hold_result
        return {
            "kind": "hold",
            "successes": [action.drone_id for action in actions],
            "failures": [],
        }

    def execute_velocity(self, actions):
        self.velocity_calls.append(actions)
        if self.velocity_result is not None:
            return self.velocity_result
        return {
            "kind": "velocity",
            "successes": [action.drone_id for action in actions],
            "failures": [],
        }

    def land(self, drone_ids, duration=2.0):
        self.land_calls.append((list(drone_ids), duration))
        return {"kind": "land", "successes": list(drone_ids), "failures": []}

    def stop_velocity_mode(self, drone_ids):
        self.stop_calls.append(list(drone_ids))
        return {
            "kind": "notify_stop",
            "successes": list(drone_ids),
            "failures": [],
        }


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
    def compute(self, leader_positions, t_meas=None):
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
                "target_velocities": None,
                "target_accelerations": None,
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

    def fast_gate(self, snapshot):
        # FakeSafety 默认不拦截；拦截路径由 evaluate() 返回 ABORT 触发
        return (False, [])

    def evaluate(
        self,
        snapshot,
        frame=None,
        commands=None,
        follower_ref=None,
        health=None,
        ignored_disconnected_ids=None,
    ):
        idx = min(self.calls, len(self.decisions) - 1)
        self.calls += 1
        return self.decisions[idx]


class FakeScheduler:
    def __init__(self):
        self.calls = []
        self.parked_history = []

    def plan(
        self,
        snapshot,
        mission_state,
        leader_ref,
        commands,
        safety_decision,
        parked_follower_ids=None,
    ):
        from src.runtime.command_plan import HoldAction

        self.calls.append(
            (snapshot.seq, mission_state, commands, safety_decision.action)
        )
        self.parked_history.append(list(parked_follower_ids or []))
        return type(
            "Plan",
            (),
            {
                "leader_actions": [],
                "follower_actions": [],
                "hold_actions": [
                    HoldAction(drone_id=drone_id)
                    for drone_id in (parked_follower_ids or [])
                ],
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
    failed_connect_drones=None,
    preflight_ok=True,
    startup_mode="auto",
    leader_ref_mode=None,
    manual_input=None,
    manual_state=None,
    watchdog_action="telemetry",
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
                    "follower_tx_freq": 8.0,
                    "readiness_wait_for_params": True,
                    "readiness_reset_estimator": True,
                    "connect_groups_in_parallel": False,
                    "trajectory_upload_groups_in_parallel": False,
                },
            )(),
            "mission": type(
                "FakeMission",
                (),
                {
                    "duration": 20.0,
                    "leader_motion": type(
                        "FakeLeaderMotion",
                        (),
                        {"trajectory_enabled": leader_ref_mode == "trajectory"},
                    )(),
                },
            )(),
            "safety": type(
                "FakeSafetyCfg",
                (),
                {
                    "hold_auto_land_timeout": 0.2,
                    "velocity_stream_watchdog_action": watchdog_action,
                },
            )(),
            "control": type(
                "FakeControlCfg",
                (),
                {
                    "output_mode": "velocity",
                    "onboard_controller": "pid",
                    "dynamics_model_order": 2,
                    "max_velocity": 0.65,
                    "max_acceleration": 1.5,
                },
            )(),
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

    fleet = FakeFleet()

    return {
        "config": fake_config,
        "config_dir": "config",
        "repo_root": "repo",
        "startup_mode": startup_mode,
        "fsm": MissionFSM(),
        "fleet": fleet,
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
        "link_manager": FakeLinkManager(
            fleet,
            should_fail=connect_fail,
            failed_drones=failed_connect_drones,
        ),
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