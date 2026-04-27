"""Crazyswarm2/crazyflie_sim adapters for the existing mission runtime."""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from typing import Any, Callable

import numpy as np

from .trajectory_common import (
    TRAJECTORY_MAX_PIECES,
    estimate_trajectory_bytes,
    validate_full_state_vector,
)

logger = logging.getLogger(__name__)


class CrazyswarmSimLinkManager:
    """Own the Crazyswarm2 client and expose the project's link-manager shape."""

    def __init__(
        self,
        fleet,
        *,
        swarm_factory: Callable[[], Any] | None = None,
        link_state_bus=None,
    ):
        self.fleet = fleet
        self._swarm_factory = swarm_factory
        self._link_state_bus = link_state_bus
        self._swarm = None
        self._cfs: dict[int, Any] = {}
        self._last_connect_report: dict[str, Any] | None = None
        self._spin_lock = threading.Lock()
        self._spin_pause = threading.Event()

    def _make_swarm(self):
        if self._swarm_factory is not None:
            return self._swarm_factory()
        from crazyflie_py import Crazyswarm

        return Crazyswarm()

    def _grouped_drone_ids(self) -> dict[int, list[int]]:
        grouped: dict[int, list[int]] = {}
        for drone_id in self.fleet.all_ids():
            grouped.setdefault(self.fleet.get_radio_group(drone_id), []).append(drone_id)
        return dict(sorted(grouped.items()))

    def last_connect_report(self) -> dict[str, Any] | None:
        return self._last_connect_report

    @property
    def swarm(self):
        return self._swarm

    def node(self):
        if self._swarm is None:
            return None
        return getattr(self._swarm, "allcfs", None)

    @contextlib.contextmanager
    def pause_spin(self):
        self._spin_pause.set()
        self._spin_lock.acquire()
        try:
            yield
        finally:
            self._spin_lock.release()
            self._spin_pause.clear()

    def spin_once(self, timeout_sec: float = 0.0) -> bool:
        node = self.node()
        if node is None or self._spin_pause.is_set():
            return False
        acquired = self._spin_lock.acquire(blocking=False)
        if not acquired:
            return False
        try:
            try:
                import rclpy
            except ImportError:
                return False

            if not rclpy.ok():
                return False
            rclpy.spin_once(node, timeout_sec=timeout_sec)
            return True
        finally:
            self._spin_lock.release()

    def connect_all(
        self,
        *,
        on_group_start: Callable[[dict[str, Any]], None] | None = None,
        on_group_result: Callable[[dict[str, Any]], None] | None = None,
        parallel_groups: bool = False,
    ):
        started_at = time.time()
        report: dict[str, Any] = {
            "ok": True,
            "connected": [],
            "failures": [],
            "radio_groups": {},
            "parallel": bool(parallel_groups),
            "per_group_duration_s": {},
        }
        self._last_connect_report = report

        self._swarm = self._make_swarm()
        by_id = getattr(getattr(self._swarm, "allcfs", None), "crazyfliesById", {})
        self._cfs.clear()

        for group_id, drone_ids in self._grouped_drone_ids().items():
            if on_group_start is not None:
                on_group_start({"group_id": group_id, "drone_ids": list(drone_ids)})
            group_started = time.time()
            group_result = {
                "group_id": group_id,
                "drone_ids": list(drone_ids),
                "connected": [],
                "failures": [],
                "ok": True,
                "duration_s": 0.0,
            }
            for drone_id in drone_ids:
                cf = by_id.get(int(drone_id))
                if cf is None:
                    group_result["ok"] = False
                    group_result["failures"].append(
                        {
                            "drone_id": drone_id,
                            "group_id": group_id,
                            "error": f"Crazyswarm2 cf id {drone_id} not found",
                            "exception_type": "KeyError",
                        }
                    )
                    break
                self._cfs[int(drone_id)] = cf
                group_result["connected"].append(int(drone_id))
                if self._link_state_bus is not None:
                    self._link_state_bus.mark_connected(int(drone_id))

            group_result["duration_s"] = time.time() - group_started
            report["radio_groups"][group_id] = group_result
            report["per_group_duration_s"][group_id] = float(group_result["duration_s"])
            report["connected"].extend(group_result["connected"])
            report["failures"].extend(group_result["failures"])
            report["ok"] = report["ok"] and group_result["ok"]
            if on_group_result is not None:
                on_group_result(group_result)
            if not group_result["ok"]:
                report["duration_s"] = time.time() - started_at
                self.close_all()
                raise RuntimeError(group_result["failures"][0]["error"])

        report["duration_s"] = time.time() - started_at
        return report

    def close_all(self):
        if self._link_state_bus is not None:
            for drone_id in list(self._cfs):
                self._link_state_bus.mark_disconnected(drone_id)
        self._cfs.clear()
        node = self.node()
        self._swarm = None
        try:
            if node is not None and hasattr(node, "destroy_node"):
                node.destroy_node()
        except Exception:
            logger.exception("Failed to destroy Crazyswarm2 API node")
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            logger.debug("Ignoring rclpy shutdown failure", exc_info=True)

    def get(self, drone_id: int) -> Any:
        if int(drone_id) not in self._cfs:
            raise KeyError(f"Drone {drone_id} is not connected")
        return self._cfs[int(drone_id)]

    def reconnect(self, drone_id: int, *, attempts: int, backoff_s: float, timeout_s: float):
        return {
            "ok": int(drone_id) in self._cfs,
            "drone_id": int(drone_id),
            "attempt_count": 1,
            "radio_group": self.fleet.get_radio_group(int(drone_id)),
        }


def _classify_command_failure(link_manager, drone_id: int, command_kind: str, exception: Exception):
    error_type = type(exception).__name__
    retryable = not isinstance(exception, (KeyError, ValueError, AttributeError))
    category = "transport_runtime"
    if isinstance(exception, KeyError):
        category = "link_lookup"
    elif isinstance(exception, TimeoutError):
        category = "timeout"
    elif isinstance(exception, ValueError):
        category = "invalid_command"
    elif isinstance(exception, AttributeError):
        category = "transport_api"
    return {
        "drone_id": int(drone_id),
        "radio_group": link_manager.fleet.get_radio_group(int(drone_id)),
        "command_kind": command_kind,
        "error": str(exception),
        "error_type": error_type,
        "failure_category": category,
        "retryable": retryable,
    }


class CrazyswarmSimCommandTransport:
    """Command transport backed by ``crazyflie_py`` objects."""

    def __init__(self, link_manager: CrazyswarmSimLinkManager):
        self.link_manager = link_manager
        self._last_velocity_command_time: dict[int, float] = {}

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _mark_velocity_command(self, drone_id: int) -> None:
        self._last_velocity_command_time[int(drone_id)] = self._now()

    def last_velocity_command_time(self, drone_id: int) -> float | None:
        return self._last_velocity_command_time.get(int(drone_id))

    def radio_group(self, drone_id: int) -> int | None:
        return self.link_manager.fleet.get_radio_group(int(drone_id))

    def classify_command_failure(self, *, drone_id: int, command_kind: str, exception: Exception):
        return _classify_command_failure(
            self.link_manager, int(drone_id), command_kind, exception
        )

    def hl_takeoff(self, drone_id: int, height: float, duration: float):
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).takeoff(
                targetHeight=height, duration=duration
            )

    def hl_land(self, drone_id: int, height: float, duration: float):
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).land(
                targetHeight=height, duration=duration
            )

    def hl_go_to(self, drone_id: int, x: float, y: float, z: float, duration: float):
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).goTo(
                [x, y, z], yaw=0.0, duration=duration
            )

    def cmd_velocity_world(self, drone_id: int, vx: float, vy: float, vz: float):
        with self.link_manager.pause_spin():
            cf = self.link_manager.get(drone_id)
            pos = _current_position(cf)
            cf.cmdFullState(
                pos,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                yaw=0.0,
                omega=[0.0, 0.0, 0.0],
            )
        self._mark_velocity_command(drone_id)

    def cmd_full_state(
        self,
        drone_id: int,
        pos: tuple[float, float, float],
        vel: tuple[float, float, float],
        acc: tuple[float, float, float],
    ):
        validate_full_state_vector("pos", pos)
        validate_full_state_vector("vel", vel)
        validate_full_state_vector("acc", acc)
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).cmdFullState(
                list(pos), list(vel), list(acc), yaw=0.0, omega=[0.0, 0.0, 0.0]
            )
        self._mark_velocity_command(drone_id)

    def set_onboard_controller(self, drone_id: int, controller: str) -> None:
        mapping = {"pid": 1, "mellinger": 2, "indi": 3}
        if controller not in mapping:
            raise ValueError(f"Unsupported onboard controller: {controller}")
        cf = self.link_manager.get(drone_id)
        param_types = getattr(cf, "paramTypeDict", None)
        if isinstance(param_types, dict) and "stabilizer.controller" not in param_types:
            logger.debug(
                "Crazyswarm2 sim has no stabilizer.controller param for drone %s; "
                "using launch-file controller config",
                drone_id,
            )
            return
        with self.link_manager.pause_spin():
            cf.setParam("stabilizer.controller", mapping[controller])

    def notify_setpoint_stop(self, drone_id: int):
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).notifySetpointsStop()

    def stop_high_level_commander(self, drone_id: int):
        with self.link_manager.pause_spin():
            cf = self.link_manager.get(drone_id)
            stop = getattr(cf, "stop", None)
            if callable(stop):
                stop()

    def hl_define_trajectory(
        self,
        drone_id: int,
        trajectory_id: int,
        offset: int,
        n_pieces: int,
        trajectory_type: str = "poly4d",
    ):
        if int(n_pieces) > TRAJECTORY_MAX_PIECES:
            raise RuntimeError(
                f"Trajectory has too many pieces for drone {drone_id}: "
                f"{n_pieces} > {TRAJECTORY_MAX_PIECES}"
            )

    def hl_start_trajectory(
        self,
        drone_id: int,
        trajectory_id: int,
        time_scale: float = 1.0,
        relative_position: bool = False,
        relative_yaw: bool = False,
        reversed: bool = False,
    ):
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).startTrajectory(
                trajectory_id,
                timescale=time_scale,
                reverse=reversed,
                relative=relative_position,
            )

    def wait_for_params(self, drone_id: int):
        self.link_manager.get(drone_id)

    def reset_estimator_and_wait(self, drone_id: int):
        self.link_manager.get(drone_id)

    def read_health_snapshot(self, drone_id: int) -> dict:
        cf = self.link_manager.get(drone_id)
        status = _status_values(cf)
        return {
            "params_updated": True,
            "link_open": True,
            "pm.vbat": status.get("pm.vbat", 4.0),
        }

    def upload_trajectory(
        self,
        drone_id: int,
        pieces: list,
        start_addr: int = 0,
        trajectory_type: str = "poly4d",
    ) -> int:
        if len(pieces) > TRAJECTORY_MAX_PIECES:
            raise RuntimeError(
                f"Trajectory has too many pieces for drone {drone_id}: "
                f"{len(pieces)} > {TRAJECTORY_MAX_PIECES}"
            )
        trajectory_id = 1
        trajectory = pieces_to_crazyswarm_trajectory(pieces)
        with self.link_manager.pause_spin():
            self.link_manager.get(drone_id).uploadTrajectory(
                trajectory_id, int(start_addr), trajectory
            )
        return len(pieces)

    def upload_trajectories_by_group(
        self,
        uploads: dict[int, dict[str, object]],
        *,
        parallel_groups: bool = False,
    ) -> dict[int, dict[str, object]]:
        results: dict[int, dict[str, object]] = {}
        for drone_id in sorted(uploads):
            spec = uploads[drone_id]
            pieces = spec.get("pieces", [])
            start_addr = int(spec.get("start_addr", 0))
            trajectory_id = int(spec.get("trajectory_id", 1))
            trajectory = pieces_to_crazyswarm_trajectory(pieces)
            with self.link_manager.pause_spin():
                self.link_manager.get(drone_id).uploadTrajectory(
                    trajectory_id, start_addr, trajectory
                )
            trajectory_type = str(spec.get("trajectory_type", "poly4d"))
            results[int(drone_id)] = {
                "piece_count": len(pieces),
                "trajectory_id": trajectory_id,
                "start_addr": start_addr,
                "trajectory_type": trajectory_type,
                "estimated_bytes": estimate_trajectory_bytes(pieces, trajectory_type),
            }
        return results


def pieces_to_crazyswarm_trajectory(pieces: list, trajectory_module=None):
    """Convert project ``TrajectoryPiece`` objects to Crazyswarm2 objects."""

    if trajectory_module is None:
        from crazyflie_py import uav_trajectory as trajectory_module

    trajectory = trajectory_module.Trajectory()
    trajectory.polynomials = [
        trajectory_module.Polynomial4D(
            float(piece.duration),
            np.array(piece.x, dtype=float),
            np.array(piece.y, dtype=float),
            np.array(piece.z, dtype=float),
            np.array(piece.yaw, dtype=float),
        )
        for piece in pieces
    ]
    trajectory.duration = float(sum(float(piece.duration) for piece in pieces))
    return trajectory


class CrazyswarmSimPoseSource:
    """Poll Crazyswarm2 pose/status data and feed PoseBus/HealthBus callbacks."""

    def __init__(
        self,
        link_manager: CrazyswarmSimLinkManager,
        fleet_model,
        log_freq_hz=10.0,
        *,
        attitude_log_enabled: bool = True,
        motor_log_enabled: bool = True,
    ):
        self.link_manager = link_manager
        self.fleet = fleet_model
        self.period_s = 1.0 / float(log_freq_hz)
        self.attitude_log_enabled = bool(attitude_log_enabled)
        self.motor_log_enabled = bool(motor_log_enabled)
        self._callbacks = []
        self._health_callbacks = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_positions: dict[int, tuple[float, np.ndarray]] = {}

    def register_callback(self, callback):
        self._callbacks.append(callback)

    def register_health_callback(self, callback):
        self._health_callbacks.append(callback)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run_loop,
            name="crazyswarm-sim-pose-source",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def reattach_drone(self, drone_id: int) -> bool:
        return self._running and int(drone_id) in set(self.fleet.all_ids())

    def _run_loop(self):
        while self._running:
            for _ in range(max(10, len(self.fleet.all_ids()) * 4)):
                self.link_manager.spin_once(timeout_sec=0.001)
            now = time.time()
            for drone_id in self.fleet.all_ids():
                try:
                    cf = self.link_manager.get(drone_id)
                except KeyError:
                    continue
                if not _has_pose_sample(cf):
                    continue
                pos = np.array(_current_position(cf), dtype=float)
                vel = self._estimate_velocity(int(drone_id), pos, now)
                for cb in list(self._callbacks):
                    try:
                        cb(int(drone_id), pos, now, vel)
                    except TypeError:
                        cb(int(drone_id), pos, now)
                health = _status_values(cf)
                for cb in list(self._health_callbacks):
                    cb(int(drone_id), health, now)
            time.sleep(self.period_s)

    def _estimate_velocity(self, drone_id: int, pos: np.ndarray, now: float):
        previous = self._last_positions.get(drone_id)
        self._last_positions[drone_id] = (now, pos.copy())
        if previous is None:
            return np.zeros(3, dtype=float)
        prev_t, prev_pos = previous
        dt = max(now - prev_t, 1e-6)
        return (pos - prev_pos) / dt


def _current_position(cf) -> list[float]:
    try:
        pos = cf.get_position()
    except Exception:
        pos = getattr(cf, "position", None)
    if pos is None:
        return [0.0, 0.0, 0.0]
    return [float(pos[0]), float(pos[1]), float(pos[2])]


def _has_pose_sample(cf) -> bool:
    pose_stamped = getattr(cf, "poseStamped", None)
    if pose_stamped is None:
        return True
    return bool(pose_stamped)


def _status_values(cf) -> dict[str, float | str]:
    try:
        status = cf.get_status() or {}
    except Exception:
        status = {}
    battery = status.get("battery", status.get("battery_voltage", 4.0))
    values: dict[str, float | str] = {
        "pm.vbat": float(battery) if battery is not None else 4.0,
        "kalman.varPX": 1e-6,
        "kalman.varPY": 1e-6,
        "kalman.varPZ": 1e-6,
        "sim.source": "crazyswarm2",
    }
    if "rssi" in status:
        try:
            values["radio.rssi"] = float(status["rssi"])
        except (TypeError, ValueError):
            pass
    return values
