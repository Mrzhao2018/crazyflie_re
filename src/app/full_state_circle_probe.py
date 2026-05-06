"""Single-drone full-state/Mellinger circular trajectory probe."""

from __future__ import annotations

import argparse
import logging
import math
import time

import numpy as np

from ..adapters.cflib_command_transport import CflibCommandTransport
from ..adapters.cflib_link_manager import CflibLinkManager
from ..adapters.lighthouse_pose_source import LighthousePoseSource
from ..config.loader import ConfigLoader
from ..runtime.pose_bus import PoseBus
from .full_state_probe import _build_single_fleet, _latest_fresh_position
from .param_probe import format_diagnostic_values

logger = logging.getLogger(__name__)


def circle_setpoint(
    center: np.ndarray,
    *,
    radius: float,
    period_s: float,
    t_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return analytic position, velocity and acceleration for a horizontal circle."""
    omega = 2.0 * math.pi / float(period_s)
    phase = omega * float(t_s)
    cos_p = math.cos(phase)
    sin_p = math.sin(phase)

    pos = np.array(
        [
            center[0] + radius * cos_p,
            center[1] + radius * sin_p,
            center[2],
        ],
        dtype=float,
    )
    vel = np.array(
        [
            -radius * omega * sin_p,
            radius * omega * cos_p,
            0.0,
        ],
        dtype=float,
    )
    acc = np.array(
        [
            -radius * omega * omega * cos_p,
            -radius * omega * omega * sin_p,
            0.0,
        ],
        dtype=float,
    )
    return pos, vel, acc


def _tuple3(vector: np.ndarray) -> tuple[float, float, float]:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


def tracking_error_components(current: np.ndarray, target: np.ndarray) -> dict[str, float]:
    delta = np.asarray(current, dtype=float) - np.asarray(target, dtype=float)
    return {
        "err": float(np.linalg.norm(delta)),
        "err_xy": float(np.linalg.norm(delta[:2])),
        "err_z": float(delta[2]),
    }


def apply_z_bias_compensation(
    target: np.ndarray,
    *,
    z_bias: float,
    max_abs_bias: float,
) -> np.ndarray:
    compensated = np.array(target, dtype=float).copy()
    limited_bias = float(np.clip(z_bias, -abs(max_abs_bias), abs(max_abs_bias)))
    compensated[2] -= limited_bias
    return compensated


def parse_param_assignment(assignment: str) -> tuple[str, str]:
    if "=" not in assignment:
        raise ValueError(
            "--set-param expects NAME=VALUE, for example ctrlMel.ki_z=0"
        )
    name, value = assignment.split("=", 1)
    name = name.strip()
    value = value.strip()
    if not name or value == "":
        raise ValueError(
            "--set-param expects NAME=VALUE, for example ctrlMel.ki_z=0"
        )
    return name, value


def apply_param_overrides(scf, assignments: list[str] | None) -> dict[str, str]:
    originals: dict[str, str] = {}
    for assignment in assignments or []:
        name, value = parse_param_assignment(assignment)
        if name not in originals:
            originals[name] = str(scf.cf.param.get_value(name))
        print(f"Set param override: {name}={value} (was {originals[name]})")
        scf.cf.param.set_value(name, value)
    return originals


def restore_param_overrides(scf, originals: dict[str, str]) -> None:
    for name, value in reversed(list(originals.items())):
        print(f"Restore param: {name}={value}")
        scf.cf.param.set_value(name, value)


def _switch_to_full_state_mode(
    *,
    link_manager: CflibLinkManager,
    transport: CflibCommandTransport,
    drone_id: int,
    controller: str,
    notify_before_full_state: bool,
) -> None:
    if notify_before_full_state:
        print("Notify setpoint stop before full-state streaming...")
        transport.notify_setpoint_stop(drone_id)
    link_manager.get(drone_id).cf.high_level_commander.stop()
    transport.set_onboard_controller(drone_id, controller)


def _stream_hold(
    *,
    transport: CflibCommandTransport,
    drone_id: int,
    target: np.ndarray,
    duration_s: float,
    rate_hz: float,
) -> None:
    if duration_s <= 0.0:
        return
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        transport.cmd_full_state(
            drone_id,
            _tuple3(target),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        time.sleep(period)


def run(args: argparse.Namespace) -> int:
    config = ConfigLoader.load(args.config_dir)
    drone_id = int(args.drone_id)
    radius = float(args.radius)
    period_s = float(args.period_s)
    cycles = float(args.cycles)
    rate_hz = float(args.rate_hz)
    max_z_bias = float(getattr(args, "max_z_bias", 0.08))
    if radius <= 0.0:
        raise ValueError("--radius must be > 0")
    if period_s <= 0.0:
        raise ValueError("--period-s must be > 0")
    if cycles <= 0.0:
        raise ValueError("--cycles must be > 0")
    if rate_hz <= 0.0:
        raise ValueError("--rate-hz must be > 0")
    if max_z_bias < 0.0:
        raise ValueError("--max-z-bias must be >= 0")

    fleet = _build_single_fleet(config, drone_id)
    link_manager = CflibLinkManager(
        fleet,
        connect_pace_s=config.comm.connect_pace_s,
        connect_timeout_s=config.comm.connect_timeout_s,
        radio_driver=config.comm.radio_driver,
    )
    transport = CflibCommandTransport(link_manager)
    pose_bus = PoseBus(fleet, config.safety.pose_timeout)
    pose_source = LighthousePoseSource(
        link_manager,
        fleet,
        config.comm.pose_log_freq,
        attitude_log_enabled=bool(getattr(args, "diagnostic_log", False)),
        motor_log_enabled=bool(getattr(args, "diagnostic_log", False)),
    )
    pose_source.register_callback(
        lambda did, pos, timestamp, velocity=None: pose_bus.update_agent(
            did, pos, timestamp, velocity
        )
    )
    latest_health: dict[int, dict] = {}
    if getattr(args, "diagnostic_log", False):
        pose_source.register_health_callback(
            lambda did, values, timestamp: latest_health.__setitem__(
                int(did), dict(values)
            )
        )

    print(f"=== Full-state circle probe: drone {drone_id} ===")
    print(
        f"radius={radius:.3f}m period={period_s:.2f}s "
        f"cycles={cycles:.2f} rate={rate_hz:.1f}Hz"
    )
    param_originals: dict[str, str] = {}
    scf = None

    try:
        link_manager.connect_all(parallel_groups=False)
        transport.wait_for_params(drone_id)
        scf = link_manager.get(drone_id)
        param_originals = apply_param_overrides(
            scf,
            list(getattr(args, "set_param", None) or []),
        )

        print("Reset estimator...")
        transport.reset_estimator_and_wait(drone_id)

        pose_source.start()
        initial_pos = _latest_fresh_position(
            pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
        )
        if initial_pos is None:
            raise RuntimeError("No fresh pose received before takeoff")
        print(f"Initial pose: {initial_pos.round(3).tolist()}")

        print(f"PID takeoff to {args.height:.2f}m...")
        transport.set_onboard_controller(drone_id, "pid")
        transport.hl_takeoff(drone_id, args.height, args.takeoff_duration)
        time.sleep(args.takeoff_duration + args.settle_s)

        start_pos = _latest_fresh_position(
            pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
        )
        if start_pos is None:
            raise RuntimeError("No fresh pose after takeoff")
        if start_pos[2] < args.min_takeoff_z:
            raise RuntimeError(
                f"Takeoff validation failed: z={start_pos[2]:.2f} < {args.min_takeoff_z:.2f}"
            )

        center = start_pos - np.array([radius, 0.0, 0.0], dtype=float)
        print(f"Circle start from measured pose: {start_pos.round(3).tolist()}")
        print(f"Circle center: {center.round(3).tolist()}")
        print("Switching to Mellinger and streaming full-state circle...")
        _switch_to_full_state_mode(
            link_manager=link_manager,
            transport=transport,
            drone_id=drone_id,
            controller=args.controller,
            notify_before_full_state=bool(
                getattr(args, "notify_before_full_state", False)
            ),
        )
        _stream_hold(
            transport=transport,
            drone_id=drone_id,
            target=start_pos,
            duration_s=float(args.warmup_s),
            rate_hz=rate_hz,
        )
        z_bias = 0.0
        if getattr(args, "z_bias_compensation", False):
            current = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
            )
            if current is None:
                raise RuntimeError("No fresh pose after full-state warmup")
            z_bias = float(current[2] - start_pos[2])
            limited_bias = float(np.clip(z_bias, -max_z_bias, max_z_bias))
            print(
                "Z bias compensation enabled: "
                f"measured={z_bias:+.3f}m limited={limited_bias:+.3f}m"
            )

        duration_s = period_s * cycles
        period = 1.0 / rate_hz
        start_t = time.monotonic()
        deadline = start_t + duration_s
        next_report = 0.0
        max_error = 0.0
        ticks = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            t_s = now - start_t
            pos, vel, acc = circle_setpoint(
                center,
                radius=radius,
                period_s=period_s,
                t_s=t_s,
            )
            command_pos = (
                apply_z_bias_compensation(
                    pos,
                    z_bias=z_bias,
                    max_abs_bias=max_z_bias,
                )
                if getattr(args, "z_bias_compensation", False)
                else pos
            )
            transport.cmd_full_state(
                drone_id,
                _tuple3(command_pos),
                _tuple3(vel),
                _tuple3(acc),
            )
            ticks += 1

            current = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=min(0.2, period)
            )
            if current is not None:
                components = tracking_error_components(current, pos)
                error = components["err"]
                max_error = max(max_error, error)
                if now >= next_report:
                    print(
                        "target="
                        f"{pos.round(3).tolist()} "
                        "pose="
                        f"{current.round(3).tolist()} "
                        f"err={error:.3f}m "
                        f"err_xy={components['err_xy']:.3f}m "
                        f"err_z={components['err_z']:+.3f}m "
                        f"max_err={max_error:.3f}m"
                        f"{format_diagnostic_values(latest_health.get(drone_id))}"
                    )
                    next_report = now + 0.5
                if error > args.abort_radius:
                    raise RuntimeError(
                        f"Probe abort: tracking error {error:.2f}m > {args.abort_radius:.2f}m"
                    )
            time.sleep(period)

        print(f"Circle probe finished. ticks={ticks} max_err={max_error:.3f}m")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        logger.exception("Full-state circle probe failed")
        print(f"Probe failed: {exc}")
        return 1
    finally:
        if scf is not None and param_originals:
            try:
                restore_param_overrides(scf, param_originals)
            except Exception:
                logger.exception("Failed to restore temporary parameter overrides")
        try:
            transport.notify_setpoint_stop(drone_id)
        except Exception:
            pass
        try:
            transport.set_onboard_controller(drone_id, "pid")
        except Exception:
            pass
        try:
            transport.hl_land(drone_id, 0.0, args.land_duration)
            time.sleep(args.land_duration + 0.5)
        except Exception:
            pass
        try:
            pose_source.stop()
        except Exception:
            pass
        link_manager.close_all()
