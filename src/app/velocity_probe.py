"""Single-drone world-velocity direction probe.

This bypasses the AFC mission and connects one Crazyflie. It takes off with the
high-level commander, optionally notifies setpoint stop, then stops HLC and
streams small world-frame velocity setpoints along +x, -x, +y, -y. The printed
displacement per segment is meant to catch URI/pose mapping issues and
world-velocity direction problems.
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np

from ..adapters.cflib_command_transport import CflibCommandTransport
from ..adapters.cflib_link_manager import CflibLinkManager
from ..adapters.lighthouse_pose_source import LighthousePoseSource
from ..config.loader import ConfigLoader
from ..config.schema import FleetConfig
from ..domain.fleet_model import FleetModel
from ..runtime.pose_bus import PoseBus

logger = logging.getLogger(__name__)


def _build_single_fleet(config, drone_id: int) -> FleetModel:
    selected = [drone for drone in config.fleet.drones if drone.id == drone_id]
    if not selected:
        raise ValueError(f"Drone {drone_id} not found in fleet config")
    return FleetModel(FleetConfig(drones=selected))


def _latest_fresh_position(
    pose_bus: PoseBus,
    fleet: FleetModel,
    drone_id: int,
    *,
    timeout_s: float,
) -> np.ndarray | None:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    idx = fleet.id_to_index(drone_id)
    while time.monotonic() < deadline:
        snapshot = pose_bus.latest()
        if snapshot is not None and snapshot.fresh_mask[idx]:
            return np.array(snapshot.positions[idx], dtype=float)
        time.sleep(0.05)
    return None


def _stream_velocity(
    transport: CflibCommandTransport,
    drone_id: int,
    velocity: np.ndarray,
    *,
    duration_s: float,
    rate_hz: float,
) -> int:
    period_s = 1.0 / max(float(rate_hz), 1e-6)
    deadline = time.monotonic() + max(0.0, float(duration_s))
    ticks = 0
    while time.monotonic() < deadline:
        transport.cmd_velocity_world(
            drone_id,
            float(velocity[0]),
            float(velocity[1]),
            float(velocity[2]),
        )
        ticks += 1
        time.sleep(period_s)
    return ticks


def _segment_result(
    *,
    label: str,
    command: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
    duration_s: float,
    ticks: int,
) -> dict:
    displacement = end - start
    command_norm = float(np.linalg.norm(command))
    direction = command / command_norm if command_norm > 1e-9 else np.zeros(3)
    projected = float(np.dot(displacement, direction))
    lateral = float(np.linalg.norm(displacement - projected * direction))
    speed_along = projected / max(float(duration_s), 1e-9)
    return {
        "label": label,
        "command": command.round(6).tolist(),
        "start": start.round(6).tolist(),
        "end": end.round(6).tolist(),
        "displacement": displacement.round(6).tolist(),
        "projected_displacement_m": projected,
        "lateral_displacement_m": lateral,
        "speed_along_mps": speed_along,
        "ticks": ticks,
    }


def run(args: argparse.Namespace) -> int:
    config = ConfigLoader.load(args.config_dir)
    drone_id = int(args.drone_id)
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
        attitude_log_enabled=False,
        motor_log_enabled=False,
    )
    pose_source.register_callback(
        lambda did, pos, timestamp, velocity=None: pose_bus.update_agent(
            did, pos, timestamp, velocity
        )
    )

    speed = float(args.speed)
    segments = [
        ("+x", np.array([speed, 0.0, 0.0], dtype=float)),
        ("-x", np.array([-speed, 0.0, 0.0], dtype=float)),
        ("+y", np.array([0.0, speed, 0.0], dtype=float)),
        ("-y", np.array([0.0, -speed, 0.0], dtype=float)),
    ]
    results: list[dict] = []

    print(f"=== Velocity-world probe: drone {drone_id} ===")
    print(
        f"speed={speed:.2f}m/s segment={args.segment_s:.2f}s "
        f"rate={args.rate_hz:.1f}Hz height={args.height:.2f}m"
    )

    try:
        link_manager.connect_all(parallel_groups=False)
        transport.wait_for_params(drone_id)

        print("Reset estimator...")
        transport.reset_estimator_and_wait(drone_id)

        pose_source.start()
        initial_pos = _latest_fresh_position(
            pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
        )
        if initial_pos is None:
            raise RuntimeError("No fresh pose received before takeoff")
        print(f"Initial pose: {initial_pos.round(3).tolist()}")

        transport.set_onboard_controller(drone_id, "pid")
        print(f"Takeoff to {args.height:.2f}m...")
        transport.hl_takeoff(drone_id, args.height, args.takeoff_duration)
        time.sleep(args.takeoff_duration + args.settle_s)

        takeoff_pos = _latest_fresh_position(
            pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
        )
        if takeoff_pos is None:
            raise RuntimeError("No fresh pose after takeoff")
        if takeoff_pos[2] < args.min_takeoff_z:
            raise RuntimeError(
                f"Takeoff validation failed: z={takeoff_pos[2]:.2f} < {args.min_takeoff_z:.2f}"
            )
        print(f"Takeoff pose: {takeoff_pos.round(3).tolist()}")

        notify_before_velocity = bool(getattr(args, "notify_before_velocity", True))
        if notify_before_velocity:
            print("Notify setpoint stop before velocity streaming...")
            transport.notify_setpoint_stop(drone_id)
            time.sleep(0.1)

        print("Stopping high-level commander before velocity streaming...")
        link_manager.get(drone_id).cf.high_level_commander.stop()
        time.sleep(0.1)

        for label, command in segments:
            start = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
            )
            if start is None:
                raise RuntimeError(f"No fresh pose before segment {label}")

            print(f"Segment {label}: command={command.tolist()}")
            ticks = _stream_velocity(
                transport,
                drone_id,
                command,
                duration_s=args.segment_s,
                rate_hz=args.rate_hz,
            )
            _stream_velocity(
                transport,
                drone_id,
                np.zeros(3, dtype=float),
                duration_s=args.rest_s,
                rate_hz=args.rate_hz,
            )

            end = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
            )
            if end is None:
                raise RuntimeError(f"No fresh pose after segment {label}")

            result = _segment_result(
                label=label,
                command=command,
                start=start,
                end=end,
                duration_s=args.segment_s,
                ticks=ticks,
            )
            results.append(result)
            print(
                f"  start={np.array(result['start']).round(3).tolist()} "
                f"end={np.array(result['end']).round(3).tolist()} "
                f"disp={np.array(result['displacement']).round(3).tolist()} "
                f"along={result['projected_displacement_m']:.3f}m "
                f"lateral={result['lateral_displacement_m']:.3f}m "
                f"speed_along={result['speed_along_mps']:.3f}m/s"
            )

        if args.output:
            output_path = Path(args.output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                json.dumps(
                    {
                        "drone_id": drone_id,
                        "speed": speed,
                        "segment_s": float(args.segment_s),
                        "rate_hz": float(args.rate_hz),
                        "notify_before_velocity": notify_before_velocity,
                        "results": results,
                    },
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            print(f"Saved probe result: {output_path}")

        print("Velocity-world probe finished.")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        logger.exception("Velocity-world probe failed")
        print(f"Probe failed: {exc}")
        return 1
    finally:
        try:
            _stream_velocity(
                transport,
                drone_id,
                np.zeros(3, dtype=float),
                duration_s=0.3,
                rate_hz=args.rate_hz,
            )
        except Exception:
            pass
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
