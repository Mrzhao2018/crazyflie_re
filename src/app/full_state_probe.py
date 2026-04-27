"""Single-drone full-state/Mellinger probe.

This command intentionally bypasses the AFC mission, leader trajectory upload,
and swarm follower scheduling. It connects one Crazyflie, takes it off with
the high-level commander under PID, then switches to Mellinger and streams a
fixed full-state position target at the measured takeoff position.
"""

from __future__ import annotations

import argparse
import logging
import time

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

    print(f"=== Full-state probe: drone {drone_id} ===")
    print("Only this drone is connected and commanded.")

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

        print(f"PID takeoff to {args.height:.2f}m...")
        transport.set_onboard_controller(drone_id, "pid")
        transport.hl_takeoff(drone_id, args.height, args.takeoff_duration)
        time.sleep(args.takeoff_duration + args.settle_s)

        hold_pos = _latest_fresh_position(
            pose_bus, fleet, drone_id, timeout_s=args.pose_timeout
        )
        if hold_pos is None:
            raise RuntimeError("No fresh pose after takeoff")
        if hold_pos[2] < args.min_takeoff_z:
            raise RuntimeError(
                f"Takeoff validation failed: z={hold_pos[2]:.2f} < {args.min_takeoff_z:.2f}"
            )

        print(f"Hold target from measured pose: {hold_pos.round(3).tolist()}")
        print("Switching to Mellinger and streaming full-state position-only hold...")
        scf = link_manager.get(drone_id)
        scf.cf.high_level_commander.stop()
        transport.set_onboard_controller(drone_id, args.controller)

        period = 1.0 / args.rate_hz
        deadline = time.monotonic() + args.hold_s
        next_report = 0.0
        max_error = 0.0
        while time.monotonic() < deadline:
            transport.cmd_full_state(
                drone_id,
                tuple(float(v) for v in hold_pos),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            current = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=min(0.2, period)
            )
            now = time.monotonic()
            if current is not None:
                error = float(np.linalg.norm(current - hold_pos))
                max_error = max(max_error, error)
                if now >= next_report:
                    print(
                        "pose="
                        f"{current.round(3).tolist()} "
                        f"err={error:.3f}m max_err={max_error:.3f}m"
                    )
                    next_report = now + 0.5
                if error > args.abort_radius:
                    raise RuntimeError(
                        f"Probe abort: position error {error:.2f}m > {args.abort_radius:.2f}m"
                    )
            time.sleep(period)

        print(f"Probe hold finished. max_err={max_error:.3f}m")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        logger.exception("Full-state probe failed")
        print(f"Probe failed: {exc}")
        return 1
    finally:
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

