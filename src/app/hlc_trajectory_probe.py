"""Single-drone HLC trajectory probe.

This runs the same uploaded trajectory path used by a mission leader, but only
connects one Crazyflie. It is intended to separate onboard HLC trajectory
tracking from AFC/follower/radio-group effects.
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
from ..domain.fleet_model import FleetModel
from ..domain.formation_model import FormationModel
from ..domain.mission_profile import MissionProfile
from ..runtime.offline_swarm_sampler import _evaluate_trajectory_spec
from ..runtime.pose_bus import PoseBus
from .full_state_probe import _build_single_fleet, _latest_fresh_position

logger = logging.getLogger(__name__)


def tracking_error_components(current: np.ndarray, target: np.ndarray) -> dict[str, float]:
    delta = np.asarray(current, dtype=float) - np.asarray(target, dtype=float)
    return {
        "err": float(np.linalg.norm(delta)),
        "err_xy": float(np.linalg.norm(delta[:2])),
        "err_z": float(delta[2]),
    }


def estimate_phase_lag(
    current: np.ndarray,
    *,
    expected_elapsed_s: float,
    reference_by_time: dict[float, np.ndarray],
) -> tuple[float, float]:
    best_t = None
    best_distance = None
    for t_s, target in reference_by_time.items():
        distance = float(np.linalg.norm(np.asarray(current, dtype=float) - target))
        if best_distance is None or distance < best_distance:
            best_t = float(t_s)
            best_distance = distance
    if best_t is None or best_distance is None:
        return 0.0, float("nan")
    return float(expected_elapsed_s) - best_t, best_distance


def _build_trajectory_spec(config, drone_id: int) -> dict:
    full_fleet = FleetModel(config.fleet)
    formation = FormationModel(
        np.array(config.mission.nominal_positions, dtype=float),
        full_fleet.leader_ids(),
        full_fleet,
    )
    mission_profile = MissionProfile(config.mission)
    return mission_profile.trajectory_spec_for_nominal(
        formation.nominal_position(drone_id)
    )


def _reference_grid(spec: dict, total_s: float, dt_s: float) -> dict[float, np.ndarray]:
    times = np.arange(0.0, max(float(total_s), 0.0) + 1e-9, max(float(dt_s), 1e-3))
    return {float(t): _evaluate_trajectory_spec(spec, float(t)) for t in times}


def run(args: argparse.Namespace) -> int:
    config = ConfigLoader.load(args.config_dir)
    drone_id = int(args.drone_id)
    fleet = _build_single_fleet(config, drone_id)
    spec = _build_trajectory_spec(config, drone_id)
    pieces = spec["pieces"]
    trajectory_id = int(spec.get("trajectory_id", 1))
    start_addr = int(spec.get("start_addr", 0))
    trajectory_type = str(spec.get("trajectory_type", "poly4d"))
    total_s = sum(float(piece.duration) for piece in pieces)

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

    print(f"=== HLC trajectory probe: drone {drone_id} ===")
    print(
        f"trajectory_id={trajectory_id} type={trajectory_type} "
        f"pieces={len(pieces)} total_s={total_s:.2f}"
    )
    results = []

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

        if getattr(args, "set_param", None):
            for assignment in args.set_param:
                name, value = assignment.split("=", 1)
                transport.set_param(drone_id, name, value)
                print(f"Set param override: {name}={value}")

        print(f"Set controller: {args.controller}")
        transport.set_onboard_controller(drone_id, args.controller)

        print(f"Takeoff to {args.height:.2f}m...")
        transport.hl_takeoff(drone_id, args.height, args.takeoff_duration)
        time.sleep(args.takeoff_duration + args.settle_s)

        start_pos = _evaluate_trajectory_spec(spec, 0.0)
        print(f"Go to trajectory start: {start_pos.round(3).tolist()}")
        transport.hl_go_to(
            drone_id,
            float(start_pos[0]),
            float(start_pos[1]),
            float(start_pos[2]),
            args.entry_duration,
        )
        time.sleep(args.entry_duration + args.start_stabilize_s)

        print("Upload and define trajectory...")
        piece_count = transport.upload_trajectory(
            drone_id,
            pieces,
            start_addr=start_addr,
            trajectory_type=trajectory_type,
        )
        transport.hl_define_trajectory(
            drone_id,
            trajectory_id,
            start_addr,
            piece_count,
            trajectory_type=trajectory_type,
        )

        reference = _reference_grid(spec, total_s, args.phase_grid_dt)
        print("Start HLC trajectory...")
        started = time.monotonic()
        transport.hl_start_trajectory(
            drone_id,
            trajectory_id,
            time_scale=1.0,
            relative_position=False,
            relative_yaw=False,
            reversed=False,
        )
        deadline = started + min(total_s, float(args.max_run_s))
        next_report = started
        max_err = 0.0
        max_lag = 0.0
        while time.monotonic() < deadline:
            elapsed = time.monotonic() - started
            target = _evaluate_trajectory_spec(spec, elapsed)
            current = _latest_fresh_position(
                pose_bus, fleet, drone_id, timeout_s=min(0.2, 1.0 / args.report_hz)
            )
            if current is not None:
                error = tracking_error_components(current, target)
                lag_s, phase_fit_error = estimate_phase_lag(
                    current,
                    expected_elapsed_s=elapsed,
                    reference_by_time=reference,
                )
                max_err = max(max_err, error["err"])
                max_lag = max(max_lag, abs(lag_s))
                sample = {
                    "elapsed_s": elapsed,
                    "target": target.tolist(),
                    "pose": current.tolist(),
                    "lag_s": lag_s,
                    "phase_fit_error_m": phase_fit_error,
                    **error,
                }
                results.append(sample)
                now = time.monotonic()
                if now >= next_report:
                    print(
                        f"t={elapsed:.2f}s target={target.round(3).tolist()} "
                        f"pose={current.round(3).tolist()} "
                        f"err={error['err']:.3f}m lag={lag_s:.2f}s "
                        f"fit={phase_fit_error:.3f}m"
                    )
                    next_report = now + 1.0 / max(float(args.report_hz), 1e-6)
                if error["err"] > args.abort_radius:
                    raise RuntimeError(
                        f"Probe abort: trajectory error {error['err']:.2f}m > {args.abort_radius:.2f}m"
                    )
            time.sleep(0.02)

        print(f"HLC trajectory probe finished. max_err={max_err:.3f}m max_abs_lag={max_lag:.2f}s")
        if args.output:
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(
                    {
                        "drone_id": drone_id,
                        "trajectory_id": trajectory_id,
                        "trajectory_type": trajectory_type,
                        "piece_count": len(pieces),
                        "total_s": total_s,
                        "controller": args.controller,
                        "samples": results,
                    },
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            print(f"Saved probe result: {output}")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        logger.exception("HLC trajectory probe failed")
        print(f"Probe failed: {exc}")
        return 1
    finally:
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
