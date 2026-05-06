"""Crazyflie firmware parameter inspection helper."""

from __future__ import annotations

import argparse

from ..adapters.cflib_link_manager import CflibLinkManager
from ..config.loader import ConfigLoader
from .full_state_probe import _build_single_fleet


DEFAULT_FILTERS = [
    "stabilizer",
    "ctrl",
    "mel",
    "mellinger",
    "posctl",
    "velctl",
    "kalman",
    "lighthouse",
    "locsrv",
]


def _iter_param_names(toc) -> list[str]:
    names: list[str] = []
    for group, params in getattr(toc, "items", lambda: [])():
        if hasattr(params, "keys"):
            for name in params.keys():
                names.append(f"{group}.{name}")
    return names


def param_matches_filters(name: str, filters: list[str] | None) -> bool:
    if not filters:
        return True
    lowered = name.lower()
    return any(str(item).lower() in lowered for item in filters)


def select_param_names(toc, filters: list[str] | None) -> list[str]:
    return sorted(
        name
        for name in _iter_param_names(toc)
        if param_matches_filters(name, filters)
    )


def format_diagnostic_values(values: dict | None) -> str:
    values = values or {}
    parts: list[str] = []
    if "stabilizer.thrust" in values:
        parts.append(f"thrust={float(values['stabilizer.thrust']):.3f}")
    if "pm.vbat" in values:
        parts.append(f"vbat={float(values['pm.vbat']):.3f}")
    motor_values = []
    for key in ("motor.m1", "motor.m2", "motor.m3", "motor.m4"):
        if key in values:
            motor_values.append(f"{float(values[key]):.3f}")
    if motor_values:
        parts.append(f"motors=[{','.join(motor_values)}]")
    return f" {' '.join(parts)}" if parts else ""


def _param_toc(scf):
    return getattr(getattr(scf.cf.param, "toc", None), "toc", {})


def run(args: argparse.Namespace) -> int:
    config = ConfigLoader.load(args.config_dir)
    drone_id = int(args.drone_id)
    filters = list(args.filter or DEFAULT_FILTERS)
    fleet = _build_single_fleet(config, drone_id)
    link_manager = CflibLinkManager(
        fleet,
        connect_pace_s=config.comm.connect_pace_s,
        connect_timeout_s=config.comm.connect_timeout_s,
        radio_driver=config.comm.radio_driver,
    )

    print(f"=== Crazyflie parameter probe: drone {drone_id} ===")
    print(f"filters={filters}")
    try:
        link_manager.connect_all(parallel_groups=False)
        scf = link_manager.get(drone_id)
        scf.wait_for_params()
        names = select_param_names(_param_toc(scf), filters)
        if not names:
            print("No matching parameters found.")
            return 0
        for name in names:
            try:
                value = scf.cf.param.get_value(name)
                print(f"{name} = {value}")
            except Exception as exc:
                print(f"{name} = <ERR {exc}>")
        return 0
    finally:
        link_manager.close_all()
