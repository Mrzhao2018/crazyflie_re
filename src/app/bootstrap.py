"""依赖注入和对象组装"""

from pathlib import Path

import numpy as np

from ..config.loader import ConfigLoader
from ..domain.fleet_model import FleetModel
from ..domain.formation_model import FormationModel
from ..domain.stress_matrix_solver import StressMatrixSolver
from ..domain.afc_model import AFCModel
from ..domain.mission_profile import MissionProfile
from ..domain.leader_reference import LeaderReferenceGenerator
from ..domain.follower_reference import FollowerReferenceGenerator
from ..runtime.pose_bus import PoseBus
from ..runtime.affine_frame_estimator import AffineFrameEstimator
from ..runtime.follower_controller import FollowerController
from ..runtime.follower_controller_v2 import FollowerControllerV2
from ..runtime.mission_fsm import MissionFSM
from ..runtime.safety_manager import SafetyManager
from ..runtime.scheduler import CommandScheduler
from ..runtime.telemetry import TelemetryRecorder
from ..runtime.health_bus import HealthBus
from ..runtime.link_quality_bus import LinkQualityBus
from ..runtime.link_state_bus import LinkStateBus
from ..runtime.manual_leader_state import ManualLeaderState
from ..runtime.manual_leader_reference import ManualLeaderReferenceSource
from .preflight import PreflightRunner
from ..adapters.cflib_link_manager import CflibLinkManager
from ..adapters.cflib_command_transport import CflibCommandTransport
from ..adapters.group_executor_pool import GroupExecutorPool
from ..adapters.leader_executor import LeaderExecutor
from ..adapters.follower_executor import FollowerExecutor
from ..adapters.lighthouse_pose_source import LighthousePoseSource
from ..adapters.manual_input_keyboard import KeyboardManualInputSource
from ..adapters.cflib_console_tap import ConsoleTap


def _build_link_quality_provider(fleet, link_quality_bus):
    """Return ``callable(group_id) -> float|None`` giving worst-case link_quality
    per radio_group, or ``None`` when the bus is not wired.
    """

    if link_quality_bus is None:
        return None

    def provider(group_id: int) -> float | None:
        latest = link_quality_bus.latest()
        if not latest:
            return None
        worst: float | None = None
        for drone_id, sample in latest.items():
            if fleet.get_radio_group(drone_id) != group_id:
                continue
            quality = sample.link_quality
            if quality is None:
                continue
            if worst is None or quality < worst:
                worst = float(quality)
        return worst

    return provider


def _build_follower_controller(control_config):
    """Select the host-side follower controller.

    ``full_state`` requires the V2 controller path because it is the one that
    emits ``target_positions`` / ``target_accelerations`` for downstream
    scheduler/executor full-state dispatch.
    """

    if control_config.output_mode == "full_state":
        return FollowerControllerV2(control_config)
    if control_config.dynamics_model_order == 2:
        return FollowerControllerV2(control_config)
    return FollowerController(control_config)


def build_core_app(config_dir: str, startup_mode_override: str | None = None):
    """构建与真机无关的核心应用对象"""
    resolved_config_dir = str(Path(config_dir).resolve())
    repo_root = str(Path(resolved_config_dir).parent)

    # 1. 配置
    config = ConfigLoader.load(resolved_config_dir, startup_mode_override=startup_mode_override)

    # 2. Domain层
    fleet = FleetModel(config.fleet)
    nominal = np.array(config.mission.nominal_positions)
    formation = FormationModel(nominal, fleet.leader_ids(), fleet)

    solver = StressMatrixSolver(formation)
    stress_result = solver.solve_dense(fleet.leader_ids())
    afc = AFCModel(stress_result, fleet)

    mission_profile = MissionProfile(config.mission)
    auto_leader_ref_gen = LeaderReferenceGenerator(mission_profile, formation, fleet)
    follower_ref_gen = FollowerReferenceGenerator(
        formation,
        afc,
        config.safety.max_condition_number,
        time_delay_compensation_enabled=config.control.time_delay_compensation_enabled,
        estimated_total_delay_ms=config.control.estimated_total_delay_ms,
        delay_prediction_gain=config.control.delay_prediction_gain,
    )

    # 3. Runtime层
    pose_bus = PoseBus(fleet, config.safety.pose_timeout)
    frame_estimator = AffineFrameEstimator(fleet)
    follower_controller = _build_follower_controller(config.control)
    fsm = MissionFSM()
    link_state_bus = LinkStateBus()
    safety = SafetyManager(config.safety, fleet, link_state_bus=link_state_bus)
    telemetry = TelemetryRecorder(
        flush_every_n=config.comm.telemetry_flush_every_n,
        queue_max_size=config.comm.telemetry_queue_max,
    )
    health_bus = HealthBus()
    link_quality_bus = LinkQualityBus() if config.comm.link_quality_enabled else None
    link_quality_provider = _build_link_quality_provider(fleet, link_quality_bus)
    scheduler = CommandScheduler(
        config.comm,
        fsm,
        fleet,
        link_quality_provider=link_quality_provider,
    )

    startup_mode = config.startup.mode
    manual_state = None
    manual_input = None
    leader_ref_gen = auto_leader_ref_gen
    if startup_mode == "manual_leader":
        if config.startup.manual is None:
            raise ValueError("manual_leader 模式缺少 startup.manual 配置")
        manual_state = ManualLeaderState(
            default_axis=config.startup.manual.default_axis,
            min_scale=config.startup.manual.min_scale,
            max_scale=config.startup.manual.max_scale,
        )
        leader_ref_gen = ManualLeaderReferenceSource(formation, fleet, manual_state)

    components = {
        "config": config,
        "config_dir": resolved_config_dir,
        "repo_root": repo_root,
        "startup_mode": startup_mode,
        "fleet": fleet,
        "formation": formation,
        "afc": afc,
        "mission_profile": mission_profile,
        "auto_leader_ref_gen": auto_leader_ref_gen,
        "leader_ref_gen": leader_ref_gen,
        "follower_ref_gen": follower_ref_gen,
        "pose_bus": pose_bus,
        "frame_estimator": frame_estimator,
        "follower_controller": follower_controller,
        "fsm": fsm,
        "safety": safety,
        "scheduler": scheduler,
        "telemetry": telemetry,
        "health_bus": health_bus,
        "link_quality_bus": link_quality_bus,
        "link_state_bus": link_state_bus,
        "manual_leader_state": manual_state,
        "manual_input": manual_input,
    }

    components["preflight"] = PreflightRunner(components)
    return components


def build_real_app(config_dir: str, startup_mode_override: str | None = None):
    """构建包含真机适配层的完整应用对象"""
    components = build_core_app(config_dir, startup_mode_override=startup_mode_override)

    link_manager = CflibLinkManager(
        components["fleet"],
        link_quality_bus=components.get("link_quality_bus"),
        link_state_bus=components.get("link_state_bus"),
        connect_pace_s=components["config"].comm.connect_pace_s,
        connect_timeout_s=components["config"].comm.connect_timeout_s,
        radio_driver=components["config"].comm.radio_driver,
    )
    transport = CflibCommandTransport(link_manager)
    radio_groups = sorted(
        {
            components["fleet"].get_radio_group(drone_id)
            for drone_id in components["fleet"].all_ids()
        }
    )
    group_executor_pool = GroupExecutorPool(group_ids=radio_groups)
    leader_executor = LeaderExecutor(
        transport, group_executor_pool=group_executor_pool
    )
    follower_executor = FollowerExecutor(
        transport, group_executor_pool=group_executor_pool
    )
    pose_source = LighthousePoseSource(
        link_manager,
        components["fleet"],
        components["config"].comm.pose_log_freq,
        attitude_log_enabled=components["config"].comm.attitude_log_enabled,
        motor_log_enabled=components["config"].comm.motor_log_enabled,
    )

    components.update(
        {
            "link_manager": link_manager,
            "transport": transport,
            "group_executor_pool": group_executor_pool,
            "leader_executor": leader_executor,
            "follower_executor": follower_executor,
            "pose_source": pose_source,
            "console_tap": ConsoleTap(link_manager),
        }
    )

    if components["startup_mode"] == "manual_leader":
        manual_cfg = components["config"].startup.manual
        if manual_cfg is None:
            raise ValueError("manual_leader 模式缺少 startup.manual 配置")
        components["manual_input"] = KeyboardManualInputSource(
            translation_step=manual_cfg.translation_step,
            vertical_step=manual_cfg.vertical_step,
            scale_step=manual_cfg.scale_step,
            rotation_step_deg=manual_cfg.rotation_step_deg,
        )

    pose_source.register_health_callback(
        lambda drone_id, health, timestamp: components["health_bus"].update(
            drone_id, health, timestamp
        )
    )

    return components


def build_app(config_dir: str, startup_mode_override: str | None = None):
    """向后兼容：默认构建完整真机应用对象"""
    return build_real_app(config_dir, startup_mode_override=startup_mode_override)
