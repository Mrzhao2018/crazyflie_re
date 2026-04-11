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
from ..runtime.mission_fsm import MissionFSM
from ..runtime.safety_manager import SafetyManager
from ..runtime.scheduler import CommandScheduler
from ..runtime.telemetry import TelemetryRecorder
from ..runtime.health_bus import HealthBus
from ..runtime.manual_leader_state import ManualLeaderState
from ..runtime.manual_leader_reference import ManualLeaderReferenceSource
from .preflight import PreflightRunner
from ..adapters.cflib_link_manager import CflibLinkManager
from ..adapters.cflib_command_transport import CflibCommandTransport
from ..adapters.leader_executor import LeaderExecutor
from ..adapters.follower_executor import FollowerExecutor
from ..adapters.lighthouse_pose_source import LighthousePoseSource
from ..adapters.manual_input_keyboard import KeyboardManualInputSource


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
        formation, afc, config.safety.max_condition_number
    )

    # 3. Runtime层
    pose_bus = PoseBus(fleet, config.safety.pose_timeout)
    frame_estimator = AffineFrameEstimator(fleet)
    follower_controller = FollowerController(config.control)
    fsm = MissionFSM()
    safety = SafetyManager(config.safety, fleet)
    scheduler = CommandScheduler(config.comm, fsm, fleet)
    telemetry = TelemetryRecorder()
    health_bus = HealthBus()

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
        "manual_leader_state": manual_state,
        "manual_input": manual_input,
    }

    components["preflight"] = PreflightRunner(components)
    return components


def build_real_app(config_dir: str, startup_mode_override: str | None = None):
    """构建包含真机适配层的完整应用对象"""
    components = build_core_app(config_dir, startup_mode_override=startup_mode_override)

    link_manager = CflibLinkManager(components["fleet"])
    transport = CflibCommandTransport(link_manager)
    leader_executor = LeaderExecutor(transport)
    follower_executor = FollowerExecutor(transport)
    pose_source = LighthousePoseSource(
        link_manager, components["fleet"], components["config"].comm.pose_log_freq
    )

    components.update(
        {
            "link_manager": link_manager,
            "transport": transport,
            "leader_executor": leader_executor,
            "follower_executor": follower_executor,
            "pose_source": pose_source,
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
