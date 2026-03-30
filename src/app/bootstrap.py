"""依赖注入和对象组装"""

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
from .preflight import PreflightRunner
from ..adapters.cflib_link_manager import CflibLinkManager
from ..adapters.cflib_command_transport import CflibCommandTransport
from ..adapters.leader_executor import LeaderExecutor
from ..adapters.follower_executor import FollowerExecutor
from ..adapters.lighthouse_pose_source import LighthousePoseSource


def build_core_app(config_dir: str):
    """构建与真机无关的核心应用对象"""
    # 1. 配置
    config = ConfigLoader.load(config_dir)

    # 2. Domain层
    fleet = FleetModel(config.fleet)
    nominal = np.array(config.mission.nominal_positions)
    formation = FormationModel(nominal, fleet.leader_ids(), fleet)

    solver = StressMatrixSolver(formation)
    stress_result = solver.solve_dense(fleet.leader_ids())
    afc = AFCModel(stress_result, fleet)

    mission_profile = MissionProfile(config.mission)
    leader_ref_gen = LeaderReferenceGenerator(mission_profile, formation, fleet)
    follower_ref_gen = FollowerReferenceGenerator(
        formation, afc, config.safety.max_condition_number
    )

    # 3. Runtime层
    pose_bus = PoseBus(fleet, config.safety.pose_timeout)
    frame_estimator = AffineFrameEstimator(fleet)
    follower_controller = FollowerController(config.control)
    fsm = MissionFSM()
    safety = SafetyManager(config.safety, fleet)
    scheduler = CommandScheduler(config.comm, fsm)
    telemetry = TelemetryRecorder()
    health_bus = HealthBus()

    components = {
        "config": config,
        "fleet": fleet,
        "formation": formation,
        "afc": afc,
        "mission_profile": mission_profile,
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
    }

    components["preflight"] = PreflightRunner(components)
    return components


def build_real_app(config_dir: str):
    """构建包含真机适配层的完整应用对象"""
    components = build_core_app(config_dir)

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

    pose_source.register_health_callback(
        lambda drone_id, health, timestamp: components["health_bus"].update(
            drone_id, health, timestamp
        )
    )

    return components


def build_app(config_dir: str):
    """向后兼容：默认构建完整真机应用对象"""
    return build_real_app(config_dir)
