"""配置加载器"""

import yaml
from pathlib import Path
from .schema import *


class ConfigLoader:
    """从YAML文件加载配置"""

    @staticmethod
    def _load_yaml_file(path: Path) -> dict:
        if not path.exists():
            raise FileNotFoundError(f"配置文件不存在: {path}")

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError(f"配置文件为空或格式错误: {path}")

        return data

    @staticmethod
    def _validate(config: AppConfig):
        drone_count = len(config.fleet.drones)
        if drone_count == 0:
            raise ValueError("fleet.yaml 中 drones 不能为空")

        drone_ids = [drone.id for drone in config.fleet.drones]
        if len(set(drone_ids)) != len(drone_ids):
            raise ValueError("drone id 必须唯一")

        leader_count = sum(1 for drone in config.fleet.drones if drone.role == "leader")
        if leader_count < 4:
            raise ValueError("3D affine formation 至少需要 4 个 leaders")

        if len(config.mission.nominal_positions) != drone_count:
            raise ValueError("mission.nominal_positions 数量必须与 drones 数量一致")

        if not config.mission.phases:
            raise ValueError("mission.phases 不能为空")

        previous_end = None
        for phase in config.mission.phases:
            if phase.t_start >= phase.t_end:
                raise ValueError(f"阶段 {phase.name} 必须满足 t_start < t_end")
            if previous_end is not None and phase.t_start < previous_end:
                raise ValueError("mission.phases 不能重叠且必须按时间排序")
            previous_end = phase.t_end

        if (
            config.mission.leader_motion.translation is not None
            and len(config.mission.leader_motion.translation) != 3
        ):
            raise ValueError("leader_motion.translation 必须是 3 维向量")

        if config.mission.leader_motion.trajectory_start_addr < 0:
            raise ValueError("trajectory_start_addr 不能为负数")

        if config.mission.leader_motion.trajectory_sample_dt <= 0:
            raise ValueError("trajectory_sample_dt 必须大于 0")

        if config.mission.leader_motion.trajectory_pieces is None:
            raise ValueError("trajectory_pieces 不能为空，请至少提供 []")

        if config.safety.min_vbat <= 0:
            raise ValueError("min_vbat 必须大于 0")

        for freq_name in (
            "pose_log_freq",
            "follower_tx_freq",
            "leader_update_freq",
            "parked_hold_freq",
        ):
            if getattr(config.comm, freq_name) <= 0:
                raise ValueError(f"{freq_name} 必须大于 0")

        if len(config.safety.boundary_min) != 3 or len(config.safety.boundary_max) != 3:
            raise ValueError("boundary_min 和 boundary_max 必须都是 3 维向量")

        if any(
            lo >= hi
            for lo, hi in zip(config.safety.boundary_min, config.safety.boundary_max)
        ):
            raise ValueError("safety 边界必须满足 boundary_min < boundary_max")

    @staticmethod
    def load(config_dir: str) -> AppConfig:
        """加载所有配置文件"""
        config_path = Path(config_dir)

        # 加载各个配置文件
        fleet_data = ConfigLoader._load_yaml_file(config_path / "fleet.yaml")
        mission_data = ConfigLoader._load_yaml_file(config_path / "mission.yaml")
        comm_data = ConfigLoader._load_yaml_file(config_path / "comm.yaml")
        safety_data = ConfigLoader._load_yaml_file(config_path / "safety.yaml")

        # 构建配置对象
        drones = [DroneConfig(**d) for d in fleet_data["drones"]]
        fleet = FleetConfig(drones=drones)

        leader_motion = LeaderMotionConfig(**mission_data["leader_motion"])
        phases = [MissionPhaseConfig(**phase) for phase in mission_data["phases"]]
        mission = MissionConfig(
            duration=mission_data["duration"],
            formation_type=mission_data["formation_type"],
            nominal_positions=mission_data["nominal_positions"],
            leader_motion=leader_motion,
            phases=phases,
        )
        comm = CommConfig(**comm_data)
        safety = SafetyConfig(**safety_data)
        control = ControlConfig(
            **fleet_data.get(
                "control", {"gain": 1.0, "damping": 2.0, "max_velocity": 1.0}
            )
        )

        app_config = AppConfig(
            fleet=fleet, mission=mission, comm=comm, safety=safety, control=control
        )

        ConfigLoader._validate(app_config)
        return app_config
