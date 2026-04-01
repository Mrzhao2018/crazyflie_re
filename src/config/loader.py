"""配置加载器"""

import yaml
from pathlib import Path
from typing import cast
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
            if previous_end is not None and abs(phase.t_start - previous_end) > 1e-6:
                raise ValueError("mission.phases 必须连续，不能留时间空档")
            previous_end = phase.t_end

        if config.mission.phases[0].t_start != 0.0:
            raise ValueError("mission.phases 必须从 t_start=0.0 开始")

        if abs(config.mission.duration - config.mission.phases[-1].t_end) > 1e-6:
            raise ValueError("mission.duration 必须等于最后一个 phase 的 t_end")

        if (
            config.mission.leader_motion.translation is not None
            and len(config.mission.leader_motion.translation) != 3
        ):
            raise ValueError("leader_motion.translation 必须是 3 维向量")

        if config.mission.leader_motion.trajectory_start_addr < 0:
            raise ValueError("trajectory_start_addr 不能为负数")

        if config.mission.leader_motion.trajectory_sample_dt <= 0:
            raise ValueError("trajectory_sample_dt 必须大于 0")

        if config.safety.min_vbat < 0:
            raise ValueError("min_vbat 不能小于 0；设为 0 可关闭电量检查")

        if config.safety.hold_auto_land_timeout <= 0:
            raise ValueError("hold_auto_land_timeout 必须大于 0")

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

        if config.startup.mode == "manual_leader" and config.startup.manual is None:
            raise ValueError("manual_leader 模式需要提供 startup.manual 配置")

        if config.startup.manual is not None:
            if config.startup.manual.translation_step <= 0:
                raise ValueError("manual.translation_step 必须大于 0")
            if config.startup.manual.vertical_step <= 0:
                raise ValueError("manual.vertical_step 必须大于 0")
            if config.startup.manual.scale_step <= 0:
                raise ValueError("manual.scale_step 必须大于 0")
            if config.startup.manual.rotation_step_deg <= 0:
                raise ValueError("manual.rotation_step_deg 必须大于 0")
            if config.startup.manual.min_scale <= 0:
                raise ValueError("manual.min_scale 必须大于 0")
            if config.startup.manual.min_scale >= config.startup.manual.max_scale:
                raise ValueError("manual 必须满足 min_scale < max_scale")

    @staticmethod
    def load(config_dir: str, startup_mode_override: str | None = None) -> AppConfig:
        """加载所有配置文件"""
        config_path = Path(config_dir)

        # 加载各个配置文件
        fleet_data = ConfigLoader._load_yaml_file(config_path / "fleet.yaml")
        mission_data = ConfigLoader._load_yaml_file(config_path / "mission.yaml")
        comm_data = ConfigLoader._load_yaml_file(config_path / "comm.yaml")
        safety_data = ConfigLoader._load_yaml_file(config_path / "safety.yaml")
        startup_file = config_path / "startup.yaml"
        startup_data = (
            ConfigLoader._load_yaml_file(startup_file)
            if startup_file.exists()
            else {"mode": "auto"}
        )

        if startup_mode_override is not None:
            startup_data = {**startup_data, "mode": startup_mode_override}

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
        manual_cfg = startup_data.get("manual")
        if manual_cfg is not None and not isinstance(manual_cfg, dict):
            raise ValueError("startup.manual 必须是对象映射")

        startup_mode = startup_data.get("mode", "auto")
        if startup_mode not in {"auto", "manual_leader"}:
            raise ValueError("startup.mode 只能是 auto 或 manual_leader")

        startup = StartupConfig(
            mode=cast(Literal["auto", "manual_leader"], startup_mode),
            manual=(ManualLeaderControlConfig(**manual_cfg) if manual_cfg else None),
        )
        safety = SafetyConfig(**safety_data)
        control = ControlConfig(
            **fleet_data.get(
                "control", {"gain": 1.0, "damping": 2.0, "max_velocity": 1.0}
            )
        )

        app_config = AppConfig(
            fleet=fleet,
            mission=mission,
            comm=comm,
            startup=startup,
            safety=safety,
            control=control,
        )

        ConfigLoader._validate(app_config)
        return app_config
