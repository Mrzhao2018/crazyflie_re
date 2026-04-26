"""配置加载器"""

import yaml
from pathlib import Path
from typing import cast
from .schema import *


class ConfigLoader:
    """从YAML文件加载配置"""

    @staticmethod
    def _validate_frequency(name: str, value: float) -> None:
        if value <= 0:
            raise ValueError(f"{name} 必须大于 0")

    @staticmethod
    def _validate_cross_config(config: AppConfig) -> None:
        pose_period = 1.0 / config.comm.pose_log_freq
        pose_period_ms = int(1000.0 / config.comm.pose_log_freq)
        follower_period = 1.0 / config.comm.follower_tx_freq
        leader_period = 1.0 / config.comm.leader_update_freq
        health_period = max(0.2, (pose_period_ms * 5) / 1000.0)

        if config.safety.pose_timeout <= pose_period:
            raise ValueError(
                "pose_timeout 必须大于 pose_log_freq 对应的采样周期，否则新鲜度判断会误触发"
            )
        if config.safety.pose_timeout <= health_period:
            raise ValueError(
                "pose_timeout 必须大于 health log block 周期，否则 health freshness 会在 preflight 中误触发"
            )

        if config.comm.leader_update_freq > config.comm.follower_tx_freq:
            raise ValueError("leader_update_freq 不能高于 follower_tx_freq")

        if config.mission.leader_motion.trajectory_enabled:
            sample_dt = config.mission.leader_motion.trajectory_sample_dt
            if sample_dt < pose_period:
                raise ValueError(
                    "trajectory_sample_dt 不能小于 pose 采样周期，否则参考轨迹采样没有实际收益"
                )
            if sample_dt < follower_period:
                raise ValueError(
                    "trajectory_sample_dt 不能小于 follower 指令周期，否则会导致参考更新与控制周期不匹配"
                )
            if sample_dt < leader_period:
                raise ValueError(
                    "trajectory_sample_dt 不能小于 leader 更新周期，否则会导致轨迹段采样过密"
                )

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
        if config.mission.leader_motion.condition_soft_limit <= 0:
            raise ValueError("condition_soft_limit 必须大于 0")
        if config.mission.leader_motion.condition_penalty_scale < 0:
            raise ValueError("condition_penalty_scale 不能小于 0")
        if config.mission.leader_motion.condition_stress_min_scale <= 0:
            raise ValueError("condition_stress_min_scale 必须大于 0")
        if config.mission.leader_motion.condition_stress_period <= 0:
            raise ValueError("condition_stress_period 必须大于 0")

        if config.safety.min_vbat < 0:
            raise ValueError("min_vbat 不能小于 0；设为 0 可关闭电量检查")

        if config.safety.hold_auto_land_timeout <= 0:
            raise ValueError("hold_auto_land_timeout 必须大于 0")
        if config.safety.executor_group_failure_streak <= 0:
            raise ValueError("executor_group_failure_streak 必须大于 0")

        if config.safety.velocity_stream_watchdog_action not in {
            "telemetry",
            "hold",
            "degrade",
        }:
            raise ValueError(
                "velocity_stream_watchdog_action 必须是 telemetry、hold 或 degrade"
            )

        for freq_name in (
            "pose_log_freq",
            "follower_tx_freq",
            "leader_update_freq",
            "parked_hold_freq",
        ):
            ConfigLoader._validate_frequency(freq_name, getattr(config.comm, freq_name))

        if config.comm.connect_pace_s < 0:
            raise ValueError("connect_pace_s 不能小于 0；0 表示不在多机连接之间 sleep")
        if config.comm.connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s 必须大于 0")
        if config.comm.telemetry_queue_max <= 0:
            raise ValueError("telemetry_queue_max 必须大于 0")
        if config.comm.telemetry_flush_every_n <= 0:
            raise ValueError("telemetry_flush_every_n 必须大于 0")

        if config.control.feedforward_gain < 0:
            raise ValueError("feedforward_gain 不能小于 0")
        if config.control.max_feedforward_velocity < 0:
            raise ValueError("max_feedforward_velocity 不能小于 0")
        if config.control.dynamics_model_order not in {1, 2}:
            raise ValueError("dynamics_model_order 只能是 1 或 2")
        if config.control.output_mode not in {"velocity", "full_state"}:
            raise ValueError("control.output_mode 只能是 'velocity' 或 'full_state'")
        if config.control.onboard_controller not in {"pid", "mellinger", "indi"}:
            raise ValueError("control.onboard_controller 只能是 'pid'/'mellinger'/'indi'")
        if config.control.output_mode == "full_state" and config.control.onboard_controller == "pid":
            raise ValueError(
                "full_state 模式建议 onboard_controller=mellinger；PID 不吃 pos/vel/acc 三元组"
            )
        if (
            config.control.output_mode == "full_state"
            and config.control.dynamics_model_order != 2
        ):
            raise ValueError(
                "full_state 模式要求 dynamics_model_order=2，确保 host 侧走能产出 pos/vel/acc 参考三元组的控制器路径"
            )
        if config.control.velocity_feedback_gain < 0:
            raise ValueError("velocity_feedback_gain 不能小于 0")
        if config.control.acceleration_feedforward_gain < 0:
            raise ValueError("acceleration_feedforward_gain 不能小于 0")
        if config.control.mass_kg <= 0:
            raise ValueError("mass_kg 必须大于 0")
        if config.control.damping_coeff < 0:
            raise ValueError("damping_coeff 不能小于 0")
        if config.control.max_acceleration <= 0:
            raise ValueError("max_acceleration 必须大于 0")
        if config.control.estimated_total_delay_ms < 0:
            raise ValueError("estimated_total_delay_ms 不能小于 0")
        if config.control.delay_prediction_gain < 0:
            raise ValueError("delay_prediction_gain 不能小于 0")
        for attr in (
            "gain_xy",
            "gain_z",
            "feedforward_gain_xy",
            "feedforward_gain_z",
            "max_feedforward_velocity_xy",
            "max_feedforward_velocity_z",
            "radial_gain_scale_xy",
            "radial_feedforward_scale_xy",
        ):
            value = getattr(config.control, attr)
            if value is not None and value < 0:
                raise ValueError(f"{attr} 不能小于 0")

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

        ConfigLoader._validate_cross_config(config)

    @staticmethod
    def _read_raw_text(path: Path) -> str | None:
        if not path.exists():
            return None
        return path.read_text(encoding="utf-8")

    @staticmethod
    def load(config_dir: str, startup_mode_override: str | None = None) -> AppConfig:
        """加载所有配置文件"""
        config_path = Path(config_dir)

        raw_files: dict[str, str] = {}
        for path in sorted(config_path.glob("*.yaml")):
            text = ConfigLoader._read_raw_text(path)
            if text is not None:
                raw_files[path.name] = text

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
            raw_files=raw_files,
        )

        ConfigLoader._validate(app_config)
        return app_config
