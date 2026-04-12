"""配置数据结构定义"""

from dataclasses import dataclass
from typing import Literal


@dataclass
class DroneConfig:
    """单个无人机配置"""

    id: int
    uri: str
    role: Literal["leader", "follower"]
    radio_group: int


@dataclass
class FleetConfig:
    """机群配置"""

    drones: list[DroneConfig]


@dataclass
class LeaderMotionConfig:
    """Leader任务配置"""

    mode: Literal["affine_rotation", "hold", "trajectory"]
    angular_rate: float = 0.0
    translation: list[float] | None = None
    trajectory_enabled: bool = False
    trajectory_id: int = 1
    trajectory_time_scale: float = 1.0
    trajectory_relative_position: bool = False
    trajectory_relative_yaw: bool = False
    trajectory_reversed: bool = False
    trajectory_type: Literal["poly4d"] = "poly4d"
    trajectory_start_addr: int = 0
    trajectory_pieces: list[dict] | None = None
    trajectory_sample_dt: float = 5.0


@dataclass
class MissionPhaseConfig:
    """任务阶段配置"""

    name: str
    t_start: float
    t_end: float
    mode: Literal[
        "settle", "trajectory_entry", "run_entry", "formation_run", "hold", "land"
    ]


@dataclass
class MissionConfig:
    """任务配置"""

    duration: float
    formation_type: str
    nominal_positions: list[list[float]]  # [[x,y,z], ...]
    leader_motion: LeaderMotionConfig
    phases: list[MissionPhaseConfig]


@dataclass
class CommConfig:
    """通信配置"""

    pose_log_freq: float  # Hz
    follower_tx_freq: float  # Hz
    leader_update_freq: float  # Hz
    parked_hold_freq: float  # Hz
    follower_cmd_deadband: float = 0.0
    readiness_wait_for_params: bool = True
    readiness_reset_estimator: bool = False
    connect_groups_in_parallel: bool = False
    trajectory_upload_groups_in_parallel: bool = False


@dataclass
class ManualLeaderControlConfig:
    """手动leader控制配置"""

    translation_step: float = 0.1
    vertical_step: float = 0.1
    scale_step: float = 0.05
    rotation_step_deg: float = 5.0
    default_axis: Literal["x", "y", "z"] = "z"
    min_scale: float = 0.6
    max_scale: float = 1.6


@dataclass
class StartupConfig:
    """应用启动模式配置"""

    mode: Literal["auto", "manual_leader"] = "auto"
    manual: ManualLeaderControlConfig | None = None


@dataclass
class SafetyConfig:
    """安全配置"""

    boundary_min: list[float]  # [x, y, z]
    boundary_max: list[float]
    pose_timeout: float  # seconds
    max_condition_number: float
    max_command_norm: float = 2.0
    estimator_variance_threshold: float = 0.001
    min_vbat: float = 3.6
    hold_auto_land_timeout: float = 3.0
    velocity_stream_watchdog_action: Literal[
        "telemetry", "hold", "degrade"
    ] = "telemetry"


@dataclass
class ControlConfig:
    """控制配置"""

    gain: float
    damping: float
    max_velocity: float
    feedforward_gain: float = 1.0
    max_feedforward_velocity: float = 0.3
    gain_xy: float | None = None
    gain_z: float | None = None
    feedforward_gain_xy: float | None = None
    feedforward_gain_z: float | None = None
    max_feedforward_velocity_xy: float | None = None
    max_feedforward_velocity_z: float | None = None
    radial_gain_scale_xy: float = 0.0
    radial_feedforward_scale_xy: float = 0.0


@dataclass
class AppConfig:
    """应用总配置"""

    fleet: FleetConfig
    mission: MissionConfig
    comm: CommConfig
    startup: StartupConfig
    safety: SafetyConfig
    control: ControlConfig
