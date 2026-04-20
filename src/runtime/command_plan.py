"""命令计划 - Scheduler和Executor之间的消息格式"""

from dataclasses import dataclass
from typing import Literal, Dict
import numpy as np


@dataclass
class LeaderAction:
    kind: Literal["takeoff", "batch_goto", "start_trajectory", "land"]
    drone_ids: list[int]
    payload: dict


@dataclass
class FollowerAction:
    """Follower setpoint。

    ``kind="velocity"`` 走 ``send_velocity_world_setpoint``，只用 ``velocity``；
    ``kind="full_state"`` 走 ``send_full_state_setpoint``，需要填 ``position`` +
    ``velocity`` + ``acceleration`` 三元组，下游 onboard（Mellinger/INDI）闭环。
    """

    kind: Literal["velocity", "full_state"]
    drone_id: int
    velocity: np.ndarray  # [vx, vy, vz]
    position: np.ndarray | None = None  # [x, y, z]，仅 full_state 必填
    acceleration: np.ndarray | None = None  # [ax, ay, az]，仅 full_state 必填


@dataclass
class HoldAction:
    drone_id: int


@dataclass
class TxPlan:
    """发送计划"""

    leader_actions: list[LeaderAction]
    follower_actions: list[FollowerAction]
    hold_actions: list[HoldAction]
    diagnostics: dict | None = None
