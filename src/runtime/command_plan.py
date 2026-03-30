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
    kind: Literal["velocity"]
    drone_id: int
    velocity: np.ndarray  # [vx, vy, vz]


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
