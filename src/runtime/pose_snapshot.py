"""Pose快照 - 核心在线状态数据结构"""
from dataclasses import dataclass
import numpy as np


@dataclass
class PoseSnapshot:
    """统一的pose快照"""
    seq: int  # 序列号，只有新测量时才递增
    t_meas: float  # 真实测量时间戳
    positions: np.ndarray  # (n, 3) 所有无人机位置
    fresh_mask: np.ndarray  # (n,) bool数组，标记哪些是新数据
    disconnected_ids: list[int]  # 断连的drone_id列表
