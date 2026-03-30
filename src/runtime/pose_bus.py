"""PoseBus - 管理pose更新和seq"""

import threading
import time
import numpy as np
from .pose_snapshot import PoseSnapshot


class PoseBus:
    def __init__(self, fleet_model, pose_timeout=1.0):
        self.fleet = fleet_model
        self.pose_timeout = pose_timeout
        self._agent_poses = {}  # {drone_id: (pos, timestamp)}
        self._seq_counter = 0
        self._lock = threading.Lock()

    def update_agent(self, drone_id: int, pos: np.ndarray, t_meas: float):
        """单个agent的pose更新 - 只有这里才递增seq"""
        with self._lock:
            self._agent_poses[drone_id] = (pos, t_meas)
            self._seq_counter += 1  # 有新测量才+1

    def latest(self) -> PoseSnapshot | None:
        """生成最新快照"""
        with self._lock:
            if not self._agent_poses:
                return None

            now = time.time()
            n = len(self.fleet.all_ids())
            positions = np.zeros((n, 3))
            fresh_mask = np.zeros(n, dtype=bool)
            disconnected_ids = []

            for drone_id in self.fleet.all_ids():
                idx = self.fleet.id_to_index(drone_id)

                if drone_id in self._agent_poses:
                    pos, ts = self._agent_poses[drone_id]
                    positions[idx] = pos

                    # 检查是否fresh
                    if now - ts < self.pose_timeout:
                        fresh_mask[idx] = True
                    else:
                        disconnected_ids.append(drone_id)
                else:
                    disconnected_ids.append(drone_id)

            # t_meas取最新的测量时间
            t_meas = (
                max(ts for _, ts in self._agent_poses.values())
                if self._agent_poses
                else now
            )

            return PoseSnapshot(
                seq=self._seq_counter,
                t_meas=t_meas,
                positions=positions,
                fresh_mask=fresh_mask,
                disconnected_ids=disconnected_ids,
            )

    def has_newer_than(self, seq: int) -> bool:
        with self._lock:
            return self._seq_counter > seq
