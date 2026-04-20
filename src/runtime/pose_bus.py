"""PoseBus - 管理pose更新和seq"""

import threading
import time
import numpy as np
from .pose_snapshot import PoseSnapshot


class PoseBus:
    def __init__(self, fleet_model, pose_timeout=1.0):
        self.fleet = fleet_model
        self.pose_timeout = pose_timeout
        self._agent_poses = {}  # {drone_id: (pos, vel, timestamp)}; vel 可能为 None
        self._seq_counter = 0
        self._lock = threading.Lock()

        n = len(fleet_model.all_ids())
        # 预分配扫描缓冲 —— 每次 latest() 在这里填再 copy 出去
        self._scratch_positions = np.zeros((n, 3), dtype=float)
        self._scratch_velocities = np.zeros((n, 3), dtype=float)
        self._scratch_fresh = np.zeros(n, dtype=bool)
        self._scratch_vel_fresh = np.zeros(n, dtype=bool)
        self._all_ids_cache = tuple(fleet_model.all_ids())
        self._idx_cache = tuple(
            fleet_model.id_to_index(d) for d in self._all_ids_cache
        )

    def update_agent(
        self,
        drone_id: int,
        pos: np.ndarray,
        t_meas: float,
        velocity: np.ndarray | None = None,
    ):
        """单个agent的pose更新 - 只有这里才递增seq"""
        with self._lock:
            self._agent_poses[drone_id] = (pos, velocity, t_meas)
            self._seq_counter += 1

    def latest(self) -> PoseSnapshot | None:
        """生成最新快照"""
        with self._lock:
            if not self._agent_poses:
                return None

            now = time.time()
            positions = self._scratch_positions
            velocities = self._scratch_velocities
            fresh_mask = self._scratch_fresh
            vel_fresh_mask = self._scratch_vel_fresh
            positions.fill(0.0)
            velocities.fill(0.0)
            fresh_mask.fill(False)
            vel_fresh_mask.fill(False)
            disconnected_ids = []
            any_velocity = False

            latest_ts = None
            for cache_i, drone_id in enumerate(self._all_ids_cache):
                idx = self._idx_cache[cache_i]
                entry = self._agent_poses.get(drone_id)
                if entry is None:
                    disconnected_ids.append(drone_id)
                    continue
                pos, vel, ts = entry
                positions[idx] = pos
                is_fresh = now - ts < self.pose_timeout
                if is_fresh:
                    fresh_mask[idx] = True
                else:
                    disconnected_ids.append(drone_id)
                if vel is not None and is_fresh:
                    velocities[idx] = vel
                    vel_fresh_mask[idx] = True
                    any_velocity = True
                if latest_ts is None or ts > latest_ts:
                    latest_ts = ts

            t_meas = latest_ts if latest_ts is not None else now

            return PoseSnapshot(
                seq=self._seq_counter,
                t_meas=t_meas,
                positions=positions.copy(),
                fresh_mask=fresh_mask.copy(),
                disconnected_ids=disconnected_ids,
                velocities=velocities.copy() if any_velocity else None,
                velocity_fresh_mask=vel_fresh_mask.copy() if any_velocity else None,
            )

    def has_newer_than(self, seq: int) -> bool:
        with self._lock:
            return self._seq_counter > seq
