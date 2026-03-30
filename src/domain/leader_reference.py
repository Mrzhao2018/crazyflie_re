"""Leader参考生成器 - 确保4个leader共时更新"""

import numpy as np
from dataclasses import dataclass

from .mission_profile import MissionProfile


@dataclass
class LeaderReferenceFrame:
    """Leader参考帧 - 所有leader同一时刻的目标"""

    t_ref: float
    leader_ids: list[int]
    positions: dict  # {drone_id: np.ndarray}
    mode: str = "batch_goto"
    trajectory: dict | None = None


class LeaderReferenceGenerator:
    """Leader参考生成器 - 关键：4个leader必须共时"""

    def __init__(self, mission_profile: MissionProfile, formation_model, fleet_model):
        self.mission = mission_profile
        self.formation = formation_model
        self.fleet = fleet_model
        self.leader_ids = fleet_model.leader_ids()

    def reference_at(self, t: float) -> LeaderReferenceFrame:
        """生成t时刻的共时参考

        关键：所有leader使用同一个时间戳t
        """
        if self.mission.trajectory_enabled():
            per_leader = {
                lid: self.mission.trajectory_spec_for_nominal(
                    self.formation.nominal_position(lid)
                )
                for lid in self.leader_ids
            }
            return LeaderReferenceFrame(
                t_ref=t,
                leader_ids=self.leader_ids.copy(),
                positions={},
                mode="trajectory",
                trajectory={"per_leader": per_leader},
            )

        transform = self.mission.affine_transform_at(t)
        A = transform.A
        b = transform.b

        positions = {}
        for lid in self.leader_ids:
            r_nominal = self.formation.nominal_position(lid)
            p_ref = A @ r_nominal + b
            positions[lid] = p_ref

        return LeaderReferenceFrame(
            t_ref=t,
            leader_ids=self.leader_ids.copy(),
            positions=positions,
            mode="batch_goto",
        )
