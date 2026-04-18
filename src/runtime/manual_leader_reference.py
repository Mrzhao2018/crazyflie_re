"""手动leader参考生成器"""

from dataclasses import dataclass
import numpy as np

from ..domain.leader_reference import LeaderReferenceFrame
from .manual_leader_state import ManualLeaderState


class ManualLeaderReferenceSource:
    """根据共享手动状态生成共时leader参考"""

    def __init__(self, formation_model, fleet_model, manual_state: ManualLeaderState):
        self.formation = formation_model
        self.fleet = fleet_model
        self.manual_state = manual_state
        self._reference_center = np.zeros(3, dtype=float)

    def initialize_from_measured_leaders(
        self, leader_positions: dict[int, np.ndarray] | None
    ) -> None:
        if leader_positions:
            matrix = np.array(list(leader_positions.values()), dtype=float)
            if len(matrix) > 0:
                self._reference_center = matrix.mean(axis=0)

        self.manual_state.initialize_from_transform(translation=self._reference_center)

    def initial_structure_reference(self, t: float = 0.0) -> LeaderReferenceFrame:
        positions = {}
        for leader_id in self.fleet.leader_ids():
            nominal = np.array(self.formation.nominal_position(leader_id), dtype=float)
            positions[leader_id] = nominal.copy()

        return LeaderReferenceFrame(
            t_ref=t,
            leader_ids=list(self.fleet.leader_ids()),
            positions=positions,
            mode="batch_goto",
        )

    def reference_at(self, t: float) -> LeaderReferenceFrame:
        state = self.manual_state.snapshot()
        positions = {}
        for leader_id in self.fleet.leader_ids():
            nominal = np.array(self.formation.nominal_position(leader_id), dtype=float)
            transformed = state.scale * (state.rotation @ nominal) + state.translation
            leader_offset = state.per_leader_offsets.get(leader_id)
            if leader_offset is not None:
                transformed = transformed + leader_offset
            positions[leader_id] = transformed

        return LeaderReferenceFrame(
            t_ref=t,
            leader_ids=list(self.fleet.leader_ids()),
            positions=positions,
            mode="batch_goto",
        )
