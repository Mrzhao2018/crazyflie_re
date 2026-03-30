"""机群模型 - 统一管理ID/索引/URI/Radio映射"""

from typing import Dict, List
from ..config.schema import FleetConfig


class FleetModel:
    """机群模型 - 唯一真相源"""

    def __init__(self, config: FleetConfig):
        self._drones = config.drones
        self._id_to_index: Dict[int, int] = {}
        self._index_to_id: Dict[int, int] = {}
        self._id_to_uri: Dict[int, str] = {}
        self._id_to_radio: Dict[int, int] = {}
        self._leader_ids: List[int] = []
        self._follower_ids: List[int] = []
        self._radio_groups: Dict[int, List[int]] = {}

        # 构建映射
        for idx, drone in enumerate(self._drones):
            self._id_to_index[drone.id] = idx
            self._index_to_id[idx] = drone.id
            self._id_to_uri[drone.id] = drone.uri
            self._id_to_radio[drone.id] = drone.radio_group

            if drone.role == "leader":
                self._leader_ids.append(drone.id)
            else:
                self._follower_ids.append(drone.id)

            if drone.radio_group not in self._radio_groups:
                self._radio_groups[drone.radio_group] = []
            self._radio_groups[drone.radio_group].append(drone.id)

    def all_ids(self) -> List[int]:
        return [d.id for d in self._drones]

    def leader_ids(self) -> List[int]:
        return self._leader_ids.copy()

    def follower_ids(self) -> List[int]:
        return self._follower_ids.copy()

    def id_to_index(self, drone_id: int) -> int:
        return self._id_to_index[drone_id]

    def index_to_id(self, idx: int) -> int:
        return self._index_to_id[idx]

    def get_uri(self, drone_id: int) -> str:
        return self._id_to_uri[drone_id]

    def get_radio_group(self, drone_id: int) -> int:
        return self._id_to_radio[drone_id]

    def get_group_members(self, group: int) -> List[int]:
        return self._radio_groups.get(group, []).copy()

    def is_leader(self, drone_id: int) -> bool:
        return drone_id in self._leader_ids

    def is_follower(self, drone_id: int) -> bool:
        return drone_id in self._follower_ids
