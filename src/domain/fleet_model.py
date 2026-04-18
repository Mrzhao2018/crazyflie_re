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
        leader_ids: list[int] = []
        follower_ids: list[int] = []
        radio_groups: Dict[int, List[int]] = {}

        for idx, drone in enumerate(self._drones):
            self._id_to_index[drone.id] = idx
            self._index_to_id[idx] = drone.id
            self._id_to_uri[drone.id] = drone.uri
            self._id_to_radio[drone.id] = drone.radio_group

            if drone.role == "leader":
                leader_ids.append(drone.id)
            else:
                follower_ids.append(drone.id)

            radio_groups.setdefault(drone.radio_group, []).append(drone.id)

        self._leader_ids: tuple[int, ...] = tuple(leader_ids)
        self._follower_ids: tuple[int, ...] = tuple(follower_ids)
        self._all_ids_cached: tuple[int, ...] = tuple(d.id for d in self._drones)
        self._radio_groups: Dict[int, tuple[int, ...]] = {
            g: tuple(ids) for g, ids in radio_groups.items()
        }

    def all_ids(self) -> tuple[int, ...]:
        return self._all_ids_cached

    def leader_ids(self) -> tuple[int, ...]:
        return self._leader_ids

    def follower_ids(self) -> tuple[int, ...]:
        return self._follower_ids

    def id_to_index(self, drone_id: int) -> int:
        return self._id_to_index[drone_id]

    def index_to_id(self, idx: int) -> int:
        return self._index_to_id[idx]

    def get_uri(self, drone_id: int) -> str:
        return self._id_to_uri[drone_id]

    def get_radio_group(self, drone_id: int) -> int:
        return self._id_to_radio[drone_id]

    def get_group_members(self, group: int) -> tuple[int, ...]:
        return self._radio_groups.get(group, ())

    def is_leader(self, drone_id: int) -> bool:
        return drone_id in self._leader_ids

    def is_follower(self, drone_id: int) -> bool:
        return drone_id in self._follower_ids
