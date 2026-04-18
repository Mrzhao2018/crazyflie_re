"""CflibLinkManager.connect_all 报告增强：包含 parallel 标记与每组耗时摘要。"""

from unittest.mock import MagicMock, patch

from src.adapters import cflib_link_manager as link_manager_module
from src.adapters.cflib_link_manager import CflibLinkManager


class FakeFleet:
    def __init__(self):
        self._drones = {
            1: "radio://0/60/2M/A01",
            2: "radio://0/60/2M/A02",
            3: "radio://1/110/2M/A01",
        }
        self._radio = {1: 0, 2: 0, 3: 1}

    def all_ids(self):
        return list(self._drones.keys())

    def get_uri(self, drone_id):
        return self._drones[drone_id]

    def get_radio_group(self, drone_id):
        return self._radio[drone_id]


# 让 init_drivers 与 Crazyflie/SyncCrazyflie 都变成 no-op mock
with patch.object(link_manager_module, "cflib", MagicMock()), \
     patch.object(link_manager_module, "Crazyflie", MagicMock()), \
     patch.object(link_manager_module, "SyncCrazyflie") as MockSync:
    scf_instance = MagicMock()
    scf_instance.cf.is_connected.return_value = True
    MockSync.return_value = scf_instance

    fleet = FakeFleet()

    # ---- sequential ---------------------------------------------------

    manager = CflibLinkManager(fleet, connect_pace_s=0.0, connect_timeout_s=1.0)
    report = manager.connect_all(parallel_groups=False)
    assert report["ok"] is True
    assert report["parallel"] is False
    assert set(report["connected"]) == {1, 2, 3}
    assert set(report["per_group_duration_s"]) == {0, 1}
    for gid, duration in report["per_group_duration_s"].items():
        assert isinstance(duration, float), f"group {gid} 的 duration_s 不是 float"
        assert duration >= 0.0

    # ---- parallel -----------------------------------------------------

    manager2 = CflibLinkManager(fleet, connect_pace_s=0.0, connect_timeout_s=1.0)
    report2 = manager2.connect_all(parallel_groups=True)
    assert report2["ok"] is True
    assert report2["parallel"] is True
    assert set(report2["per_group_duration_s"]) == {0, 1}

print("[OK] CflibLinkManager.connect_all parallel flag & per-group duration verified")
