"""CflibLinkManager.reconnect 契约：有限次数的重连 + 结构化事件."""

from unittest.mock import MagicMock, patch

from src.adapters import cflib_link_manager as link_manager_module
from src.adapters.cflib_link_manager import CflibLinkManager


class FakeFleet:
    def __init__(self):
        self._drones = {1: "radio://0/60/2M/A01"}
        self._radio = {1: 0}

    def all_ids(self):
        return list(self._drones.keys())

    def get_uri(self, drone_id):
        return self._drones[drone_id]

    def get_radio_group(self, drone_id):
        return self._radio[drone_id]


# ---- 成功重连 ----------------------------------------------------------

with patch.object(link_manager_module, "cflib", MagicMock()), \
     patch.object(link_manager_module, "Crazyflie", MagicMock()), \
     patch.object(link_manager_module, "SyncCrazyflie") as MockSync:

    # 首次 open_link + close_link 正常；第二次 open_link 成功（模拟重连）
    initial_scf = MagicMock()
    initial_scf.cf.is_connected.return_value = True
    new_scf = MagicMock()
    new_scf.cf.is_connected.return_value = True
    MockSync.side_effect = [initial_scf, new_scf]

    fleet = FakeFleet()
    manager = CflibLinkManager(
        fleet,
        connect_pace_s=0.0,
        connect_timeout_s=0.5,
    )
    manager.connect_all()
    assert manager.get(1) is initial_scf

    result = manager.reconnect(1, attempts=2, backoff_s=0.01, timeout_s=0.5)

    assert result["ok"] is True
    assert result["drone_id"] == 1
    assert result["attempt_count"] == 1
    assert manager.get(1) is new_scf
    # 老链路应当被 close
    initial_scf.close_link.assert_called()

# ---- 最终失败 ----------------------------------------------------------

with patch.object(link_manager_module, "cflib", MagicMock()), \
     patch.object(link_manager_module, "Crazyflie", MagicMock()), \
     patch.object(link_manager_module, "SyncCrazyflie") as MockSync:

    initial_scf = MagicMock()
    initial_scf.cf.is_connected.return_value = True

    def fail_scf(*args, **kwargs):
        bad = MagicMock()
        # open_link 抛错模拟连接失败
        bad.open_link.side_effect = RuntimeError("radio busy")
        return bad

    MockSync.side_effect = [initial_scf, fail_scf(), fail_scf(), fail_scf()]

    fleet = FakeFleet()
    manager = CflibLinkManager(
        fleet,
        connect_pace_s=0.0,
        connect_timeout_s=0.5,
    )
    manager.connect_all()

    result = manager.reconnect(1, attempts=3, backoff_s=0.001, timeout_s=0.2)
    assert result["ok"] is False
    assert result["attempt_count"] == 3
    assert "radio busy" in result.get("error", "")

# ---- 非法 attempts 参数 ------------------------------------------------

try:
    CflibLinkManager(FakeFleet()).reconnect(1, attempts=0, backoff_s=0.0, timeout_s=0.1)
except ValueError:
    pass
else:
    raise AssertionError("attempts=0 应当被拒绝")

print("[OK] CflibLinkManager.reconnect contracts verified")
