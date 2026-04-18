"""SafetyManager.fast_gate_decision：
只有部分 radio_group 掉线时返回 HOLD_GROUP（不 ABORT 整队），越界或全组掉线才 ABORT。
"""

import numpy as np

from src.config.schema import SafetyConfig
from src.runtime.safety_manager import SafetyManager
from src.runtime.pose_snapshot import PoseSnapshot


class FakeFleet:
    def __init__(self):
        self._drones = {1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2}

    def all_ids(self):
        return list(self._drones.keys())

    def id_to_index(self, drone_id):
        return list(self._drones.keys()).index(drone_id)

    def get_radio_group(self, drone_id):
        return self._drones[drone_id]


fleet = FakeFleet()
config = SafetyConfig(
    boundary_min=[-1.5, -1.5, -0.1],
    boundary_max=[1.5, 1.5, 1.5],
    pose_timeout=1.0,
    max_condition_number=100.0,
)
safety = SafetyManager(config, fleet)


def _snapshot(fresh_mask, disconnected_ids):
    positions = np.array(
        [
            [0.0, 0.0, 0.5],
            [0.0, 0.1, 0.5],
            [0.0, 0.2, 0.5],
            [0.0, 0.3, 0.5],
            [0.0, 0.4, 0.5],
            [0.0, 0.5, 0.5],
        ]
    )
    return PoseSnapshot(
        seq=1,
        t_meas=0.0,
        positions=positions,
        fresh_mask=np.array(fresh_mask, dtype=bool),
        disconnected_ids=list(disconnected_ids),
    )


# ---- 全部正常：EXECUTE ----------------------------------------------------

snap_ok = _snapshot([1, 1, 1, 1, 1, 1], [])
decision = safety.fast_gate_decision(snap_ok)
assert decision.action == "EXECUTE"
assert decision.reason_codes == []
assert decision.degrade_groups == []

# ---- 单组部分掉线 -> HOLD_GROUP ------------------------------------------

snap_partial = _snapshot([1, 1, 0, 0, 1, 1], [3, 4])  # group 1 两机掉线
decision = safety.fast_gate_decision(snap_partial)
assert decision.action == "HOLD_GROUP"
assert decision.degrade_groups == [1]
assert any(code.startswith("DISCONNECTED") for code in decision.reason_codes)

# ---- 所有组都有掉线 -> ABORT ---------------------------------------------

snap_all = _snapshot([0, 1, 1, 0, 1, 0], [1, 4, 6])  # group 0/1/2 均中招
decision = safety.fast_gate_decision(snap_all)
assert decision.action == "ABORT"
assert set(decision.degrade_groups) == set()  # ABORT 时不再做 group 降级

# ---- 越界永远是 ABORT，无论掉线分布 ----------------------------------------

snap_oob = _snapshot([1, 1, 1, 1, 1, 1], [])
snap_oob.positions[0] = np.array([10.0, 0.0, 0.5])  # 超 boundary_max
decision = safety.fast_gate_decision(snap_oob)
assert decision.action == "ABORT"
assert "OUT_OF_BOUNDS" in decision.reason_codes

# ---- 向后兼容：fast_gate() 老签名保持不变 ---------------------------------

blocked, reasons = safety.fast_gate(snap_all)
assert blocked is True
assert any("DISCONNECTED" in r for r in reasons)

blocked, reasons = safety.fast_gate(snap_partial)
# 旧行为：partial disconnect 仍然 blocked，保持原有 ABORT 语义兼容旧主循环
assert blocked is True

print("[OK] SafetyManager.fast_gate_decision group-level HOLD verified")
