"""ConfigLoader 对 CommConfig 新字段的合法性校验。"""

import tempfile
from pathlib import Path

from src.config.loader import ConfigLoader


def _write_comm(tmp: Path, **extras):
    base = {
        "pose_log_freq": 10.0,
        "follower_tx_freq": 8.0,
        "leader_update_freq": 1.0,
        "parked_hold_freq": 0.5,
    }
    base.update(extras)
    import yaml
    (tmp / "comm.yaml").write_text(yaml.safe_dump(base), encoding="utf-8")


def _write_minimal_config(tmp: Path) -> None:
    import yaml
    (tmp / "fleet.yaml").write_text(
        yaml.safe_dump(
            {
                "drones": [
                    {"id": i, "uri": f"radio://0/60/2M/E7E7E7E70{i}", "role": "leader" if i <= 4 else "follower", "radio_group": 0}
                    for i in range(1, 7)
                ],
                "control": {"gain": 1.0, "damping": 2.0, "max_velocity": 0.7},
            }
        ),
        encoding="utf-8",
    )
    (tmp / "mission.yaml").write_text(
        yaml.safe_dump(
            {
                "duration": 10.0,
                "formation_type": "square",
                "nominal_positions": [[0, 0, 0]] * 6,
                "leader_motion": {"mode": "hold"},
                "phases": [
                    {"name": "settle", "t_start": 0, "t_end": 2, "mode": "settle"},
                    {"name": "formation_run", "t_start": 2, "t_end": 10, "mode": "formation_run"},
                ],
            }
        ),
        encoding="utf-8",
    )
    (tmp / "safety.yaml").write_text(
        yaml.safe_dump(
            {
                "boundary_min": [-1, -1, -0.1],
                "boundary_max": [1, 1, 1.5],
                "pose_timeout": 1.0,
                "max_condition_number": 100.0,
            }
        ),
        encoding="utf-8",
    )


with tempfile.TemporaryDirectory() as tmp_str:
    tmp = Path(tmp_str)
    _write_minimal_config(tmp)

    # 合法：默认
    _write_comm(tmp)
    cfg = ConfigLoader.load(str(tmp))
    assert cfg.comm.connect_pace_s == 0.2
    assert cfg.comm.connect_timeout_s == 5.0

    # 合法：显式设 0（关闭间隔）/较大超时
    _write_comm(tmp, connect_pace_s=0.0, connect_timeout_s=30.0)
    cfg = ConfigLoader.load(str(tmp))
    assert cfg.comm.connect_pace_s == 0.0
    assert cfg.comm.connect_timeout_s == 30.0

    # 非法：负的 pace
    _write_comm(tmp, connect_pace_s=-0.01)
    try:
        ConfigLoader.load(str(tmp))
    except ValueError as exc:
        assert "connect_pace_s" in str(exc)
    else:
        raise AssertionError("connect_pace_s 负数应当被拒绝")

    # 非法：connect_timeout_s <= 0
    _write_comm(tmp, connect_timeout_s=0.0)
    try:
        ConfigLoader.load(str(tmp))
    except ValueError as exc:
        assert "connect_timeout_s" in str(exc)
    else:
        raise AssertionError("connect_timeout_s 必须大于 0")

print("[OK] ConfigLoader validation for new comm fields verified")
