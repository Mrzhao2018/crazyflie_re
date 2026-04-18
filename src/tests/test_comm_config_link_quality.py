"""CommConfig 新增 link_quality / connect_pace 相关字段的默认值契约。"""

from src.config.schema import CommConfig


# 旧 comm.yaml 风格——不传新字段也能构造
cfg = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
)
assert cfg.connect_pace_s == 0.2, "connect_pace_s 默认值必须保留 0.2（保持旧行为）"
assert cfg.connect_timeout_s == 5.0, "connect_timeout_s 默认值 5.0"
assert cfg.link_quality_enabled is True, "link_quality 默认开启以采集观测数据"

# 可显式覆盖
cfg2 = CommConfig(
    pose_log_freq=10.0,
    follower_tx_freq=8.0,
    leader_update_freq=1.0,
    parked_hold_freq=0.5,
    connect_pace_s=0.05,
    connect_timeout_s=10.0,
    link_quality_enabled=False,
)
assert cfg2.connect_pace_s == 0.05
assert cfg2.connect_timeout_s == 10.0
assert cfg2.link_quality_enabled is False

print("[OK] CommConfig new fields defaults verified")
