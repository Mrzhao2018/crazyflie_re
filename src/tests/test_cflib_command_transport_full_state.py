"""cmd_full_state int16 饱和防御契约测试。

cflib `send_full_state_setpoint` 把 pos/vel/acc 量化成 int16 (mm / mm·s /
mm·s^2)。host 侧必须在下发前主动拦截：

* 任意分量为 NaN / +Inf / -Inf
* |v| 超过 ``FULL_STATE_INT16_LIMIT`` (32.0)

否则 struct.pack('<h', ...) 会抛 struct.error，executor 会把整个 radio_group
的 follower 拖进 executor_group_hold。预期行为：抛 ``ValueError``（被上层
``classify_command_failure`` 归入 ``invalid_command`` / ``retryable=false``）。
"""

import math

import numpy as np

from src.adapters.cflib_command_transport import (
    CflibCommandTransport,
    FULL_STATE_INT16_LIMIT,
    _validate_full_state_vector,
)


class _FakeCommander:
    def __init__(self):
        self.calls = []

    def send_full_state_setpoint(
        self, pos, vel, acc, orientation, rollrate, pitchrate, yawrate
    ):
        self.calls.append(
            (
                "full_state",
                tuple(pos),
                tuple(vel),
                tuple(acc),
                tuple(orientation),
                rollrate,
                pitchrate,
                yawrate,
            )
        )


class _FakeCF:
    def __init__(self):
        self.commander = _FakeCommander()


class _FakeSCF:
    def __init__(self):
        self.cf = _FakeCF()


class _FakeLinkManager:
    def __init__(self):
        self.fleet = type(
            "FakeFleet",
            (),
            {"get_radio_group": lambda self, drone_id: 0},
        )()
        self._scf = _FakeSCF()

    def get(self, drone_id):
        return self._scf


# --- validator 单元层面：NaN / Inf / 越界 ---

for bad_value in (float("nan"), float("inf"), float("-inf")):
    for bad_axis in range(3):
        vec = [0.0, 0.0, 0.0]
        vec[bad_axis] = bad_value
        try:
            _validate_full_state_vector("pos", vec)
        except ValueError as exc:
            assert "not finite" in str(exc), f"expected not-finite message, got {exc!r}"
        else:
            raise AssertionError(
                f"expected ValueError for {bad_value!r} at axis {bad_axis}"
            )

for bad_magnitude in (FULL_STATE_INT16_LIMIT + 0.01, 1000.0, -1000.0):
    try:
        _validate_full_state_vector("vel", (bad_magnitude, 0.0, 0.0))
    except ValueError as exc:
        assert "int16 quantization" in str(exc)
    else:
        raise AssertionError(f"expected ValueError for magnitude {bad_magnitude}")

_validate_full_state_vector("pos", (1.0, -1.0, 0.5))
_validate_full_state_vector("vel", np.array([0.1, 0.2, 0.3]))
_validate_full_state_vector("acc", [FULL_STATE_INT16_LIMIT, 0.0, 0.0])  # 边界等值允许

try:
    _validate_full_state_vector("pos", (1.0, 2.0))
except ValueError as exc:
    assert "length-3" in str(exc)
else:
    raise AssertionError("expected ValueError for length-2 vector")


# --- 集成层面：transport.cmd_full_state 拦截与正常路径 ---

link_manager = _FakeLinkManager()
transport = CflibCommandTransport(link_manager)

# 正常调用应当透传到 FakeCommander
transport.cmd_full_state(
    drone_id=1,
    pos=(0.5, 0.5, 1.0),
    vel=(0.1, 0.0, 0.0),
    acc=(0.0, 0.0, 0.0),
)
assert link_manager._scf.cf.commander.calls[-1][0] == "full_state"
assert link_manager._scf.cf.commander.calls[-1][4] == (0.0, 0.0, 0.0, 1.0)
assert link_manager._scf.cf.commander.calls[-1][5:] == (0.0, 0.0, 0.0)
assert transport.last_velocity_command_time(1) is not None

# NaN 应该直接抛 ValueError 且不会触达 cflib
calls_before = len(link_manager._scf.cf.commander.calls)
try:
    transport.cmd_full_state(
        drone_id=1,
        pos=(math.nan, 0.0, 0.0),
        vel=(0.0, 0.0, 0.0),
        acc=(0.0, 0.0, 0.0),
    )
except ValueError as exc:
    assert "not finite" in str(exc)
else:
    raise AssertionError("expected ValueError for NaN pos")
assert len(link_manager._scf.cf.commander.calls) == calls_before

# 越界 velocity 同样拦截
try:
    transport.cmd_full_state(
        drone_id=1,
        pos=(0.0, 0.0, 0.0),
        vel=(50.0, 0.0, 0.0),
        acc=(0.0, 0.0, 0.0),
    )
except ValueError as exc:
    assert "int16 quantization" in str(exc)
else:
    raise AssertionError("expected ValueError for oversized velocity")
assert len(link_manager._scf.cf.commander.calls) == calls_before

# Inf acc 同样拦截
try:
    transport.cmd_full_state(
        drone_id=1,
        pos=(0.0, 0.0, 0.0),
        vel=(0.0, 0.0, 0.0),
        acc=(0.0, float("inf"), 0.0),
    )
except ValueError as exc:
    assert "not finite" in str(exc)
else:
    raise AssertionError("expected ValueError for inf acc")
assert len(link_manager._scf.cf.commander.calls) == calls_before


# --- classify_command_failure 对 ValueError 归入 invalid_command / 不可重试 ---

failure = transport.classify_command_failure(
    drone_id=1,
    command_kind="full_state",
    exception=ValueError("full_state vel.x=50.0 exceeds int16 quantization range"),
)
assert failure["failure_category"] == "invalid_command"
assert failure["retryable"] is False

print("[OK] cmd_full_state NaN/Inf/int16-overflow guard verified")
