"""Cflib命令传输层 - 封装cflib命令调用"""

from __future__ import annotations

import logging
import math
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
from cflib.crazyflie.mem import (
    CompressedSegment,
    CompressedStart,
    MemoryElement,
    Poly4D,
)
from cflib.utils.reset_estimator import reset_estimator

logger = logging.getLogger(__name__)


POLY4D_RAW_PIECE_BYTES = 132
TRAJECTORY_MEMORY_BYTES = 4096
TRAJECTORY_MAX_PIECES = 255
TRAJECTORY_TYPE_POLY4D = 0
TRAJECTORY_TYPE_POLY4D_COMPRESSED = 1

# cflib `send_full_state_setpoint` 内部把 pos/vel/acc 乘 1000 量化成 int16（见
# cflib/crazyflie/commander.py ~ L203）。可表示范围 [-32.767, +32.767]，我们留
# 一点 headroom 拒绝 |v| > 32.0 的输入，避免 struct.pack `<h` 静默溢出或抛
# `struct.error` 把一整组 follower 拖进 executor_group_hold。NaN/Inf 同样拦截。
FULL_STATE_INT16_LIMIT = 32.0


def _validate_full_state_vector(name: str, vec) -> None:
    """Reject NaN/Inf 与 int16 量化越界的 full-state 分量。

    vec 可能是 tuple/list/ndarray。要求长度恰好为 3 且每个分量满足
    ``math.isfinite(v) and abs(v) <= FULL_STATE_INT16_LIMIT``；否则抛
    ``ValueError``。不做 clip / clamp —— 静默截断一个真实的数值爆炸比显式拒绝
    更危险。
    """
    try:
        components = [float(vec[0]), float(vec[1]), float(vec[2])]
    except (TypeError, IndexError, ValueError) as exc:
        raise ValueError(
            f"full_state {name} must be length-3 numeric vector, got {vec!r}"
        ) from exc
    for axis, value in zip(("x", "y", "z"), components):
        if not math.isfinite(value):
            raise ValueError(
                f"full_state {name}.{axis} is not finite: {value!r}"
            )
        if abs(value) > FULL_STATE_INT16_LIMIT:
            raise ValueError(
                f"full_state {name}.{axis}={value} exceeds int16 quantization "
                f"range (|v| <= {FULL_STATE_INT16_LIMIT})"
            )


def _poly_coefficients(values) -> list[float]:
    coeffs = [float(value) for value in values]
    if len(coeffs) != 8:
        raise ValueError(f"trajectory polynomial must have 8 coefficients, got {len(coeffs)}")
    return coeffs


def _effective_power_degree(coeffs: list[float], *, eps: float = 1e-9) -> int:
    for degree in range(len(coeffs) - 1, 0, -1):
        if abs(coeffs[degree]) > eps:
            return degree
    return 0


def _bernstein_row(degree: int, u: float) -> list[float]:
    return [
        math.comb(degree, i) * (u ** i) * ((1.0 - u) ** (degree - i))
        for i in range(degree + 1)
    ]


def _power_to_bezier_points(coeffs: list[float], duration: float, degree: int) -> list[float]:
    if degree == 0:
        return [coeffs[0]]
    u_values = [i / degree for i in range(degree + 1)]
    bernstein = np.array([_bernstein_row(degree, u) for u in u_values], dtype=float)
    values = np.array(
        [
            sum(coeffs[k] * ((duration * u) ** k) for k in range(len(coeffs)))
            for u in u_values
        ],
        dtype=float,
    )
    return [float(value) for value in np.linalg.solve(bernstein, values)]


def _compressed_element_for_poly(values, duration: float) -> list[float]:
    coeffs = _poly_coefficients(values)
    power_degree = _effective_power_degree(coeffs)
    if power_degree == 0:
        return []
    if power_degree == 1:
        return [_power_to_bezier_points(coeffs, duration, 1)[1]]
    if power_degree <= 3:
        return _power_to_bezier_points(coeffs, duration, 3)[1:]
    return _power_to_bezier_points(coeffs, duration, 7)[1:]


def _compressed_trajectory_elements(pieces: list) -> list:
    if not pieces:
        return []
    first = pieces[0]
    elements = [
        CompressedStart(
            float(_poly_coefficients(first.x)[0]),
            float(_poly_coefficients(first.y)[0]),
            float(_poly_coefficients(first.z)[0]),
            float(_poly_coefficients(first.yaw)[0]),
        )
    ]
    for piece in pieces:
        duration = float(piece.duration)
        elements.append(
            CompressedSegment(
                duration,
                _compressed_element_for_poly(piece.x, duration),
                _compressed_element_for_poly(piece.y, duration),
                _compressed_element_for_poly(piece.z, duration),
                _compressed_element_for_poly(piece.yaw, duration),
            )
        )
    return elements


def estimate_trajectory_bytes(pieces: list, trajectory_type: str = "poly4d") -> int:
    if trajectory_type == "poly4d_compressed":
        return sum(len(element.pack()) for element in _compressed_trajectory_elements(pieces))
    return len(pieces) * POLY4D_RAW_PIECE_BYTES


def trajectory_define_type(trajectory_type: str) -> int:
    if trajectory_type == "poly4d_compressed":
        return TRAJECTORY_TYPE_POLY4D_COMPRESSED
    if trajectory_type == "poly4d":
        return TRAJECTORY_TYPE_POLY4D
    raise ValueError(f"Unsupported trajectory_type: {trajectory_type}")


class CflibCommandTransport:
    """封装cflib命令，上层不直接调用cflib"""

    def __init__(self, link_manager):
        self.link_manager = link_manager
        self._last_velocity_command_time: dict[int, float] = {}

    @staticmethod
    def _now() -> float:
        # Velocity-stream watchdog compares these timestamps against scheduler /
        # failure-policy cadence windows, so use a monotonic clock rather than
        # wall clock to stay immune to NTP or manual clock adjustments.
        return time.monotonic()

    def _mark_velocity_command(self, drone_id: int) -> None:
        self._last_velocity_command_time[drone_id] = self._now()

    def last_velocity_command_time(self, drone_id: int) -> float | None:
        return self._last_velocity_command_time.get(drone_id)

    def radio_group(self, drone_id: int) -> int | None:
        fleet = getattr(self.link_manager, "fleet", None)
        if fleet is None or not hasattr(fleet, "get_radio_group"):
            return None
        return fleet.get_radio_group(drone_id)

    def classify_command_failure(
        self,
        *,
        drone_id: int,
        command_kind: str,
        exception: Exception,
    ) -> dict[str, object]:
        error_type = type(exception).__name__
        message = str(exception)
        lowered = message.lower()

        retryable = True
        if isinstance(exception, (KeyError, ValueError, AttributeError)):
            retryable = False
        if isinstance(exception, RuntimeError) and (
            "trajectory too large" in lowered
            or "no trajectory memory" in lowered
            or "unsupported" in lowered
        ):
            retryable = False

        category = "transport_runtime"
        if isinstance(exception, KeyError):
            category = "link_lookup"
        elif isinstance(exception, TimeoutError):
            category = "timeout"
        elif isinstance(exception, ValueError):
            category = "invalid_command"
        elif isinstance(exception, AttributeError):
            category = "transport_api"
        elif isinstance(exception, RuntimeError):
            category = "transport_runtime"

        return {
            "drone_id": drone_id,
            "radio_group": self.radio_group(drone_id),
            "command_kind": command_kind,
            "error": message,
            "error_type": error_type,
            "failure_category": category,
            "retryable": retryable,
        }

    def hl_takeoff(self, drone_id: int, height: float, duration: float):
        """高层起飞"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.takeoff(height, duration)
        logger.info("Drone %s takeoff to %sm", drone_id, height)

    def hl_land(self, drone_id: int, height: float, duration: float):
        """高层降落"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.land(height, duration)
        logger.info("Drone %s landing", drone_id)

    def hl_go_to(self, drone_id: int, x: float, y: float, z: float, duration: float):
        """高层go_to"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.go_to(x, y, z, 0, duration)
        logger.debug(
            "Drone %s go_to (%.2f, %.2f, %.2f)", drone_id, x, y, z
        )

    def cmd_velocity_world(self, drone_id: int, vx: float, vy: float, vz: float):
        """速度命令（世界坐标系）"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)
        self._mark_velocity_command(drone_id)

    def cmd_full_state(
        self,
        drone_id: int,
        pos: "tuple[float, float, float]",
        vel: "tuple[float, float, float]",
        acc: "tuple[float, float, float]",
    ):
        """Full-state setpoint：pos + vel + acc，onboard Mellinger/INDI 做闭环。

        姿态固定为 identity quaternion，角速度喂 0，让 onboard 自主解算。

        会在下发前 reject NaN/Inf 与任意分量 |v| > ``FULL_STATE_INT16_LIMIT`` 的
        输入。cflib 的 `send_full_state_setpoint` 会把三组向量量化成 int16 mm/
        mm·s/mm·s^2，超界时 `struct.pack('<h', ...)` 会抛 ``struct.error``，放任
        下去会把整组 follower 拖进 ``executor_group_hold``，所以在此主动拦截并
        `raise ValueError` 让上层 `classify_command_failure` 归入
        `invalid_command` / `retryable=false`。
        """
        _validate_full_state_vector("pos", pos)
        _validate_full_state_vector("vel", vel)
        _validate_full_state_vector("acc", acc)
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_full_state_setpoint(
            list(pos),
            list(vel),
            list(acc),
            [0.0, 0.0, 0.0, 1.0],  # quaternion (x,y,z,w) identity
            0.0,                    # rollrate deg/s
            0.0,                    # pitchrate deg/s
            0.0,                    # yawrate deg/s
        )
        self._mark_velocity_command(drone_id)

    def set_onboard_controller(self, drone_id: int, controller: str) -> None:
        """切换 onboard 飞控控制器：pid(1) / mellinger(2) / indi(3)。

        参数通过 cflib param port 一次性写入；真机现场实测 Mellinger 对
        pos+vel+acc full-state 的跟踪质量最佳，但对 EKF 不稳很敏感。
        """
        mapping = {"pid": 1, "mellinger": 2, "indi": 3}
        if controller not in mapping:
            raise ValueError(f"Unsupported onboard controller: {controller}")
        scf = self.link_manager.get(drone_id)
        scf.cf.param.set_value("stabilizer.controller", str(mapping[controller]))

    def notify_setpoint_stop(self, drone_id: int):
        """通知停止低层setpoint流，恢复高层控制权限"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_notify_setpoint_stop()

    def stop_high_level_commander(self, drone_id: int):
        """停止 high-level commander，避免切入 streaming setpoint 时模式残留。"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.stop()

    def hl_define_trajectory(
        self,
        drone_id: int,
        trajectory_id: int,
        offset: int,
        n_pieces: int,
        trajectory_type: str = "poly4d",
    ):
        """定义已上传轨迹"""
        if n_pieces > TRAJECTORY_MAX_PIECES:
            raise RuntimeError(
                f"Trajectory has too many pieces for drone {drone_id}: "
                f"{n_pieces} > {TRAJECTORY_MAX_PIECES}"
            )
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.define_trajectory(
            trajectory_id,
            offset,
            n_pieces,
            type=trajectory_define_type(trajectory_type),
        )
        logger.info("Drone %s define trajectory %s", drone_id, trajectory_id)

    def hl_start_trajectory(
        self,
        drone_id: int,
        trajectory_id: int,
        time_scale: float = 1.0,
        relative_position: bool = False,
        relative_yaw: bool = False,
        reversed: bool = False,
    ):
        """启动已定义轨迹"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.start_trajectory(
            trajectory_id,
            time_scale=time_scale,
            relative_position=relative_position,
            relative_yaw=relative_yaw,
            reversed=reversed,
        )
        logger.info("Drone %s start trajectory %s", drone_id, trajectory_id)

    def wait_for_params(self, drone_id: int):
        """等待参数更新完成"""
        scf = self.link_manager.get(drone_id)
        scf.wait_for_params()

    def upload_trajectory(
        self,
        drone_id: int,
        pieces: list,
        start_addr: int = 0,
        trajectory_type: str = "poly4d",
    ) -> int:
        """上传 trajectory content 到内存，返回 piece 数量"""
        scf = self.link_manager.get(drone_id)
        trajectory_mems = scf.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)
        if not trajectory_mems:
            raise RuntimeError(f"Drone {drone_id} has no trajectory memory")

        if len(pieces) > TRAJECTORY_MAX_PIECES:
            raise RuntimeError(
                f"Trajectory has too many pieces for drone {drone_id}: "
                f"{len(pieces)} > {TRAJECTORY_MAX_PIECES}"
            )
        estimated_bytes = estimate_trajectory_bytes(pieces, trajectory_type)
        if start_addr + estimated_bytes > TRAJECTORY_MEMORY_BYTES:
            raise RuntimeError(
                "Trajectory too large for drone "
                f"{drone_id}: {len(pieces)} pieces, estimated {estimated_bytes} bytes, "
                f"start_addr={start_addr}, capacity={TRAJECTORY_MEMORY_BYTES} bytes"
            )

        trajectory_mem = trajectory_mems[0]
        if trajectory_type == "poly4d_compressed":
            trajectory_mem.trajectory = _compressed_trajectory_elements(pieces)
        else:
            trajectory_mem.trajectory = []
            for piece in pieces:
                x = Poly4D.Poly(piece.x)
                y = Poly4D.Poly(piece.y)
                z = Poly4D.Poly(piece.z)
                yaw = Poly4D.Poly(piece.yaw)
                trajectory_mem.trajectory.append(Poly4D(piece.duration, x, y, z, yaw))

        success = trajectory_mem.write_data_sync(start_addr=start_addr)
        if not success:
            raise RuntimeError(f"Failed to upload trajectory for drone {drone_id}")

        logger.info(
            "Drone %s uploaded trajectory with %d pieces",
            drone_id,
            len(pieces),
        )
        return len(pieces)

    def upload_trajectories_by_group(
        self,
        uploads: dict[int, dict[str, object]],
        *,
        parallel_groups: bool = False,
    ) -> dict[int, dict[str, object]]:
        grouped: dict[int, list[tuple[int, dict[str, object]]]] = {}
        for drone_id, spec in uploads.items():
            group_id = self.radio_group(drone_id)
            grouped.setdefault(-1 if group_id is None else int(group_id), []).append(
                (drone_id, spec)
            )

        def run_group(items: list[tuple[int, dict[str, object]]]) -> dict[int, dict[str, object]]:
            results: dict[int, dict[str, object]] = {}
            for grouped_drone_id, grouped_spec in items:
                pieces = grouped_spec.get("pieces", [])
                start_addr = int(grouped_spec.get("start_addr", 0))
                trajectory_id = int(grouped_spec.get("trajectory_id", 1))
                trajectory_type = str(grouped_spec.get("trajectory_type", "poly4d"))
                piece_count = self.upload_trajectory(
                    grouped_drone_id,
                    pieces,
                    start_addr=start_addr,
                    trajectory_type=trajectory_type,
                )
                self.hl_define_trajectory(
                    grouped_drone_id,
                    trajectory_id,
                    start_addr,
                    piece_count,
                    trajectory_type=trajectory_type,
                )
                results[grouped_drone_id] = {
                    "piece_count": piece_count,
                    "trajectory_id": trajectory_id,
                    "start_addr": start_addr,
                    "trajectory_type": trajectory_type,
                    "estimated_bytes": estimate_trajectory_bytes(pieces, trajectory_type),
                }
            return results

        if not parallel_groups:
            sequential_results: dict[int, dict[str, object]] = {}
            for group_id in sorted(grouped):
                sequential_results.update(run_group(grouped[group_id]))
            return sequential_results

        results: dict[int, dict[str, object]] = {}
        with ThreadPoolExecutor(max_workers=len(grouped) or 1) as executor:
            futures = [executor.submit(run_group, grouped[group_id]) for group_id in sorted(grouped)]
            for future in as_completed(futures):
                results.update(future.result())
        return results

    def reset_estimator_and_wait(self, drone_id: int):
        """重置 Kalman estimator 并等待稳定"""
        scf = self.link_manager.get(drone_id)
        reset_estimator(scf)

    def read_health_snapshot(self, drone_id: int) -> dict:
        """读取最新健康快照"""
        scf = self.link_manager.get(drone_id)
        return {
            "params_updated": scf.is_params_updated(),
            "link_open": scf.is_link_open(),
        }
