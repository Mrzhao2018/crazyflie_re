"""Cflib命令传输层 - 封装cflib命令调用"""

from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from cflib.crazyflie.mem import MemoryElement, Poly4D
from cflib.utils.reset_estimator import reset_estimator

logger = logging.getLogger(__name__)


POLY4D_RAW_PIECE_BYTES = 132
TRAJECTORY_MEMORY_BYTES = 4096


class CflibCommandTransport:
    """封装cflib命令，上层不直接调用cflib"""

    def __init__(self, link_manager):
        self.link_manager = link_manager
        self._last_velocity_command_time: dict[int, float] = {}
        self._last_high_level_command_time: dict[int, float] = {}

    @staticmethod
    def _now() -> float:
        return time.time()

    def _mark_velocity_command(self, drone_id: int) -> None:
        self._last_velocity_command_time[drone_id] = self._now()

    def _mark_high_level_command(self, drone_id: int) -> None:
        self._last_high_level_command_time[drone_id] = self._now()

    def last_velocity_command_time(self, drone_id: int) -> float | None:
        return self._last_velocity_command_time.get(drone_id)

    def last_high_level_command_time(self, drone_id: int) -> float | None:
        return self._last_high_level_command_time.get(drone_id)

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
        self._mark_high_level_command(drone_id)
        logger.info(f"Drone {drone_id} takeoff to {height}m")

    def hl_land(self, drone_id: int, height: float, duration: float):
        """高层降落"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.land(height, duration)
        self._mark_high_level_command(drone_id)
        logger.info(f"Drone {drone_id} landing")

    def hl_go_to(self, drone_id: int, x: float, y: float, z: float, duration: float):
        """高层go_to"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.go_to(x, y, z, 0, duration)
        self._mark_high_level_command(drone_id)
        logger.debug(f"Drone {drone_id} go_to ({x:.2f}, {y:.2f}, {z:.2f})")

    def cmd_velocity_world(self, drone_id: int, vx: float, vy: float, vz: float):
        """速度命令（世界坐标系）"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)
        self._mark_velocity_command(drone_id)

    def notify_setpoint_stop(self, drone_id: int):
        """通知停止低层setpoint流，恢复高层控制权限"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_notify_setpoint_stop()
        self._mark_high_level_command(drone_id)

    def hl_define_trajectory(
        self, drone_id: int, trajectory_id: int, offset: int, n_pieces: int
    ):
        """定义已上传轨迹"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.define_trajectory(trajectory_id, offset, n_pieces)
        self._mark_high_level_command(drone_id)
        logger.info(f"Drone {drone_id} define trajectory {trajectory_id}")

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
        self._mark_high_level_command(drone_id)
        logger.info(f"Drone {drone_id} start trajectory {trajectory_id}")

    def wait_for_params(self, drone_id: int):
        """等待参数更新完成"""
        scf = self.link_manager.get(drone_id)
        scf.wait_for_params()

    def upload_trajectory(
        self, drone_id: int, pieces: list, start_addr: int = 0
    ) -> int:
        """上传 trajectory content 到内存，返回 piece 数量"""
        scf = self.link_manager.get(drone_id)
        trajectory_mems = scf.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)
        if not trajectory_mems:
            raise RuntimeError(f"Drone {drone_id} has no trajectory memory")

        estimated_bytes = len(pieces) * POLY4D_RAW_PIECE_BYTES
        if start_addr + estimated_bytes > TRAJECTORY_MEMORY_BYTES:
            raise RuntimeError(
                "Trajectory too large for drone "
                f"{drone_id}: {len(pieces)} pieces, estimated {estimated_bytes} bytes, "
                f"start_addr={start_addr}, capacity={TRAJECTORY_MEMORY_BYTES} bytes"
            )

        trajectory_mem = trajectory_mems[0]
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
            f"Drone {drone_id} uploaded trajectory with {len(trajectory_mem.trajectory)} pieces"
        )
        return len(trajectory_mem.trajectory)

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
                piece_count = self.upload_trajectory(
                    grouped_drone_id, pieces, start_addr=start_addr
                )
                self.hl_define_trajectory(
                    grouped_drone_id, trajectory_id, start_addr, piece_count
                )
                results[grouped_drone_id] = {
                    "piece_count": piece_count,
                    "trajectory_id": trajectory_id,
                    "start_addr": start_addr,
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
