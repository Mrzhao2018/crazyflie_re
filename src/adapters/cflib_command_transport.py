"""Cflib命令传输层 - 封装cflib命令调用"""

import logging
from cflib.crazyflie.mem import MemoryElement, Poly4D
from cflib.utils.reset_estimator import reset_estimator

logger = logging.getLogger(__name__)


class CflibCommandTransport:
    """封装cflib命令，上层不直接调用cflib"""

    def __init__(self, link_manager):
        self.link_manager = link_manager

    def hl_takeoff(self, drone_id: int, height: float, duration: float):
        """高层起飞"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.takeoff(height, duration)
        logger.info(f"Drone {drone_id} takeoff to {height}m")

    def hl_land(self, drone_id: int, height: float, duration: float):
        """高层降落"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.land(height, duration)
        logger.info(f"Drone {drone_id} landing")

    def hl_go_to(self, drone_id: int, x: float, y: float, z: float, duration: float):
        """高层go_to"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.go_to(x, y, z, 0, duration)
        logger.debug(f"Drone {drone_id} go_to ({x:.2f}, {y:.2f}, {z:.2f})")

    def cmd_velocity_world(self, drone_id: int, vx: float, vy: float, vz: float):
        """速度命令（世界坐标系）"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def notify_setpoint_stop(self, drone_id: int):
        """通知停止低层setpoint流，恢复高层控制权限"""
        scf = self.link_manager.get(drone_id)
        scf.cf.commander.send_notify_setpoint_stop()

    def hl_define_trajectory(
        self, drone_id: int, trajectory_id: int, offset: int, n_pieces: int
    ):
        """定义已上传轨迹"""
        scf = self.link_manager.get(drone_id)
        scf.cf.high_level_commander.define_trajectory(trajectory_id, offset, n_pieces)
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
