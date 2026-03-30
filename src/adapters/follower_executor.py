"""Follower执行器 - 只执行follower命令"""

import logging
from ..runtime.command_plan import FollowerAction, HoldAction

logger = logging.getLogger(__name__)


class FollowerExecutor:
    """Follower命令执行器"""

    def __init__(self, transport):
        self.transport = transport

    def execute_velocity(self, actions: list[FollowerAction]):
        """执行速度命令"""
        for action in actions:
            vel = action.velocity
            self.transport.cmd_velocity_world(action.drone_id, vel[0], vel[1], vel[2])

    def execute_hold(self, actions: list[HoldAction]):
        """执行hold命令"""
        for action in actions:
            self.transport.cmd_velocity_world(action.drone_id, 0, 0, 0)

    def takeoff(self, drone_ids: list[int], height: float = 0.5, duration: float = 2.0):
        for drone_id in drone_ids:
            self.transport.hl_takeoff(drone_id, height, duration)

    def land(self, drone_ids: list[int], duration: float = 2.0):
        for drone_id in drone_ids:
            self.transport.hl_land(drone_id, 0.0, duration)

    def stop_velocity_mode(self, drone_ids: list[int]):
        for drone_id in drone_ids:
            self.transport.notify_setpoint_stop(drone_id)
