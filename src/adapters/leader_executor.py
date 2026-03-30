"""Leader执行器 - 只执行leader命令"""

import logging
from ..runtime.command_plan import LeaderAction

logger = logging.getLogger(__name__)


class LeaderExecutor:
    """Leader命令执行器"""

    def __init__(self, transport):
        self.transport = transport

    def execute(self, actions: list[LeaderAction]):
        """执行leader动作列表"""
        for action in actions:
            if action.kind == "takeoff":
                self._execute_takeoff(action)
            elif action.kind == "batch_goto":
                self._execute_batch_goto(action)
            elif action.kind == "start_trajectory":
                self._execute_start_trajectory(action)
            elif action.kind == "land":
                self._execute_land(action)

    def _execute_takeoff(self, action: LeaderAction):
        """执行起飞"""
        height = action.payload.get("height", 0.5)
        duration = action.payload.get("duration", 2.0)
        for drone_id in action.drone_ids:
            self.transport.hl_takeoff(drone_id, height, duration)

    def _execute_batch_goto(self, action: LeaderAction):
        """批量go_to（共时更新）"""
        positions = action.payload["positions"]  # {drone_id: [x,y,z]}
        duration = action.payload.get("duration", 1.0)
        for drone_id in action.drone_ids:
            pos = positions[drone_id]
            self.transport.hl_go_to(drone_id, pos[0], pos[1], pos[2], duration)

    def _execute_land(self, action: LeaderAction):
        """执行降落"""
        duration = action.payload.get("duration", 2.0)
        for drone_id in action.drone_ids:
            self.transport.hl_land(drone_id, 0.0, duration)

    def _execute_start_trajectory(self, action: LeaderAction):
        trajectory_id = action.payload.get("trajectory_id", 1)
        time_scale = action.payload.get("time_scale", 1.0)
        relative_position = action.payload.get("relative_position", False)
        relative_yaw = action.payload.get("relative_yaw", False)
        reversed_mode = action.payload.get("reversed", False)
        for drone_id in action.drone_ids:
            self.transport.hl_start_trajectory(
                drone_id,
                trajectory_id,
                time_scale=time_scale,
                relative_position=relative_position,
                relative_yaw=relative_yaw,
                reversed=reversed_mode,
            )
