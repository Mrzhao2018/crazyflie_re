"""Follower 二阶控制器 - 内部加速度状态，输出速度命令"""

from dataclasses import dataclass

import numpy as np

from .pose_snapshot import PoseSnapshot
from ..config.schema import ControlConfig
from ..domain.follower_reference import FollowerReferenceSet
from .follower_controller import FollowerCommandSet
from .follower_controller_base import FollowerControllerBase


@dataclass
class _FollowerStateEstimate:
    position: np.ndarray
    velocity: np.ndarray
    t_meas: float


class FollowerControllerV2(FollowerControllerBase):
    """二阶 follower 控制器。

    第一版保持最小侵入：
    - 使用位置误差 + 速度误差 + 加速度前馈计算加速度命令
    - 在控制器内部离散积分为 velocity setpoint
    - 不改变下游 scheduler/executor/transport 的 velocity output 语义
    """

    def __init__(self, config: ControlConfig):
        super().__init__(config)
        self.max_acceleration = config.max_acceleration
        self.velocity_feedback_gain = config.velocity_feedback_gain
        self.acceleration_feedforward_gain = config.acceleration_feedforward_gain
        self.damping_coeff = config.damping_coeff
        self._state_estimates: dict[int, _FollowerStateEstimate] = {}

    def _estimate_current_velocity(
        self,
        fid: int,
        position: np.ndarray,
        t_meas: float,
    ) -> np.ndarray:
        previous = self._state_estimates.get(fid)
        if previous is None:
            return np.zeros(3, dtype=float)
        dt = float(t_meas) - previous.t_meas
        if dt <= 1e-9:
            return previous.velocity.copy()
        return (position - previous.position) / dt

    def compute(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        commands: dict[int, np.ndarray] = {}
        skipped_stale: list[int] = []
        missing_reference: list[int] = []
        feedforward_applied: list[int] = []
        acceleration_feedforward_applied: list[int] = []
        commanded_accelerations: dict[int, np.ndarray] = {}

        radial_scales, radial_scaled_followers = self._compute_radial_scales(
            references, active_follower_ids
        )

        for fid in active_follower_ids:
            if fid not in references.target_positions:
                missing_reference.append(fid)
                continue

            idx = fleet_model.id_to_index(fid)
            if not snapshot.fresh_mask[idx]:
                skipped_stale.append(fid)
                continue

            p_current = np.array(snapshot.positions[idx], dtype=float)
            p_target = np.array(references.target_positions[fid], dtype=float)
            v_current = self._estimate_current_velocity(fid, p_current, snapshot.t_meas)
            gain_scale_xy, ff_scale_xy = radial_scales.get(fid, (1.0, 1.0))

            position_error = p_current - p_target
            target_velocity = None
            if references.target_velocities is not None:
                raw_target_velocity = references.target_velocities.get(fid)
                if raw_target_velocity is not None:
                    target_velocity = self._clip_feedforward_velocity(raw_target_velocity)
                    feedforward_applied.append(fid)
            target_acceleration = None
            if references.target_accelerations is not None:
                raw_target_acceleration = references.target_accelerations.get(fid)
                if raw_target_acceleration is not None:
                    target_acceleration = np.array(raw_target_acceleration, dtype=float)
                    acceleration_feedforward_applied.append(fid)

            velocity_error = (
                v_current - target_velocity
                if target_velocity is not None
                else v_current
            )
            commanded_acceleration = np.array(
                [
                    -(self.gain_xy * gain_scale_xy) * position_error[0],
                    -(self.gain_xy * gain_scale_xy) * position_error[1],
                    -self.gain_z * position_error[2],
                ],
                dtype=float,
            )
            commanded_acceleration = commanded_acceleration - (
                self.velocity_feedback_gain * velocity_error
            )
            commanded_acceleration = commanded_acceleration - (
                self.damping_coeff * v_current
            )

            if target_acceleration is not None:
                commanded_acceleration = commanded_acceleration + (
                    self.acceleration_feedforward_gain * target_acceleration
                )

            accel_norm = np.linalg.norm(commanded_acceleration)
            if accel_norm > self.max_acceleration:
                commanded_acceleration = (
                    commanded_acceleration / accel_norm * self.max_acceleration
                )
            commanded_accelerations[fid] = commanded_acceleration.copy()

            previous = self._state_estimates.get(fid)
            previous_velocity = (
                previous.velocity.copy()
                if previous is not None
                else np.zeros(3, dtype=float)
            )
            dt = (
                max(float(snapshot.t_meas) - previous.t_meas, 0.0)
                if previous is not None
                else 0.0
            )
            velocity = previous_velocity + commanded_acceleration * dt

            if target_velocity is not None:
                velocity = velocity + np.array(
                    [
                        self.feedforward_gain_xy * ff_scale_xy * target_velocity[0],
                        self.feedforward_gain_xy * ff_scale_xy * target_velocity[1],
                        self.feedforward_gain_z * target_velocity[2],
                    ],
                    dtype=float,
                )

            velocity = self._clip_output_velocity(velocity)

            commands[fid] = velocity.copy()
            self._state_estimates[fid] = _FollowerStateEstimate(
                position=p_current.copy(),
                velocity=velocity.copy(),
                t_meas=float(snapshot.t_meas),
            )

        command_norms = {
            fid: float(np.linalg.norm(velocity))
            for fid, velocity in commands.items()
        }
        commanded_acceleration_norms = {
            fid: float(np.linalg.norm(acc))
            for fid, acc in commanded_accelerations.items()
        }

        return FollowerCommandSet(
            commands=commands,
            diagnostics={
                "skipped_stale_followers": skipped_stale,
                "missing_reference_followers": missing_reference,
                "feedforward_followers": feedforward_applied,
                "acceleration_feedforward_followers": acceleration_feedforward_applied,
                "radial_scaled_followers": radial_scaled_followers,
                "commanded_accelerations": commanded_accelerations,
                "command_norms": command_norms,
                "commanded_acceleration_norms": commanded_acceleration_norms,
            },
        )
