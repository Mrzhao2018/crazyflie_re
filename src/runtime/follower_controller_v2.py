"""Follower 二阶控制器 - 内部加速度状态，输出速度命令"""

from dataclasses import dataclass

import numpy as np

from .pose_snapshot import PoseSnapshot
from ..config.schema import ControlConfig
from ..domain.follower_reference import FollowerReferenceSet
from .follower_controller import FollowerCommandSet


@dataclass
class _FollowerStateEstimate:
    position: np.ndarray
    velocity: np.ndarray
    t_meas: float


class FollowerControllerV2:
    """二阶 follower 控制器。

    第一版保持最小侵入：
    - 使用位置误差 + 速度误差 + 加速度前馈计算加速度命令
    - 在控制器内部离散积分为 velocity setpoint
    - 不改变下游 scheduler/executor/transport 的 velocity output 语义
    """

    @staticmethod
    def _resolve_axis_gain(value: float | None, fallback: float) -> float:
        return fallback if value is None else value

    def __init__(self, config: ControlConfig):
        self.gain = config.gain
        self.max_velocity = config.max_velocity
        self.max_acceleration = config.max_acceleration
        self.feedforward_gain = config.feedforward_gain
        self.velocity_feedback_gain = config.velocity_feedback_gain
        self.acceleration_feedforward_gain = config.acceleration_feedforward_gain
        self.damping_coeff = config.damping_coeff
        self.gain_xy = self._resolve_axis_gain(config.gain_xy, config.gain)
        self.gain_z = self._resolve_axis_gain(config.gain_z, config.gain)
        self.feedforward_gain_xy = self._resolve_axis_gain(
            config.feedforward_gain_xy, config.feedforward_gain
        )
        self.feedforward_gain_z = self._resolve_axis_gain(
            config.feedforward_gain_z, config.feedforward_gain
        )
        self.max_feedforward_velocity_xy = self._resolve_axis_gain(
            config.max_feedforward_velocity_xy, config.max_feedforward_velocity
        )
        self.max_feedforward_velocity_z = self._resolve_axis_gain(
            config.max_feedforward_velocity_z, config.max_feedforward_velocity
        )
        self.radial_gain_scale_xy = config.radial_gain_scale_xy
        self.radial_feedforward_scale_xy = config.radial_feedforward_scale_xy
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

    def _clip_xy_and_z(self, velocity: np.ndarray) -> np.ndarray:
        clipped = np.array(velocity, dtype=float)
        xy_norm = np.linalg.norm(clipped[:2])
        if xy_norm > self.max_feedforward_velocity_xy > 0:
            clipped[:2] = clipped[:2] / xy_norm * self.max_feedforward_velocity_xy
        if abs(clipped[2]) > self.max_feedforward_velocity_z > 0:
            clipped[2] = np.sign(clipped[2]) * self.max_feedforward_velocity_z
        return clipped

    def compute(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        commands = {}
        skipped_stale = []
        missing_reference = []
        feedforward_applied = []
        acceleration_feedforward_applied = []
        radial_scaled_followers = []
        commanded_accelerations = {}
        follower_radii = {
            fid: float(
                np.linalg.norm(
                    np.array(references.target_positions[fid], dtype=float)[:2]
                )
            )
            for fid in active_follower_ids
            if fid in references.target_positions
        }
        max_radius = max(follower_radii.values(), default=0.0)

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
            radius_xy = follower_radii.get(fid, 0.0)
            radius_ratio = radius_xy / max_radius if max_radius > 1e-9 else 0.0
            gain_scale_xy = 1.0 + self.radial_gain_scale_xy * radius_ratio
            ff_scale_xy = 1.0 + self.radial_feedforward_scale_xy * radius_ratio
            if gain_scale_xy > 1.0 or ff_scale_xy > 1.0:
                radial_scaled_followers.append(fid)

            position_error = p_current - p_target
            target_velocity = None
            if references.target_velocities is not None:
                raw_target_velocity = references.target_velocities.get(fid)
                if raw_target_velocity is not None:
                    target_velocity = self._clip_xy_and_z(raw_target_velocity)
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
                previous.velocity.copy() if previous is not None else np.zeros(3, dtype=float)
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

            speed = np.linalg.norm(velocity)
            if speed > self.max_velocity:
                velocity = velocity / speed * self.max_velocity

            commands[fid] = velocity.copy()
            self._state_estimates[fid] = _FollowerStateEstimate(
                position=p_current.copy(),
                velocity=velocity.copy(),
                t_meas=float(snapshot.t_meas),
            )

        return FollowerCommandSet(
            commands=commands,
            diagnostics={
                "skipped_stale_followers": skipped_stale,
                "missing_reference_followers": missing_reference,
                "feedforward_followers": feedforward_applied,
                "acceleration_feedforward_followers": acceleration_feedforward_applied,
                "radial_scaled_followers": radial_scaled_followers,
                "commanded_accelerations": commanded_accelerations,
            },
        )