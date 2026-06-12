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
        self.full_state_position_smoothing_alpha = float(
            getattr(config, "full_state_position_smoothing_alpha", 1.0)
        )
        self.full_state_max_position_step = float(
            getattr(config, "full_state_max_position_step", float("inf"))
        )
        self._last_full_state_targets: dict[int, np.ndarray] = {}
        self._last_full_state_velocities: dict[int, np.ndarray] = {}
        self._last_full_state_t: dict[int, float] = {}

    def reset_full_state_state(self, follower_ids: list[int] | set[int] | None = None) -> None:
        """Clear committed full-state smoothing state.

        Used when a follower is held or re-enters the stream so the next
        reference starts from the measured pose instead of a stale internal
        target.
        """

        if follower_ids is None:
            self._last_full_state_targets.clear()
            self._last_full_state_velocities.clear()
            self._last_full_state_t.clear()
            return
        for fid in follower_ids:
            self._last_full_state_targets.pop(int(fid), None)
            self._last_full_state_velocities.pop(int(fid), None)
            self._last_full_state_t.pop(int(fid), None)

    def commit_full_state_state(
        self,
        command_set: FollowerCommandSet,
        follower_ids: list[int] | set[int] | None = None,
    ) -> None:
        """Commit only full-state references that were actually sent."""

        pending = command_set.full_state_state or {}
        allowed = None if follower_ids is None else {int(fid) for fid in follower_ids}
        for fid, state in pending.items():
            fid_int = int(fid)
            if allowed is not None and fid_int not in allowed:
                continue
            self._last_full_state_targets[fid_int] = np.asarray(
                state["target_position"], dtype=float
            ).copy()
            self._last_full_state_velocities[fid_int] = np.asarray(
                state["target_velocity"], dtype=float
            ).copy()
            self._last_full_state_t[fid_int] = float(state["t_meas"])

    def _estimate_current_velocity(
        self,
        fid: int,
        position: np.ndarray,
        t_meas: float,
        onboard_velocity: np.ndarray | None = None,
    ) -> np.ndarray:
        if onboard_velocity is not None:
            return np.asarray(onboard_velocity, dtype=float)
        previous = self._state_estimates.get(fid)
        if previous is None:
            return np.zeros(3, dtype=float)
        dt = float(t_meas) - previous.t_meas
        # 防止除零：dt 过小时返回上次速度
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
        if self.output_mode == "full_state":
            return self._compute_full_state(
                snapshot, references, active_follower_ids, fleet_model
            )
        return self._compute_velocity(
            snapshot, references, active_follower_ids, fleet_model
        )

    def _compute_full_state(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        """full_state 模式：把 (pos, vel, acc) reference 透传给 onboard Mellinger。

        host 侧不再做 PD / 积分闭环；只保留参考平滑与速度/加速度限幅，
        闭环逻辑在飞控完成。
        ``commands`` 里保留 ref velocity 仅供 scheduler 的 deadband 与 diagnostics
        使用；真正下发的是 full_state action 里的 pos+vel+acc。
        """
        commands: dict[int, np.ndarray] = {}
        target_positions: dict[int, np.ndarray] = {}
        target_accelerations: dict[int, np.ndarray] = {}
        full_state_state: dict[int, dict[str, np.ndarray | float]] = {}
        skipped_stale: list[int] = []
        missing_reference: list[int] = []
        derived_feedforward_applied: list[int] = []
        derived_acceleration_applied: list[int] = []
        raw_feedforward_ignored: list[int] = []
        raw_acceleration_ignored: list[int] = []

        for fid in active_follower_ids:
            if fid not in references.target_positions:
                missing_reference.append(fid)
                self._last_full_state_targets.pop(fid, None)
                self._last_full_state_velocities.pop(fid, None)
                self._last_full_state_t.pop(fid, None)
                continue
            idx = fleet_model.id_to_index(fid)
            if not snapshot.fresh_mask[idx]:
                skipped_stale.append(fid)
                self._last_full_state_targets.pop(fid, None)
                self._last_full_state_velocities.pop(fid, None)
                self._last_full_state_t.pop(fid, None)
                continue

            p_current = np.array(snapshot.positions[idx], dtype=float)
            p_target_raw = np.array(references.target_positions[fid], dtype=float)
            previous_target = self._last_full_state_targets.get(fid)
            previous_velocity = self._last_full_state_velocities.get(fid)
            previous_t = self._last_full_state_t.get(fid)
            p_target = self._smooth_full_state_position(
                fid, p_target_raw, current_position=p_current
            )
            raw_v = None
            if references.target_velocities is not None:
                raw_v = self._finite_vector(references.target_velocities.get(fid))
                if raw_v is not None:
                    raw_feedforward_ignored.append(fid)
            raw_a = None
            if references.target_accelerations is not None:
                raw_a = self._finite_vector(references.target_accelerations.get(fid))
                if raw_a is not None:
                    raw_acceleration_ignored.append(fid)

            fallback_v = np.zeros(3, dtype=float)
            fallback_a = np.zeros(3, dtype=float)
            current_t = float(snapshot.t_meas)
            if previous_target is not None and previous_t is not None:
                dt = current_t - previous_t
                # 防止除零
                if dt > 1e-6:
                    fallback_v = self._clip_vector_norm(
                        (p_target - previous_target) / dt,
                        self.max_velocity,
                    )
                    if previous_velocity is not None:
                        # 防止除零
                        fallback_a = self._clip_vector_norm(
                            (fallback_v - previous_velocity) / dt,
                            self.max_acceleration,
                        )
                    if np.linalg.norm(fallback_v) > 1e-12:
                        derived_feedforward_applied.append(fid)
                    if np.linalg.norm(fallback_a) > 1e-12:
                        derived_acceleration_applied.append(fid)

            v_target = fallback_v
            a_target = fallback_a

            target_positions[fid] = p_target
            target_accelerations[fid] = a_target
            commands[fid] = v_target
            full_state_state[fid] = {
                "target_position": p_target.copy(),
                "target_velocity": v_target.copy(),
                "t_meas": current_t,
            }

        active_set = set(active_follower_ids)
        for fid in list(self._last_full_state_targets):
            if fid not in active_set:
                self._last_full_state_targets.pop(fid, None)
                self._last_full_state_velocities.pop(fid, None)
                self._last_full_state_t.pop(fid, None)

        command_norms = {
            fid: float(np.linalg.norm(v)) for fid, v in commands.items()
        }
        return FollowerCommandSet(
            commands=commands,
            diagnostics={
                "output_mode": "full_state",
                "skipped_stale_followers": skipped_stale,
                "missing_reference_followers": missing_reference,
                "feedforward_followers": derived_feedforward_applied,
                "acceleration_feedforward_followers": derived_acceleration_applied,
                "derived_feedforward_followers": derived_feedforward_applied,
                "derived_acceleration_followers": derived_acceleration_applied,
                "raw_feedforward_ignored_followers": raw_feedforward_ignored,
                "raw_acceleration_ignored_followers": raw_acceleration_ignored,
                "feedforward_suppressed_followers": [],
                "acceleration_feedforward_suppressed_followers": [],
                "radial_scaled_followers": [],
                "commanded_accelerations": {},
                "command_norms": command_norms,
                "commanded_acceleration_norms": {},
            },
            target_positions=target_positions,
            target_accelerations=target_accelerations,
            full_state_state=full_state_state,
        )

    @staticmethod
    def _finite_vector(value) -> np.ndarray | None:
        if value is None:
            return None
        try:
            vector = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return None
        if vector.shape != (3,) or not np.isfinite(vector).all():
            return None
        return vector

    def _smooth_full_state_position(
        self,
        fid: int,
        target: np.ndarray,
        *,
        current_position: np.ndarray | None = None,
    ) -> np.ndarray:
        previous = self._last_full_state_targets.get(fid)
        if previous is None:
            if current_position is None:
                previous = np.array(target, dtype=float)
            else:
                previous = np.array(current_position, dtype=float)
            smoothed = previous + (target - previous)
        else:
            alpha = min(max(self.full_state_position_smoothing_alpha, 1e-6), 1.0)
            smoothed = previous + alpha * (target - previous)
        delta = smoothed - previous
        norm = float(np.linalg.norm(delta))
        if norm > self.full_state_max_position_step:
            smoothed = previous + delta / norm * self.full_state_max_position_step
        return smoothed

    @staticmethod
    def _clip_vector_norm(value, limit: float) -> np.ndarray:
        vector = np.array(value, dtype=float)
        norm = float(np.linalg.norm(vector))
        if limit > 0 and norm > limit:
            return vector / norm * limit
        return vector

    def _compute_velocity(
        self,
        snapshot: PoseSnapshot,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
        fleet_model,
    ) -> FollowerCommandSet:
        active_follower_set = set(active_follower_ids)
        for fid in list(self._state_estimates):
            if fid not in active_follower_set:
                self._state_estimates.pop(fid, None)

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
                self._state_estimates.pop(fid, None)
                continue

            try:
                idx = fleet_model.id_to_index(fid)
            except (KeyError, ValueError, IndexError) as exc:
                logger.warning("Invalid follower ID %d in velocity mode: %s", fid, exc)
                discarded_reasons[fid] = f"invalid_id:{exc}"
                continue
            if not snapshot.fresh_mask[idx]:
                skipped_stale.append(fid)
                self._state_estimates.pop(fid, None)
                continue

            p_current = np.array(snapshot.positions[idx], dtype=float)
            p_target = np.array(references.target_positions[fid], dtype=float)
            onboard_vel = None
            if snapshot.velocities is not None and snapshot.velocity_fresh_mask is not None:
                if snapshot.velocity_fresh_mask[idx]:
                    onboard_vel = snapshot.velocities[idx]
            v_current = self._estimate_current_velocity(
                fid, p_current, snapshot.t_meas, onboard_vel
            )
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
                v_current.copy()
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
