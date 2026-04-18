"""任务描述层 - 负责给出任务阶段和affine变换"""

from dataclasses import dataclass
import numpy as np

from ..config.schema import MissionConfig, MissionPhaseConfig, LeaderMotionConfig


@dataclass
class AffineTransform:
    A: np.ndarray
    b: np.ndarray


@dataclass
class TrajectoryPiece:
    duration: float
    x: list[float]
    y: list[float]
    z: list[float]
    yaw: list[float]


@dataclass
class MissionPhase:
    name: str
    t_start: float
    t_end: float
    mode: str


class MissionProfile:
    """任务级profile，向上提供阶段与affine参考"""

    def __init__(self, config: MissionConfig):
        self.config = config
        self._phases = [
            MissionPhase(
                name=phase.name,
                t_start=phase.t_start,
                t_end=phase.t_end,
                mode=phase.mode,
            )
            for phase in config.phases
        ]
        self._leader_motion = config.leader_motion

    def total_time(self) -> float:
        return self.config.duration

    def trajectory_enabled(self) -> bool:
        return bool(self._leader_motion.trajectory_enabled)

    def _nominal_leader_positions(self) -> list[np.ndarray]:
        return [
            np.array(self.config.nominal_positions[index], dtype=float)
            for index in (0, 3, 6, 7)
        ]

    def _condition_number_for_transform(self, transform: AffineTransform) -> float:
        leader_positions = [
            transform.A @ leader + transform.b
            for leader in self._nominal_leader_positions()
        ]
        p0 = leader_positions[0]
        diff_matrix = np.array(leader_positions[1:], dtype=float) - p0
        rank = np.linalg.matrix_rank(diff_matrix)
        return float(np.linalg.cond(diff_matrix)) if rank == 3 else float("inf")

    def _stress_transform(self, t: float, transform: AffineTransform) -> AffineTransform:
        if not self._leader_motion.condition_stress_enabled:
            return transform

        axis = self._leader_motion.condition_stress_axis
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        period = float(self._leader_motion.condition_stress_period)
        min_scale = float(self._leader_motion.condition_stress_min_scale)
        oscillation = 0.5 * (1.0 + np.sin((2.0 * np.pi * t) / period))
        axis_scale = min_scale + (1.0 - min_scale) * oscillation
        scale_matrix = np.eye(3)
        scale_matrix[axis_index, axis_index] = axis_scale
        stressed_A = transform.A @ scale_matrix
        return AffineTransform(A=stressed_A, b=transform.b)

    def _apply_condition_penalty(self, transform: AffineTransform) -> AffineTransform:
        if not self._leader_motion.condition_penalty_enabled:
            return transform

        scale = float(self._leader_motion.condition_penalty_scale)
        if scale <= 0.0:
            return transform

        cond = self._condition_number_for_transform(transform)
        soft_limit = float(self._leader_motion.condition_soft_limit)
        if not np.isfinite(cond) or cond <= soft_limit:
            return transform

        u, singular_values, vt = np.linalg.svd(transform.A)
        if singular_values[-1] <= 1e-9:
            return transform

        blend = 1.0 - np.exp(-scale)
        target_cond = soft_limit + (cond - soft_limit) * (1.0 - blend)
        target_cond = max(1.0, float(target_cond))
        min_allowed = singular_values[0] / target_cond
        clipped = np.maximum(singular_values, min_allowed)
        penalized_A = u @ np.diag(clipped) @ vt
        return AffineTransform(A=penalized_A, b=transform.b)

    def trajectory_spec(self) -> dict:
        raw_pieces = self._leader_motion.trajectory_pieces or []
        if self.trajectory_enabled() and not raw_pieces:
            raw_pieces = self._generate_phase_poly4d_pieces()

        return {
            "trajectory_id": self._leader_motion.trajectory_id,
            "time_scale": self._leader_motion.trajectory_time_scale,
            "relative_position": self._leader_motion.trajectory_relative_position,
            "relative_yaw": self._leader_motion.trajectory_relative_yaw,
            "reversed": self._leader_motion.trajectory_reversed,
            "trajectory_type": self._leader_motion.trajectory_type,
            "start_addr": self._leader_motion.trajectory_start_addr,
            "pieces": [
                piece
                if isinstance(piece, TrajectoryPiece)
                else TrajectoryPiece(**piece)
                for piece in raw_pieces
            ],
        }

    def trajectory_spec_for_nominal(self, nominal_position: np.ndarray) -> dict:
        raw_pieces = self._leader_motion.trajectory_pieces or []
        if self.trajectory_enabled() and not raw_pieces:
            raw_pieces = self._generate_phase_poly4d_pieces_for_nominal(
                nominal_position
            )

        return {
            "trajectory_id": self._leader_motion.trajectory_id,
            "time_scale": self._leader_motion.trajectory_time_scale,
            "relative_position": self._leader_motion.trajectory_relative_position,
            "relative_yaw": self._leader_motion.trajectory_relative_yaw,
            "reversed": self._leader_motion.trajectory_reversed,
            "trajectory_type": self._leader_motion.trajectory_type,
            "start_addr": self._leader_motion.trajectory_start_addr,
            "pieces": [
                piece
                if isinstance(piece, TrajectoryPiece)
                else TrajectoryPiece(**piece)
                for piece in raw_pieces
            ],
            "nominal_position": nominal_position.tolist(),
        }

    def _generate_phase_poly4d_pieces(self) -> list[TrajectoryPiece]:
        return self._generate_phase_poly4d_pieces_for_nominal(np.array([1.0, 0.0, 0.0]))

    def _generate_phase_poly4d_pieces_for_nominal(
        self, nominal_position: np.ndarray
    ) -> list[TrajectoryPiece]:
        pieces: list[TrajectoryPiece] = []
        phases = [phase for phase in self._phases if phase.mode == "formation_run"]
        sample_dt = self._leader_motion.trajectory_sample_dt

        for idx, phase in enumerate(phases):
            phase_duration = max(phase.t_end - phase.t_start, 0.1)
            n_segments = max(1, int(np.ceil(phase_duration / sample_dt)))
            segment_dt = phase_duration / n_segments

            for seg in range(n_segments):
                seg_start = phase.t_start + seg * segment_dt
                seg_end_raw = min(phase.t_start + (seg + 1) * segment_dt, phase.t_end)
                seg_end = (
                    seg_end_raw - 1e-6
                    if (idx < len(phases) - 1 or seg < n_segments - 1)
                    else seg_end_raw
                )
                duration = max(seg_end_raw - seg_start, 0.1)

                local_start = seg_start - phase.t_start
                local_end = seg_end - phase.t_start
                start_transform = self._trajectory_transform_at(local_start)
                end_transform = self._trajectory_transform_at(local_end)
                start_velocity = self._trajectory_velocity_at(
                    local_start, nominal_position
                )
                end_velocity = self._trajectory_velocity_at(local_end, nominal_position)

                start_pos = start_transform.A @ nominal_position + start_transform.b
                end_pos = end_transform.A @ nominal_position + end_transform.b

                pieces.append(
                    TrajectoryPiece(
                        duration=duration,
                        x=self._cubic_hermite_coeffs(
                            float(start_pos[0]),
                            float(end_pos[0]),
                            float(start_velocity[0]),
                            float(end_velocity[0]),
                            duration,
                        ),
                        y=self._cubic_hermite_coeffs(
                            float(start_pos[1]),
                            float(end_pos[1]),
                            float(start_velocity[1]),
                            float(end_velocity[1]),
                            duration,
                        ),
                        z=self._cubic_hermite_coeffs(
                            float(start_pos[2]),
                            float(end_pos[2]),
                            float(start_velocity[2]),
                            float(end_velocity[2]),
                            duration,
                        ),
                        yaw=[0.0] * 8,
                    )
                )

        return pieces or [
            TrajectoryPiece(
                duration=0.1,
                x=[0.0] * 8,
                y=[0.0] * 8,
                z=[0.0] * 8,
                yaw=[0.0] * 8,
            )
        ]

    def trajectory_quality_summary(self) -> dict:
        if not self.trajectory_enabled():
            return {
                "condition_penalty_enabled": False,
                "condition_soft_limit": self._leader_motion.condition_soft_limit,
                "condition_penalty_scale": self._leader_motion.condition_penalty_scale,
                "condition_stress_enabled": self._leader_motion.condition_stress_enabled,
                "condition_number_max": None,
                "condition_number_min": None,
                "raw_condition_number_max": None,
                "raw_condition_number_min": None,
                "penalized_samples": 0,
                "penalty_active": False,
            }

        samples = self._trajectory_condition_samples()
        condition_numbers = [item["condition_number"] for item in samples]
        raw_condition_numbers = [item["raw_condition_number"] for item in samples]
        penalized_samples = [
            item for item in samples if item["penalized"]
        ]
        return {
            "condition_penalty_enabled": self._leader_motion.condition_penalty_enabled,
            "condition_soft_limit": self._leader_motion.condition_soft_limit,
            "condition_penalty_scale": self._leader_motion.condition_penalty_scale,
            "condition_stress_enabled": self._leader_motion.condition_stress_enabled,
            "condition_number_max": max(condition_numbers) if condition_numbers else None,
            "condition_number_min": min(condition_numbers) if condition_numbers else None,
            "raw_condition_number_max": max(raw_condition_numbers) if raw_condition_numbers else None,
            "raw_condition_number_min": min(raw_condition_numbers) if raw_condition_numbers else None,
            "penalized_samples": len(penalized_samples),
            "penalty_active": bool(penalized_samples),
        }

    def _trajectory_condition_samples(self) -> list[dict]:
        formation_run_phases = [phase for phase in self._phases if phase.mode == "formation_run"]
        if not formation_run_phases:
            return []

        sample_dt = min(self._leader_motion.trajectory_sample_dt, 1.0)
        samples = []
        for phase in formation_run_phases:
            t = phase.t_start
            while t <= phase.t_end + 1e-9:
                local_t = t - phase.t_start
                base_transform = self._trajectory_transform_at(local_t)
                stressed_transform = self._stress_transform(t, base_transform)
                raw_cond = self._condition_number_for_transform(stressed_transform)
                transform = self._apply_condition_penalty(stressed_transform)
                cond = self._condition_number_for_transform(transform)
                penalized = (
                    self._leader_motion.condition_penalty_enabled
                    and cond < raw_cond - 1e-9
                )
                samples.append(
                    {
                        "time": float(t),
                        "raw_condition_number": float(raw_cond),
                        "condition_number": float(cond),
                        "penalized": penalized,
                    }
                )
                t += sample_dt
        return samples

    def trajectory_start_time(self) -> float:
        for phase in self._phases:
            if phase.mode == "formation_run":
                return phase.t_start
        return 0.0

    @staticmethod
    def _linear_coeffs(displacement: float, duration: float) -> list[float]:
        velocity = displacement / duration
        return [0.0, velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    @staticmethod
    def _cubic_hermite_coeffs(
        p0: float, p1: float, v0: float, v1: float, duration: float
    ) -> list[float]:
        T = duration
        a0 = p0
        a1 = v0
        a2 = (3 * (p1 - p0) / (T**2)) - (2 * v0 + v1) / T
        a3 = (-2 * (p1 - p0) / (T**3)) + (v0 + v1) / (T**2)
        return [a0, a1, a2, a3, 0.0, 0.0, 0.0, 0.0]

    def affine_velocity_at(self, t: float, nominal_position: np.ndarray) -> np.ndarray:
        return self._trajectory_velocity_at(t, nominal_position)

    def _trajectory_transform_at(self, t: float) -> AffineTransform:
        if self._leader_motion.mode == "affine_rotation":
            base_transform = self._rotation_transform(t, self._leader_motion)
            stressed_transform = self._stress_transform(t, base_transform)
            return self._apply_condition_penalty(stressed_transform)
        if self._leader_motion.mode == "hold":
            return AffineTransform(A=np.eye(3), b=np.zeros(3))
        raise ValueError(f"Unsupported leader motion mode: {self._leader_motion.mode}")

    def _trajectory_velocity_at(
        self, t: float, nominal_position: np.ndarray
    ) -> np.ndarray:
        epsilon = min(max(self._leader_motion.trajectory_sample_dt * 0.05, 1e-3), 0.05)
        t0 = max(0.0, t - epsilon)
        t1 = t + epsilon
        p0 = self._trajectory_transform_at(t0).A @ nominal_position + self._trajectory_transform_at(t0).b
        p1 = self._trajectory_transform_at(t1).A @ nominal_position + self._trajectory_transform_at(t1).b
        dt = max(t1 - t0, 1e-9)
        return (p1 - p0) / dt

    def phase_at(self, t: float) -> MissionPhase:
        for phase in self._phases:
            if phase.t_start <= t < phase.t_end:
                return phase
        return self._phases[-1]

    def affine_transform_at(self, t: float) -> AffineTransform:
        phase = self.phase_at(t)

        if (
            phase.mode in {"settle", "trajectory_entry", "run_entry"}
            or self._leader_motion.mode == "hold"
        ):
            return AffineTransform(A=np.eye(3), b=np.zeros(3))

        if self._leader_motion.mode == "affine_rotation":
            local_t = t - phase.t_start if self.trajectory_enabled() else t
            return self._trajectory_transform_at(local_t)

        raise ValueError(f"Unsupported leader motion mode: {self._leader_motion.mode}")

    @staticmethod
    def _rotation_transform(t: float, motion: LeaderMotionConfig) -> AffineTransform:
        angle = motion.angular_rate * t
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        A = np.array(
            [
                [cos_a, -sin_a, 0.0],
                [sin_a, cos_a, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        b = np.array(motion.translation or [0.0, 0.0, 0.0], dtype=float)
        return AffineTransform(A=A, b=b)
