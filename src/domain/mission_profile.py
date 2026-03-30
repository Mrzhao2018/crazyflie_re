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
        phases = self._phases
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

                start_transform = self.affine_transform_at(seg_start)
                end_transform = self.affine_transform_at(seg_end)

                start_pos = start_transform.A @ nominal_position + start_transform.b
                end_pos = end_transform.A @ nominal_position + end_transform.b
                delta = end_pos - start_pos

                pieces.append(
                    TrajectoryPiece(
                        duration=duration,
                        x=self._linear_coeffs(float(delta[0]), duration),
                        y=self._linear_coeffs(float(delta[1]), duration),
                        z=self._linear_coeffs(float(delta[2]), duration),
                        yaw=[0.0] * 8,
                    )
                )

        return pieces or [
            TrajectoryPiece(
                duration=max(self.total_time(), 0.1),
                x=[0.0] * 8,
                y=[0.0] * 8,
                z=[0.0] * 8,
                yaw=[0.0] * 8,
            )
        ]

    @staticmethod
    def _linear_coeffs(displacement: float, duration: float) -> list[float]:
        velocity = displacement / duration
        return [0.0, velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def phase_at(self, t: float) -> MissionPhase:
        for phase in self._phases:
            if phase.t_start <= t < phase.t_end:
                return phase
        return self._phases[-1]

    def affine_transform_at(self, t: float) -> AffineTransform:
        phase = self.phase_at(t)

        if phase.mode == "settle" or self._leader_motion.mode == "hold":
            return AffineTransform(A=np.eye(3), b=np.zeros(3))

        if self._leader_motion.mode == "affine_rotation":
            return self._rotation_transform(t, self._leader_motion)

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
