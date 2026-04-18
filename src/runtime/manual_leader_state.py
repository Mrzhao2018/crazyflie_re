"""手动leader控制运行时状态"""

from dataclasses import dataclass
import math
import time
import numpy as np

from .manual_input_port import ManualLeaderIntent


_AXES = ("x", "y", "z")
_TARGETS = ("swarm", 1, 2, 3, 4)


@dataclass
class ManualLeaderStateSnapshot:
    translation: np.ndarray
    scale: float
    rotation: np.ndarray
    selected_axis: str
    target_mode: str
    selected_leader_id: int | None
    per_leader_offsets: dict[int, np.ndarray]
    last_input_t: float | None


class ManualLeaderState:
    """保存手动模式下的共享affine控制状态"""

    def __init__(
        self,
        default_axis: str = "z",
        min_scale: float = 0.6,
        max_scale: float = 1.6,
    ):
        if default_axis not in _AXES:
            raise ValueError(f"Unsupported default axis: {default_axis}")

        self._translation = np.zeros(3, dtype=float)
        self._scale = 1.0
        self._rotation = np.eye(3, dtype=float)
        self._selected_axis = default_axis
        self._target_index = 0
        self._per_leader_offsets = {
            1: np.zeros(3, dtype=float),
            2: np.zeros(3, dtype=float),
            3: np.zeros(3, dtype=float),
            4: np.zeros(3, dtype=float),
        }
        self._last_input_t: float | None = None
        self._min_scale = float(min_scale)
        self._max_scale = float(max_scale)

    def initialize_from_transform(
        self,
        translation: np.ndarray | None = None,
        scale: float = 1.0,
        rotation: np.ndarray | None = None,
    ) -> None:
        self._translation = (
            np.array(translation, dtype=float)
            if translation is not None
            else np.zeros(3)
        )
        self._scale = float(np.clip(scale, self._min_scale, self._max_scale))
        self._rotation = (
            np.array(rotation, dtype=float)
            if rotation is not None
            else np.eye(3, dtype=float)
        )
        self._target_index = 0
        for leader_id in self._per_leader_offsets:
            self._per_leader_offsets[leader_id] = np.zeros(3, dtype=float)
        self._last_input_t = None

    def apply_intent(
        self, intent: ManualLeaderIntent, t_wall: float | None = None
    ) -> None:
        if intent.axis_switch:
            self._cycle_axis()
        if intent.target_switch:
            self._cycle_target()

        target = _TARGETS[self._target_index]
        translation_delta = np.array(intent.translation_delta, dtype=float)
        if target == "swarm":
            self._translation = self._translation + translation_delta
            self._scale = float(
                np.clip(
                    self._scale + intent.scale_delta, self._min_scale, self._max_scale
                )
            )

            if intent.rotation_delta_deg != 0.0:
                delta_rotation = self._axis_rotation_matrix(
                    self._selected_axis, math.radians(intent.rotation_delta_deg)
                )
                self._rotation = delta_rotation @ self._rotation
        else:
            self._per_leader_offsets[int(target)] = (
                self._per_leader_offsets[int(target)] + translation_delta
            )

        self._last_input_t = time.time() if t_wall is None else t_wall

    def snapshot(self) -> ManualLeaderStateSnapshot:
        return ManualLeaderStateSnapshot(
            translation=self._translation.copy(),
            scale=self._scale,
            rotation=self._rotation.copy(),
            selected_axis=self._selected_axis,
            target_mode=(
                "swarm" if _TARGETS[self._target_index] == "swarm" else "leader"
            ),
            selected_leader_id=(
                None
                if _TARGETS[self._target_index] == "swarm"
                else int(_TARGETS[self._target_index])
            ),
            per_leader_offsets={
                leader_id: offset.copy()
                for leader_id, offset in self._per_leader_offsets.items()
            },
            last_input_t=self._last_input_t,
        )

    @staticmethod
    def _axis_rotation_matrix(axis: str, angle_rad: float) -> np.ndarray:
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        if axis == "x":
            return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        if axis == "y":
            return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def _cycle_axis(self) -> None:
        idx = _AXES.index(self._selected_axis)
        self._selected_axis = _AXES[(idx + 1) % len(_AXES)]

    def _cycle_target(self) -> None:
        self._target_index = (self._target_index + 1) % len(_TARGETS)
