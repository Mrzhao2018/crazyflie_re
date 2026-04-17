"""Shared base class for Follower controllers (v1 / v2).

Both controllers resolve axis gains, radial scales, feedforward clipping, and
output-velocity clipping in the same way. Factor the shared initialisation and
helpers here so ``FollowerController`` / ``FollowerControllerV2`` only keep
their core math.
"""

from __future__ import annotations

import numpy as np

from ..config.schema import ControlConfig
from ..domain.follower_reference import FollowerReferenceSet


class FollowerControllerBase:
    """Common init + radial/feedforward helpers shared by V1 and V2 controllers."""

    @staticmethod
    def _resolve_axis_gain(value: float | None, fallback: float) -> float:
        return fallback if value is None else value

    def __init__(self, config: ControlConfig):
        self.gain = config.gain
        self.max_velocity = config.max_velocity
        self.feedforward_gain = config.feedforward_gain
        self.max_feedforward_velocity = config.max_feedforward_velocity
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

    # ---- radial scaling ---------------------------------------------------

    def _compute_radial_scales(
        self,
        references: FollowerReferenceSet,
        active_follower_ids: list[int],
    ) -> tuple[dict[int, tuple[float, float]], list[int]]:
        """Return ``{fid: (gain_scale_xy, ff_scale_xy)}`` plus radial-scaled ids."""

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
        scales: dict[int, tuple[float, float]] = {}
        radial_scaled: list[int] = []
        for fid, radius_xy in follower_radii.items():
            radius_ratio = radius_xy / max_radius if max_radius > 1e-9 else 0.0
            gain_scale_xy = 1.0 + self.radial_gain_scale_xy * radius_ratio
            ff_scale_xy = 1.0 + self.radial_feedforward_scale_xy * radius_ratio
            scales[fid] = (gain_scale_xy, ff_scale_xy)
            if gain_scale_xy > 1.0 or ff_scale_xy > 1.0:
                radial_scaled.append(fid)
        return scales, radial_scaled

    # ---- clipping ---------------------------------------------------------

    def _clip_feedforward_velocity(self, target_velocity: np.ndarray) -> np.ndarray:
        clipped = np.array(target_velocity, dtype=float)
        xy_norm = np.linalg.norm(clipped[:2])
        if xy_norm > self.max_feedforward_velocity_xy > 0:
            clipped[:2] = clipped[:2] / xy_norm * self.max_feedforward_velocity_xy
        if abs(clipped[2]) > self.max_feedforward_velocity_z > 0:
            clipped[2] = np.sign(clipped[2]) * self.max_feedforward_velocity_z
        return clipped

    def _clip_output_velocity(self, velocity: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(velocity)
        if speed > self.max_velocity:
            return velocity / speed * self.max_velocity
        return velocity
