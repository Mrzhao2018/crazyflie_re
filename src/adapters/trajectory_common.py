"""Trajectory and full-state helpers shared by real and sim transports."""

from __future__ import annotations

import math

POLY4D_RAW_PIECE_BYTES = 132
TRAJECTORY_MEMORY_BYTES = 4096
TRAJECTORY_MAX_PIECES = 255

# cflib quantizes full-state vectors as int16 millimetres. Keep a little
# headroom so both real and sim transports reject the same invalid setpoints.
FULL_STATE_INT16_LIMIT = 32.0


def validate_full_state_vector(name: str, vec) -> None:
    try:
        components = [float(vec[0]), float(vec[1]), float(vec[2])]
    except (TypeError, IndexError, ValueError) as exc:
        raise ValueError(
            f"full_state {name} must be length-3 numeric vector, got {vec!r}"
        ) from exc
    for axis, value in zip(("x", "y", "z"), components):
        if not math.isfinite(value):
            raise ValueError(
                f"full_state {name}.{axis} is not finite: {value!r}"
            )
        if abs(value) > FULL_STATE_INT16_LIMIT:
            raise ValueError(
                f"full_state {name}.{axis}={value} exceeds int16 quantization "
                f"range (|v| <= {FULL_STATE_INT16_LIMIT})"
            )


def _coefficient_count(values) -> int:
    try:
        return len(values)
    except TypeError:
        return 0


def _compressed_axis_payload_len(values, *, eps: float = 1e-9) -> int:
    """Estimate compressed trajectory payload terms without importing cflib."""

    coeffs = [float(value) for value in values]
    degree = 0
    for idx in range(len(coeffs) - 1, 0, -1):
        if abs(coeffs[idx]) > eps:
            degree = idx
            break
    if degree == 0:
        return 0
    if degree == 1:
        return 1
    if degree <= 3:
        return 3
    return 7


def estimate_trajectory_bytes(pieces: list, trajectory_type: str = "poly4d") -> int:
    """Return a conservative trajectory memory estimate.

    The sim backend does not upload to Crazyflie trajectory memory, but the
    readiness path still records the same budget fields. This implementation is
    cflib-free so WSL can import CLI modules without real-radio dependencies.
    """

    if trajectory_type == "poly4d":
        return len(pieces) * POLY4D_RAW_PIECE_BYTES
    if trajectory_type != "poly4d_compressed":
        raise ValueError(f"Unsupported trajectory_type: {trajectory_type}")
    if not pieces:
        return 0

    # cflib packs CompressedStart as four int16 values (8 bytes). Each
    # CompressedSegment has one type byte, uint16 duration, then int16 payload
    # terms for x/y/z/yaw.
    total_bytes = 8
    for piece in pieces:
        payload_terms = (
            _compressed_axis_payload_len(piece.x)
            + _compressed_axis_payload_len(piece.y)
            + _compressed_axis_payload_len(piece.z)
            + _compressed_axis_payload_len(piece.yaw)
        )
        total_bytes += 3 + (2 * payload_terms)
        for axis_name in ("x", "y", "z", "yaw"):
            if _coefficient_count(getattr(piece, axis_name)) != 8:
                raise ValueError(
                    f"trajectory polynomial {axis_name} must have 8 coefficients"
                )
    return total_bytes
