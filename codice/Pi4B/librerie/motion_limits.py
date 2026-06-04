import numpy as np

from robot_config import MAX_SPEED_MPS, MAX_WHEEL_ACCEL_MPS2


def apply_wheel_speed_limits(
    wheel_mps,
    previous_wheel_mps,
    dt_s,
    max_speed_mps=MAX_SPEED_MPS,
    max_accel_mps2=MAX_WHEEL_ACCEL_MPS2,
):
    """Limit wheel speeds [left, right] by absolute max speed and shared acceleration ramp."""
    limited_wheel_mps = np.asarray(wheel_mps, dtype=float).copy()
    previous_wheel_mps = np.asarray(previous_wheel_mps, dtype=float)

    max_requested_speed = np.max(np.abs(limited_wheel_mps))
    previous_max_speed = np.max(np.abs(previous_wheel_mps))
    max_allowed_speed = min(max_speed_mps, previous_max_speed + max_accel_mps2 * max(0.0, dt_s))

    if max_requested_speed > max_allowed_speed and max_requested_speed > 0.0:
        limited_wheel_mps *= max_allowed_speed / max_requested_speed

    return limited_wheel_mps
