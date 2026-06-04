import math

import numpy as np

from robot_config import METERS_PER_STEP, TRACK_WIDTH


def heading_vector(theta_rad):
    return np.array([math.cos(theta_rad), math.sin(theta_rad)], dtype=float)


def b_point_pose(position, theta_rad, b_point_distance):
    """Return the point B placed ahead of the robot center by b_point_distance."""
    return np.asarray(position, dtype=float) + b_point_distance * heading_vector(theta_rad)


def io_linearization_to_unicycle(theta_rad, b_point_distance, cartesian_velocity_mps):
    """Map desired Cartesian velocity of point B into unicycle commands [v, omega]."""
    u_x_mps, u_y_mps = np.asarray(cartesian_velocity_mps, dtype=float)
    v_mps = math.cos(theta_rad) * u_x_mps + math.sin(theta_rad) * u_y_mps
    omega_rad_s = (
        (-math.sin(theta_rad) / b_point_distance) * u_x_mps
        + (math.cos(theta_rad) / b_point_distance) * u_y_mps
    )
    return np.array([v_mps, omega_rad_s], dtype=float)


def unicycle_to_wheel_mps(unicycle_mps_rad_s, track_width=TRACK_WIDTH):
    """Convert unicycle command [v, omega] into wheel speeds [left, right]."""
    v_mps, omega_rad_s = np.asarray(unicycle_mps_rad_s, dtype=float)
    half_track = track_width / 2.0
    return np.array(
        [
            v_mps - half_track * omega_rad_s,
            v_mps + half_track * omega_rad_s,
        ],
        dtype=float,
    )


def wheel_mps_to_steps_s(wheel_mps, meters_per_step=METERS_PER_STEP):
    """Convert wheel speeds [left, right] from m/s to step/s."""
    return np.asarray(wheel_mps, dtype=float) / meters_per_step


def wheel_mps_to_unicycle(wheel_mps, track_width=TRACK_WIDTH):
    """Convert wheel speeds [left, right] into unicycle command [v, omega]."""
    left_mps, right_mps = np.asarray(wheel_mps, dtype=float)
    return np.array(
        [
            (right_mps + left_mps) / 2.0,
            (right_mps - left_mps) / track_width,
        ],
        dtype=float,
    )


def wheel_steps_s_to_mps(wheel_steps_s, meters_per_step=METERS_PER_STEP):
    """Convert wheel speeds [left, right] from step/s to m/s."""
    return np.asarray(wheel_steps_s, dtype=float) * meters_per_step
