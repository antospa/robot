import math

import numpy as np

from robot_config import METERS_PER_STEP, TRACK_WIDTH

# Setup part of the code


def normalize_angle(angle_rad):
    """Normalize an angle to the interval (-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi

    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi

    return angle_rad


def prompt_odometry_type():
    while True:
        odom_type = input("Select odometry [1=IMU yaw, 2=ruote]: ").strip()

        if odom_type == "1":
            return "IMU"

        if odom_type == "2":
            return "WHEEL"

        print("[!] Invalid odometry type. Use 1 for IMU yaw or 2 for ruote.")



# Odometry update functions

def update_robot_odometry(state, steps, yaw_deg, odometry_type):
    """Update both wheel and IMU-yaw odometry, then select the active estimate."""
    steps = np.asarray(steps, dtype=float)

    if state.first_cycle:
        state.prev_steps[:] = steps
        state.first_cycle = False

    delta_steps = steps - state.prev_steps
    state.prev_steps[:] = steps

    wheel_displacements = delta_steps * METERS_PER_STEP
    delta_s = np.mean(wheel_displacements)
    delta_theta_wheels = (wheel_displacements[1] - wheel_displacements[0]) / TRACK_WIDTH
    yaw_rad = math.radians(yaw_deg)

    state.delta_s = delta_s
    state.delta_theta_wheel_rad = delta_theta_wheels

    update_wheel_odometry(state, delta_s, delta_theta_wheels)
    update_imu_yaw_odometry(state, delta_s, yaw_rad)
    select_active_odometry(state, odometry_type)


def update_wheel_odometry(state, delta_s, delta_theta_wheels):
    """Integrate differential-drive odometry using midpoint orientation."""
    theta_wheel_mid = state.theta_wheel_rad + delta_theta_wheels / 2.0

    state.wheel_position += delta_s * np.array(
        [math.cos(theta_wheel_mid), math.sin(theta_wheel_mid)],
        dtype=float,
    )
    state.theta_wheel_rad = normalize_angle(state.theta_wheel_rad + delta_theta_wheels)


def update_imu_yaw_odometry(state, delta_s, yaw_rad):
    """Integrate translation from wheel motion using IMU yaw as heading."""
    state.imu_position += delta_s * np.array([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=float)
    state.theta_imu_rad = normalize_angle(yaw_rad)


def select_active_odometry(state, odometry_type):
    if odometry_type == "WHEEL":
        state.position[:] = state.wheel_position
        state.theta_rad = state.theta_wheel_rad
        return

    if odometry_type == "IMU":
        state.position[:] = state.imu_position
        state.theta_rad = state.theta_imu_rad
        return

    raise ValueError(f"Unsupported odometry type: {odometry_type!r}")
