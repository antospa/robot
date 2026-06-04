import math
import sys
from pathlib import Path

import numpy as np

LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

from kinematics import (
    b_point_pose,
    io_linearization_to_unicycle,
    unicycle_to_wheel_mps,
    wheel_mps_to_steps_s,
    wheel_mps_to_unicycle,
)
from motion_limits import apply_wheel_speed_limits
from robot_config import (
    K_TRAJECTORY_X,
    K_TRAJECTORY_Y,
    TRAJECTORY_B_POINT_P,
    TRAJECTORY_PERIOD_S,
    TRAJECTORY_X_SPEED_MPS,
    TRAJECTORY_Y_AMPLITUDE_M,
)


CONTROLLER_TYPE = "TRAJECTORY_TRACKING"
TRAJECTORY_GAINS = np.array([K_TRAJECTORY_X, K_TRAJECTORY_Y], dtype=float)


def setup_interface():
    while True:
        try:
            x_start = float(input("Enter initial reference X (meters): "))
            y_start = float(input("Enter initial reference Y (meters): "))
            raw_mode = input("Select trajectory [1=straight x, 2=x plus sinusoidal y]: ").strip()

            if raw_mode == "1":
                trajectory_mode = "STRAIGHT_X"
            elif raw_mode == "2":
                trajectory_mode = "SINE_Y"
            else:
                print("[!] Invalid trajectory mode. Please try again.")
                continue

            print(f"\n[OK] Trajectory locked from: ({x_start}, {y_start})")
            print(f"[*] Trajectory mode: {trajectory_mode}")
            return np.array([x_start, y_start], dtype=float), trajectory_mode
        except ValueError:
            print("[!] Invalid value. Please try again.")


def trajectory_reference(start_position, elapsed_s, trajectory_mode):
    reference_position = np.asarray(start_position, dtype=float).copy()
    reference_velocity = np.array([TRAJECTORY_X_SPEED_MPS, 0.0], dtype=float)
    reference_position[0] += TRAJECTORY_X_SPEED_MPS * elapsed_s

    if trajectory_mode == "STRAIGHT_X":
        return reference_position, reference_velocity

    phase = 2.0 * math.pi * elapsed_s / TRAJECTORY_PERIOD_S
    reference_position[1] += TRAJECTORY_Y_AMPLITUDE_M * math.sin(phase)
    reference_velocity[1] = TRAJECTORY_Y_AMPLITUDE_M * (2.0 * math.pi / TRAJECTORY_PERIOD_S) * math.cos(phase)
    return reference_position, reference_velocity


def calculate_trajectory_tracking(control, control_memory):
    robot_state = control.robot_state
    reference_position, reference_velocity = trajectory_reference(
        control.target,
        control.elapsed,
        control.controller_mode,
    )

    point_b = b_point_pose(robot_state.position, robot_state.theta_rad, TRAJECTORY_B_POINT_P)
    error = reference_position - point_b
    position_error = np.linalg.norm(error)

    control.target[:] = reference_position
    control.point_b[:] = point_b
    control.error[:] = error
    control.position_error = position_error

    cartesian_velocity_mps = TRAJECTORY_GAINS * error + reference_velocity
    raw_unicycle = io_linearization_to_unicycle(
        robot_state.theta_rad,
        TRAJECTORY_B_POINT_P,
        cartesian_velocity_mps,
    )
    wheel_mps = unicycle_to_wheel_mps(raw_unicycle)
    wheel_mps = apply_wheel_speed_limits(
        wheel_mps,
        control_memory.prev_wheel_mps,
        control.dt,
    )
    control_memory.prev_wheel_mps[:] = wheel_mps

    cmd_unicycle = wheel_mps_to_unicycle(wheel_mps)
    cmd_wheel_steps_s = wheel_mps_to_steps_s(wheel_mps)

    control.raw_unicycle[:] = raw_unicycle
    control.cmd_unicycle[:] = cmd_unicycle
    control.cmd_wheel_mps[:] = wheel_mps
    control.cmd_wheel_steps_s[:] = cmd_wheel_steps_s
