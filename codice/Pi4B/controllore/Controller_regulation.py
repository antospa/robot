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
from robot_config import B_POINT_P, K_X, K_Y, POSITION_TOLERANCE_M


CONTROLLER_TYPE = "XY"
XY_GAINS = np.array([K_X, K_Y], dtype=float)


# Desired target definition

def setup_interface():
    while True:
        try:
            x_target = float(input("Enter target X (meters): "))
            y_target = float(input("Enter target Y (meters): "))
            print(f"\n[OK] Target locked: ({x_target}, {y_target})")
            return np.array([x_target, y_target], dtype=float)
        except ValueError:
            print("[!] Invalid value. Please try again.")

# Control law computation

def calculate_io_linearization(control, control_memory):
    robot_state = control.robot_state
    point_b = b_point_pose(robot_state.position, robot_state.theta_rad, B_POINT_P)
    error = control.target - point_b
    position_error = np.linalg.norm(error)

    control.point_b[:] = point_b
    control.error[:] = error
    control.position_error = position_error

    if position_error < POSITION_TOLERANCE_M:
        control_memory.prev_wheel_mps[:] = 0.0
        return

    cartesian_velocity_mps = XY_GAINS * error
    raw_unicycle = io_linearization_to_unicycle(
        robot_state.theta_rad,
        B_POINT_P,
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
