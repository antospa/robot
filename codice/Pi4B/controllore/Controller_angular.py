import sys
from pathlib import Path

import numpy as np

LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

from kinematics import unicycle_to_wheel_mps, wheel_mps_to_steps_s, wheel_mps_to_unicycle
from motion_limits import apply_wheel_speed_limits
from odometry import normalize_angle
from robot_config import K_YAW_P


CONTROLLER_TYPE = "THETA"


def setup_interface():
    while True:
        try:
            target_distance = float(input("Enter target distance (meters): "))
            forward_speed_mps = float(input("Enter forward speed (m/s): "))
            yaw_reference_deg = float(input("Enter yaw reference (deg): "))

            print(
                f"\n[OK] Theta test locked: distance={target_distance:.3f} m, "
                f"speed={forward_speed_mps:.3f} m/s, yaw_ref={yaw_reference_deg:.2f} deg"
            )
            return target_distance, forward_speed_mps, yaw_reference_deg
        except ValueError:
            print("[!] Invalid value. Please try again.")


def calculate_yaw_hold_command(control, control_memory):
    yaw_error_rad = normalize_angle(control.yaw_reference_rad - control.control_theta_rad)
    omega_feedback_rad_s = K_YAW_P * yaw_error_rad

    unicycle_command = np.array([control.v_feedforward_mps, omega_feedback_rad_s], dtype=float)
    wheel_mps = unicycle_to_wheel_mps(unicycle_command)
    wheel_mps = apply_wheel_speed_limits(
        wheel_mps,
        control_memory.prev_wheel_mps,
        control.dt,
    )
    control_memory.prev_wheel_mps[:] = wheel_mps

    cmd_unicycle = wheel_mps_to_unicycle(wheel_mps)
    cmd_wheel_steps_s = wheel_mps_to_steps_s(wheel_mps)

    control.yaw_error_rad = yaw_error_rad
    control.omega_feedback_rad_s = omega_feedback_rad_s
    control.cmd_unicycle[:] = cmd_unicycle
    control.cmd_wheel_mps[:] = wheel_mps
    control.cmd_wheel_steps_s[:] = cmd_wheel_steps_s
