import math
import sys
import time
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
sys.path.insert(0, str(LIB_DIR))

from odometry import normalize_angle
from pico_serial import find_and_connect_pico, read_latest_pico_data, send_velocity
from robot_config import METERS_PER_STEP, SERIAL_IDLE_SLEEP_S, TRACK_WIDTH


COMMAND_PERIOD_S = 0.02
DEFAULT_TURNS = 1.0
DEFAULT_WHEEL_SPEED_MPS = 0.06
STOP_SETTLE_S = 0.2


def prompt_float(prompt, default=None):
    while True:
        raw_value = input(prompt).strip()

        if raw_value == "" and default is not None:
            return default

        try:
            return float(raw_value)
        except ValueError:
            print("[!] Invalid numeric value.")


def prompt_direction():
    while True:
        raw_value = input("Direction [1=CCW, 2=CW] (default 1): ").strip()

        if raw_value in {"", "1", "ccw", "CCW"}:
            return 1.0, "CCW"

        if raw_value in {"2", "cw", "CW"}:
            return -1.0, "CW"

        print("[!] Invalid direction.")


def unwrap_yaw_sample(yaw_deg, previous_yaw_rad, unwrapped_yaw_rad):
    yaw_rad = math.radians(yaw_deg)

    if previous_yaw_rad is None:
        return yaw_rad, yaw_rad

    return yaw_rad, unwrapped_yaw_rad + normalize_angle(yaw_rad - previous_yaw_rad)


def main():
    print("\n=== RICObot track-width calibration ===")
    print("[*] This test rotates in place using equal/opposite wheel commands.")
    print("[*] The effective track width is computed from wheel step motion and IMU yaw.")
    print(f"[*] Current TRACK_WIDTH: {TRACK_WIDTH:.6f} m")
    print(f"[*] Current METERS_PER_STEP: {METERS_PER_STEP:.10f} m/step")

    turns = prompt_float(f"Target turns (default {DEFAULT_TURNS:.1f}): ", DEFAULT_TURNS)
    wheel_speed_mps = prompt_float(
        f"Wheel speed magnitude [m/s] (default {DEFAULT_WHEEL_SPEED_MPS:.2f}): ",
        DEFAULT_WHEEL_SPEED_MPS,
    )
    direction_sign, direction_label = prompt_direction()

    target_yaw_rad = direction_sign * turns * 2.0 * math.pi
    target_wheel_distance = abs(target_yaw_rad) * TRACK_WIDTH / 2.0
    target_steps_per_wheel = target_wheel_distance / METERS_PER_STEP
    command_steps_s = abs(wheel_speed_mps) / METERS_PER_STEP
    left_command_steps_s = -direction_sign * command_steps_s
    right_command_steps_s = direction_sign * command_steps_s

    print(f"[*] Direction: {direction_label}")
    print(f"[*] Nominal target yaw: {math.degrees(target_yaw_rad):.1f} deg")
    print(f"[*] Nominal target steps per wheel: {target_steps_per_wheel:.1f}")
    print(f"[*] Command: L={left_command_steps_s:.1f}, R={right_command_steps_s:.1f} step/s")
    input("[*] Place the robot with room to rotate, then press ENTER.")

    ser = find_and_connect_pico()
    start_steps_l = None
    start_steps_r = None
    final_steps_l = None
    final_steps_r = None
    previous_yaw_rad = None
    start_yaw_unwrapped_rad = None
    yaw_unwrapped_rad = None
    start_time = time.monotonic()

    try:
        while True:
            send_velocity(ser, left_command_steps_s, right_command_steps_s)
            data = read_latest_pico_data(ser)["odom"]

            if data is None:
                time.sleep(SERIAL_IDLE_SLEEP_S)
                continue

            steps_l, steps_r, yaw_deg = data
            previous_yaw_rad, yaw_unwrapped_rad = unwrap_yaw_sample(
                yaw_deg,
                previous_yaw_rad,
                yaw_unwrapped_rad,
            )

            if start_steps_l is None:
                start_steps_l = steps_l
                start_steps_r = steps_r
                start_yaw_unwrapped_rad = yaw_unwrapped_rad

            delta_l = steps_l - start_steps_l
            delta_r = steps_r - start_steps_r
            avg_abs_delta_steps = (abs(delta_l) + abs(delta_r)) / 2.0
            imu_delta_yaw_rad = yaw_unwrapped_rad - start_yaw_unwrapped_rad
            elapsed = time.monotonic() - start_time
            final_steps_l = steps_l
            final_steps_r = steps_r

            print(
                f"\rTime:{elapsed:.2f}s "
                f"Target:{target_steps_per_wheel:.0f} step/wheel "
                f"dL:{delta_l:.0f} dR:{delta_r:.0f} "
                f"IMU yaw:{math.degrees(imu_delta_yaw_rad):.1f} deg    ",
                end="",
            )

            if avg_abs_delta_steps >= target_steps_per_wheel:
                break

            time.sleep(COMMAND_PERIOD_S)

        send_velocity(ser, 0.0, 0.0)
        time.sleep(STOP_SETTLE_S)
        print("\n[*] Step target reached.")

    except KeyboardInterrupt:
        print("\n[!] Manual interruption.")
    finally:
        if "ser" in locals() and ser.is_open:
            send_velocity(ser, 0.0, 0.0)
            ser.close()
            print("[*] Robot stopped and port closed safely.")

    if final_steps_l is None or start_steps_l is None or start_yaw_unwrapped_rad is None:
        print("[!] No valid samples collected.")
        return

    final_delta_l = final_steps_l - start_steps_l
    final_delta_r = final_steps_r - start_steps_r
    final_delta_s_l = final_delta_l * METERS_PER_STEP
    final_delta_s_r = final_delta_r * METERS_PER_STEP
    final_imu_delta_yaw_rad = yaw_unwrapped_rad - start_yaw_unwrapped_rad

    if abs(final_imu_delta_yaw_rad) < math.radians(5.0):
        print("[!] IMU yaw change too small, cannot compute track width reliably.")
        return

    calibrated_track_width = (final_delta_s_r - final_delta_s_l) / final_imu_delta_yaw_rad
    calibrated_track_width_abs = abs(calibrated_track_width)

    print("\n=== Calibration result ===")
    print(f"Final delta steps: L={final_delta_l:.1f}, R={final_delta_r:.1f}")
    print(f"Wheel travel: L={final_delta_s_l:.4f} m, R={final_delta_s_r:.4f} m")
    print(f"IMU yaw change: {math.degrees(final_imu_delta_yaw_rad):.2f} deg")
    print(f"Suggested TRACK_WIDTH: {calibrated_track_width_abs:.6f} m")
    print("[*] Update TRACK_WIDTH in robot_config.py, then repeat clockwise and counterclockwise.")


if __name__ == "__main__":
    main()
