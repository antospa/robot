import sys
import time
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
sys.path.insert(0, str(LIB_DIR))

from pico_serial import find_and_connect_pico, read_latest_pico_data, send_velocity
from robot_config import METERS_PER_STEP, SERIAL_IDLE_SLEEP_S, WHEEL_RADIUS


COMMAND_PERIOD_S = 0.02
DEFAULT_DISTANCE_M = 1.0
DEFAULT_SPEED_MPS = 0.08
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


def signed_target_reached(delta_steps, target_steps):
    if target_steps >= 0:
        return delta_steps >= target_steps

    return delta_steps <= target_steps


def main():
    print("\n=== RICObot meters-per-step calibration ===")
    print("[*] This test drives both wheels by the same commanded step count.")
    print("[*] After the robot stops, measure the real straight distance on the floor.")
    print(f"[*] Current METERS_PER_STEP: {METERS_PER_STEP:.10f} m/step")
    print(f"[*] Current WHEEL_RADIUS: {WHEEL_RADIUS:.6f} m")

    target_distance = prompt_float(
        f"Nominal target distance [m] (default {DEFAULT_DISTANCE_M:.2f}): ",
        DEFAULT_DISTANCE_M,
    )
    speed_mps = prompt_float(
        f"Wheel speed [m/s] (default {DEFAULT_SPEED_MPS:.2f}): ",
        DEFAULT_SPEED_MPS,
    )

    target_steps = target_distance / METERS_PER_STEP
    command_steps_s = speed_mps / METERS_PER_STEP

    print(f"[*] Nominal target steps per wheel: {target_steps:.1f}")
    print(f"[*] Command speed: {command_steps_s:.1f} step/s")
    input("[*] Place the robot straight, mark the start point, then press ENTER.")

    ser = find_and_connect_pico()
    start_steps_l = None
    start_steps_r = None
    last_steps_l = None
    last_steps_r = None
    start_time = time.monotonic()

    try:
        while True:
            send_velocity(ser, command_steps_s, command_steps_s)
            data = read_latest_pico_data(ser)["odom"]

            if data is None:
                time.sleep(SERIAL_IDLE_SLEEP_S)
                continue

            steps_l, steps_r, yaw_deg = data

            if start_steps_l is None:
                start_steps_l = steps_l
                start_steps_r = steps_r

            delta_l = steps_l - start_steps_l
            delta_r = steps_r - start_steps_r
            avg_delta_steps = (delta_l + delta_r) / 2.0
            elapsed = time.monotonic() - start_time
            last_steps_l = steps_l
            last_steps_r = steps_r

            print(
                f"\rTime:{elapsed:.2f}s "
                f"Target:{target_steps:.0f} step "
                f"dL:{delta_l:.0f} dR:{delta_r:.0f} avg:{avg_delta_steps:.0f} "
                f"Nominal distance:{avg_delta_steps * METERS_PER_STEP:.3f} m "
                f"Yaw:{yaw_deg:.1f} deg    ",
                end="",
            )

            if signed_target_reached(avg_delta_steps, target_steps):
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

    if last_steps_l is None or start_steps_l is None:
        print("[!] No valid step samples collected.")
        return

    final_delta_l = last_steps_l - start_steps_l
    final_delta_r = last_steps_r - start_steps_r
    final_avg_delta_steps = (final_delta_l + final_delta_r) / 2.0
    measured_distance = prompt_float("Measured real distance [m]: ")

    if final_avg_delta_steps == 0.0:
        print("[!] Step delta is zero, cannot compute calibration.")
        return

    calibrated_meters_per_step = measured_distance / final_avg_delta_steps
    calibrated_wheel_radius = WHEEL_RADIUS * (calibrated_meters_per_step / METERS_PER_STEP)

    print("\n=== Calibration result ===")
    print(f"Final delta steps: L={final_delta_l:.1f}, R={final_delta_r:.1f}, avg={final_avg_delta_steps:.1f}")
    print(f"Nominal distance from current config: {final_avg_delta_steps * METERS_PER_STEP:.4f} m")
    print(f"Measured real distance: {measured_distance:.4f} m")
    print(f"Suggested METERS_PER_STEP: {calibrated_meters_per_step:.10f}")
    print(f"Suggested WHEEL_RADIUS: {calibrated_wheel_radius:.6f} m")
    print("[*] Update WHEEL_RADIUS in robot_config.py, then repeat once to verify.")


if __name__ == "__main__":
    main()
