import sys
import time
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
sys.path.insert(0, str(LIB_DIR))

from pico_serial import find_and_connect_pico, read_latest_pico_data, send_velocity
from robot_config import MAX_SPEED_MPS, METERS_PER_STEP, SERIAL_IDLE_SLEEP_S


COMMAND_PERIOD_S = 0.05
DEFAULT_DURATION_S = 3.0


def prompt_float(prompt, default=None):
    while True:
        raw_value = input(prompt).strip()

        if raw_value == "" and default is not None:
            return default

        try:
            return float(raw_value)
        except ValueError:
            print("[!] Invalid numeric value.")


def main():
    print("\n=== RICObot max speed test ===")
    print(f"[*] Robot config MAX_SPEED_MPS: {MAX_SPEED_MPS:.3f} m/s")
    print(f"[*] Conversion factor: {METERS_PER_STEP:.8f} m/step")
    print("[*] Positive speed drives both wheels forward; negative speed drives backward.")

    speed_mps = prompt_float("Target wheel speed [m/s]: ")
    duration_s = prompt_float(f"Duration [s] (default {DEFAULT_DURATION_S:.1f}, 0 = until CTRL+C): ", DEFAULT_DURATION_S)
    wheel_steps_s = speed_mps / METERS_PER_STEP

    if abs(speed_mps) > MAX_SPEED_MPS:
        print(
            f"[WARN] Requested speed {speed_mps:.3f} m/s is above Python config "
            f"MAX_SPEED_MPS={MAX_SPEED_MPS:.3f} m/s."
        )
        print("[WARN] The Pico firmware may still limit or reject the command if its config is lower.")

    print(f"[*] Command: left={wheel_steps_s:.1f} step/s, right={wheel_steps_s:.1f} step/s")
    input("[*] Press ENTER to start. Keep the robot lifted or in a safe free area.")

    ser = find_and_connect_pico()
    start_time = time.monotonic()
    previous_sample_time = None
    previous_steps_l = None
    previous_steps_r = None

    try:
        while duration_s <= 0.0 or (time.monotonic() - start_time) < duration_s:
            send_velocity(ser, wheel_steps_s, wheel_steps_s)
            data = read_latest_pico_data(ser)["odom"]

            if data is not None:
                steps_l, steps_r, yaw_deg = data
                now = time.monotonic()
                elapsed = now - start_time
                measured_left_steps_s = 0.0
                measured_right_steps_s = 0.0
                measured_left_mps = 0.0
                measured_right_mps = 0.0

                if previous_sample_time is not None:
                    dt = now - previous_sample_time

                    if dt > 0.0:
                        measured_left_steps_s = (steps_l - previous_steps_l) / dt
                        measured_right_steps_s = (steps_r - previous_steps_r) / dt
                        measured_left_mps = measured_left_steps_s * METERS_PER_STEP
                        measured_right_mps = measured_right_steps_s * METERS_PER_STEP

                previous_sample_time = now
                previous_steps_l = steps_l
                previous_steps_r = steps_r

                print(
                    f"\rTime:{elapsed:.2f}s "
                    f"Cmd:{wheel_steps_s:.0f} step/s "
                    f"Meas L:{measured_left_steps_s:.0f} R:{measured_right_steps_s:.0f} step/s "
                    f"V L:{measured_left_mps:.3f} R:{measured_right_mps:.3f} m/s "
                    f"Steps L:{steps_l:.0f} R:{steps_r:.0f} "
                    f"Yaw:{yaw_deg:.1f} deg    ",
                    end="",
                )

            time.sleep(max(SERIAL_IDLE_SLEEP_S, COMMAND_PERIOD_S))

        print("\n[*] Test duration completed.")

    except KeyboardInterrupt:
        print("\n[!] Manual interruption.")
    finally:
        if "ser" in locals() and ser.is_open:
            send_velocity(ser, 0.0, 0.0)
            time.sleep(0.1)
            ser.close()
            print("[*] Robot stopped and port closed safely.")


if __name__ == "__main__":
    main()
