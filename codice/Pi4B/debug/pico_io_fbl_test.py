import sys
import time
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
sys.path.insert(0, str(LIB_DIR))

from pico_serial import find_and_connect_pico, read_latest_pico_data
from robot_config import METERS_PER_STEP, SERIAL_IDLE_SLEEP_S


def prompt_float(prompt):
    while True:
        raw_value = input(prompt).strip()

        try:
            return float(raw_value)
        except ValueError:
            print("[!] Invalid numeric value.")


def prompt_odometry():
    while True:
        raw_value = input("Pico odometry [1=wheel, 2=IMU yaw] (default 1): ").strip()

        if raw_value in {"", "1", "wheel", "WHEEL", "odom", "ODOM"}:
            return "ODOM"

        if raw_value in {"2", "imu", "IMU"}:
            return "IMU"

        print("[!] Invalid odometry choice.")


def send_raw_command(ser, payload):
    ser.write(f"<{payload}>\n".encode("utf-8"))
    ser.flush()


def main():
    print("\n=== Pico-only I/O FBL test ===")
    print("[*] The Raspberry Pi only sends the target. The control loop runs on the Pico.")
    print("[*] Firmware command format: <TARGET,x,y,ODOM> or <TARGET,x,y,IMU>")

    x_target = prompt_float("Target B X [m]: ")
    y_target = prompt_float("Target B Y [m]: ")
    odometry_type = prompt_odometry()

    ser = find_and_connect_pico()
    command = f"TARGET,{x_target:.6f},{y_target:.6f},{odometry_type}"
    print(f"[*] Sending: <{command}>")

    previous_time = None
    previous_steps_l = None
    previous_steps_r = None
    start_time = time.monotonic()

    try:
        ser.reset_input_buffer()
        send_raw_command(ser, command)

        while True:
            data = read_latest_pico_data(ser)["odom"]

            if data is None:
                time.sleep(SERIAL_IDLE_SLEEP_S)
                continue

            steps_l, steps_r, yaw_deg = data
            now = time.monotonic()
            elapsed = now - start_time
            measured_left_steps_s = 0.0
            measured_right_steps_s = 0.0

            if previous_time is not None:
                dt = now - previous_time

                if dt > 0.0:
                    measured_left_steps_s = (steps_l - previous_steps_l) / dt
                    measured_right_steps_s = (steps_r - previous_steps_r) / dt

            previous_time = now
            previous_steps_l = steps_l
            previous_steps_r = steps_r

            print(
                f"\rTime:{elapsed:.2f}s "
                f"Meas L:{measured_left_steps_s:.0f} R:{measured_right_steps_s:.0f} step/s "
                f"V L:{measured_left_steps_s * METERS_PER_STEP:.3f} "
                f"R:{measured_right_steps_s * METERS_PER_STEP:.3f} m/s "
                f"Steps L:{steps_l:.0f} R:{steps_r:.0f} "
                f"Yaw:{yaw_deg:.1f} deg    ",
                end="",
            )

    except KeyboardInterrupt:
        print("\n[!] Manual interruption.")
    finally:
        if "ser" in locals() and ser.is_open:
            send_raw_command(ser, "STOP")
            time.sleep(0.1)
            ser.close()
            print("[*] Stop command sent and port closed.")


if __name__ == "__main__":
    main()
