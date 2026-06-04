import sys
import time
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
sys.path.insert(0, str(LIB_DIR))

from pico_serial import find_and_connect_pico, read_latest_pico_data, request_pico_status
from robot_config import SERIAL_IDLE_SLEEP_S, TOF_COLUMN_COUNT, TOF_SENSOR_COUNT


STATUS_PERIOD_S = 5.0


def format_ready(value):
    return "OK" if value else "FAIL"


def print_status(status):
    if status is None:
        print("[STATUS] No response from Pico")
        return

    tof_status = " ".join(
        f"T{index}:{format_ready(ready)}"
        for index, ready in enumerate(status["tof"])
    )
    print(f"\n[STATUS] IMU:{format_ready(status['imu'])} | {tof_status}")


def print_tof_vector(tof_values):
    print(f"\n[TOF] {TOF_SENSOR_COUNT}x{TOF_COLUMN_COUNT} compact vector flattened by rows")

    for row in range(TOF_SENSOR_COUNT):
        start = row * TOF_COLUMN_COUNT
        values = tof_values[start:start + TOF_COLUMN_COUNT]
        formatted_values = " ".join(f"{value:5d}" for value in values)
        print(f"  S{row}: {formatted_values}")


def print_odom(odom):
    steps_l, steps_r, yaw_deg = odom
    print(
        f"[ODOM] left={steps_l:10.2f} step | "
        f"right={steps_r:10.2f} step | yaw={yaw_deg:8.2f} deg"
    )


def main():
    ser = find_and_connect_pico()
    last_status_time = 0.0

    try:
        print_status(request_pico_status(ser))
        last_status_time = time.monotonic()
        print("\n[*] Listening to Pico serial stream. Press CTRL+C to stop.\n")

        while True:
            now = time.monotonic()

            if now - last_status_time >= STATUS_PERIOD_S:
                print_status(request_pico_status(ser))
                last_status_time = now

            data = read_latest_pico_data(ser, print_warnings=True)

            if data["status"] is not None:
                print_status(data["status"])

            if data["odom"] is not None:
                print_odom(data["odom"])

            if data["tof"] is not None:
                print_tof_vector(data["tof"])

            time.sleep(SERIAL_IDLE_SLEEP_S)

    except KeyboardInterrupt:
        print("\n[*] Serial monitor stopped.")
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
