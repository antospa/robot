import time

import serial
import serial.tools.list_ports

from robot_config import (
    PICO_STATUS_TIMEOUT_S,
    PICO_USB_VID,
    REQUIRE_TOF_READY,
    SERIAL_IDLE_SLEEP_S,
    SERIAL_TIMEOUT_S,
    TOF_COLUMN_COUNT,
    TOF_SENSOR_COUNT,
    TOF_VECTOR_SIZE,
    USB_BAUD_RATE,
)


def empty_tof_vector():
    return [None for _ in range(TOF_VECTOR_SIZE)]


_latest_tof_vector = empty_tof_vector()
_serial_rx_buffer = ""


# Function to search among the USB ports for the Raspberry Pi Pico and establish a serial connection
def find_and_connect_pico():
    global _serial_rx_buffer

    ports = serial.tools.list_ports.comports()

    for port in ports:
        if port.vid == PICO_USB_VID:
            try:
                ser = serial.Serial(port.device, USB_BAUD_RATE, timeout=SERIAL_TIMEOUT_S)
                ser.reset_input_buffer()
                _serial_rx_buffer = ""
                print(f"[OK] Connected to Raspberry Pi Pico on {port.device}")
                return ser
            except serial.SerialException as e:
                raise RuntimeError(f"Could not open Pico serial port {port.device}: {e}") from e

    raise RuntimeError("Pico not found")


def parse_serial_line(line, *, print_warnings=False):
    parts = line.split(",")

    try:
        if len(parts) == 3:
            steps_l = float(parts[0])
            steps_r = float(parts[1])
            yaw_deg = float(parts[2])
            return "odom", (steps_l, steps_r, yaw_deg)

        packet_type = parts[0].upper()

        if packet_type == "ODOM":
            if len(parts) != 4:
                if print_warnings:
                    print("[!] Wrong odometry packet format:", line)
                return None

            steps_l = float(parts[1])
            steps_r = float(parts[2])
            yaw_deg = float(parts[3])
            return "odom", (steps_l, steps_r, yaw_deg)

        if packet_type == "TOF":
            if len(parts) != 3 + TOF_VECTOR_SIZE:
                if print_warnings:
                    print("[!] Wrong ToF packet format:", line)
                return None

            row_count = int(parts[1])
            column_count = int(parts[2])

            if row_count != TOF_SENSOR_COUNT or column_count != TOF_COLUMN_COUNT:
                if print_warnings:
                    print("[!] Wrong ToF matrix size:", line)
                return None

            values = [int(value) for value in parts[3:]]
            return "tof", values

        if packet_type == "STATUS":
            if len(parts) != 5 + TOF_SENSOR_COUNT or parts[1] != "IMU" or parts[3] != "TOF":
                if print_warnings:
                    print("[!] Wrong status packet format:", line)
                return None

            tof_count = int(parts[4])

            if tof_count != TOF_SENSOR_COUNT:
                if print_warnings:
                    print("[!] Wrong status ToF count:", line)
                return None

            status = {
                "imu": bool(int(parts[2])),
                "tof": [bool(int(value)) for value in parts[5:]],
            }
            return "status", status

        if print_warnings:
            print("[!] Unknown packet type:", line)
        return None
    except ValueError:
        if print_warnings:
            print("[!] Could not parse packet:", line)
        return None


# Function to read all currently available complete packets from the serial buffer.
def read_latest_pico_data(ser, *, print_warnings=False):
    global _latest_tof_vector
    global _serial_rx_buffer

    data = {
        "odom": None,
        "tof": None,
        "status": None,
    }

    waiting_bytes = ser.in_waiting
    if waiting_bytes <= 0:
        return data

    chunk = ser.read(waiting_bytes).decode("utf-8", errors="ignore")
    chunk = chunk.replace("\r", "\n")
    _serial_rx_buffer += chunk

    lines = _serial_rx_buffer.split("\n")
    _serial_rx_buffer = lines.pop()

    if len(_serial_rx_buffer) > 4096:
        _serial_rx_buffer = _serial_rx_buffer[-4096:]

    for line in lines:
        line = line.strip()

        if line == "":
            continue

        parsed = parse_serial_line(line, print_warnings=print_warnings)

        if parsed is None:
            continue

        packet_type, payload = parsed

        if packet_type == "odom":
            data["odom"] = payload
        elif packet_type == "tof":
            _latest_tof_vector = payload
            data["tof"] = _latest_tof_vector[:]
        elif packet_type == "status":
            data["status"] = payload

    return data


# Function to read the latest complete odometry packet from the serial buffer.
def read_latest_packet(ser, *, print_warnings=False):
    return read_latest_pico_data(ser, print_warnings=print_warnings)["odom"]


def request_pico_status(ser, *, timeout_s=PICO_STATUS_TIMEOUT_S, print_warnings=False):
    ser.write(b"<STATUS>\n")
    ser.flush()

    start_time = time.monotonic()

    while time.monotonic() - start_time < timeout_s:
        data = read_latest_pico_data(ser, print_warnings=print_warnings)

        if data["status"] is not None:
            return data["status"]

        time.sleep(SERIAL_IDLE_SLEEP_S)

    return None


def check_pico_ready(ser):
    status = request_pico_status(ser)

    if status is None:
        raise RuntimeError("No status response from Pico")

    imu_ready = status["imu"]
    tof_ready = status["tof"]
    missing_tof = [index for index, ready in enumerate(tof_ready) if not ready]

    if not imu_ready or (REQUIRE_TOF_READY and missing_tof):
        details = []

        if not imu_ready:
            details.append("IMU not ready")

        if missing_tof:
            details.append(f"ToF not ready: {missing_tof}")

        raise RuntimeError("Pico sensors are not ready: " + "; ".join(details))

    if missing_tof:
        print(f"[!] Pico ready with IMU, but ToF not required/not all ready: missing {missing_tof}")
    else:
        print(f"[OK] Pico sensors ready: IMU and {TOF_SENSOR_COUNT} ToF initialized")

    return status


# Function to send velocity commands to the Pico in the format "<left_steps_s,right_steps_s>\n"
def send_velocity(ser, left_steps_s, right_steps_s):
    command = f"<{left_steps_s:.1f},{right_steps_s:.1f}>\n"
    ser.write(command.encode("utf-8"))
    ser.flush()


# Function to send a command to the Pico to zero the IMU yaw
def zero_imu_yaw(ser):
    global _serial_rx_buffer

    start_time = time.monotonic()

    while time.monotonic() - start_time < 2.0:
        if read_latest_pico_data(ser)["odom"] is not None:
            break

        time.sleep(0.01)

    ser.write(b"<ZERO_IMU>\n")
    ser.flush()
    time.sleep(0.10)
    ser.reset_input_buffer()
    _serial_rx_buffer = ""
    print("[*] IMU yaw zeroed.")
