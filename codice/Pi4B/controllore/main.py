import importlib
import math
import sys
import time
from pathlib import Path

import numpy as np


LIB_DIR = Path(__file__).resolve().parents[1] / "librerie"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

from data_structures import (
    ControlClock,
    ControlMemory,
    ControllerDefinition,
    ControllerSession,
    RobotState,
    ThetaControl,
    XYControl,
)
from logger import create_controller_logger
from odometry import prompt_odometry_type, update_robot_odometry
from pico_serial import check_pico_ready, find_and_connect_pico, read_latest_pico_data, send_velocity, zero_imu_yaw
from robot_config import CONTROL_LOOP_PERIOD_S, METERS_PER_STEP, POSITION_TOLERANCE_M, SERIAL_IDLE_SLEEP_S


CONTROLLERS = {
    "1": ControllerDefinition(
        label="I/O FBL XY regulation",
        module_name="Controller_regulation",
        runner_type="xy_regulation",
    ),
    "2": ControllerDefinition(
        label="Trajectory tracking",
        module_name="Controller_traj",
        runner_type="trajectory_tracking",
    ),
    "3": ControllerDefinition(
        label="Theta / angular controller",
        module_name="Controller_angular",
        runner_type="theta",
    ),
}


def print_menu():
    print("\n=== RICObot controller launcher ===")

    for key, controller in CONTROLLERS.items():
        print(f"  {key} = {controller.label}")

    print("  q = quit")


def choose_controller():
    while True:
        print_menu()
        raw_choice = input("Select controller: ").strip().lower()

        if raw_choice in {"q", "quit", "exit"}:
            return None

        if raw_choice in CONTROLLERS:
            return CONTROLLERS[raw_choice]

        print("[!] Invalid controller selection.")


def start_controller_session(controller_type, extra_log_metadata=None):
    ser = None

    try:
        odometry_type = prompt_odometry_type()
        ser = find_and_connect_pico()
        check_pico_ready(ser)
        zero_imu_yaw(ser)
        log_writer = create_controller_logger(controller_type, odometry_type, extra_metadata=extra_log_metadata)
        return ControllerSession(odometry_type, ser, log_writer)
    except Exception:
        if ser is not None and ser.is_open:
            send_velocity(ser, 0.0, 0.0)
            ser.close()
        raise


def close_controller_session(session):
    if session is None:
        return

    if session.ser.is_open:
        send_velocity(session.ser, 0.0, 0.0)
        session.ser.close()
        print("\n[*] Robot stopped and port closed safely.")

    if session.log_writer.close():
        print("[*] Log file closed.")


def start_control_clock():
    start_time = time.monotonic()
    return ControlClock(start_time, start_time, start_time)


def wait_for_next_cycle(clock):
    now = time.monotonic()

    if now < clock.next_update_time:
        time.sleep(min(SERIAL_IDLE_SLEEP_S, clock.next_update_time - now))
        return None

    clock.next_update_time = now + CONTROL_LOOP_PERIOD_S
    return now


def update_loop_timing(clock, now):
    elapsed = now - clock.start_time
    dt = now - clock.last_update_time
    clock.last_update_time = now
    return elapsed, dt


def run_xy_controller(
    controller_type,
    target,
    calculate_command,
    *,
    stop_on_target,
    control_memory=None,
    controller_mode="",
    extra_log_metadata=None,
):
    session = None
    robot_state = RobotState()
    control_memory = control_memory or ControlMemory()
    phase = "NAVIGATE"
    target = np.asarray(target, dtype=float)
    control = XYControl(
        phase=phase,
        robot_state=robot_state,
        target=target.copy(),
        controller_mode=controller_mode,
    )

    try:
        session = start_controller_session(controller_type, extra_log_metadata)
        clock = start_control_clock()

        print(f"[*] Controller type: {controller_type}")
        print(f"[*] Odometry type: {session.odometry_type}")

        while True:
            now = wait_for_next_cycle(clock)
            if now is None:
                continue

            serial_data = read_latest_pico_data(session.ser)
            data = serial_data["odom"]

            if serial_data["tof"] is not None:
                robot_state.tof_measurements[:] = np.asarray(serial_data["tof"], dtype=float)

            if data is None:
                time.sleep(SERIAL_IDLE_SLEEP_S)
                continue

            elapsed, dt = update_loop_timing(clock, now)
            steps_l, steps_r, yaw_deg = data
            steps = np.array([steps_l, steps_r], dtype=float)
            update_robot_odometry(robot_state, steps, yaw_deg, session.odometry_type)

            update_xy_control(control, phase, elapsed, dt, steps, yaw_deg, target)

            if phase == "NAVIGATE":
                calculate_command(control, control_memory)

                if stop_on_target and control.position_error < POSITION_TOLERANCE_M:
                    phase = "DONE"
                    control.phase = phase
                    stop_command(control)
                    reset_motion_memory(control_memory)
                    print(f"\n[*] Target B reached. Final theta: {math.degrees(robot_state.theta_rad):.2f} deg")
            else:
                stop_command(control)
                reset_motion_memory(control_memory)

            send_velocity(session.ser, control.cmd_wheel_steps_s[0], control.cmd_wheel_steps_s[1])
            session.log_writer.record(control)
            print_xy_status(control)

            if phase == "DONE":
                break

    except KeyboardInterrupt:
        print("\n\n[!] Manual interruption.")
    finally:
        close_controller_session(session)


def update_xy_control(control, phase, elapsed, dt, steps, yaw_deg, target):
    control.elapsed = elapsed
    control.dt = dt
    control.phase = phase
    control.steps[:] = steps
    control.yaw_deg = yaw_deg
    control.target[:] = target


def stop_command(control):
    control.cmd_wheel_steps_s[:] = 0.0
    control.cmd_wheel_mps[:] = 0.0
    control.cmd_unicycle[:] = 0.0

    if hasattr(control, "raw_unicycle"):
        control.raw_unicycle[:] = 0.0


def reset_motion_memory(control_memory):
    control_memory.prev_wheel_mps[:] = 0.0


def print_xy_status(control):
    robot_state = control.robot_state
    print(
        f"\r[{control.phase}] "
        f"Pos:({robot_state.position[0]:.3f}, {robot_state.position[1]:.3f}) "
        f"Th:{math.degrees(robot_state.theta_rad):.1f} deg "
        f"Yaw:{control.yaw_deg:.1f} deg "
        f"B:({control.point_b[0]:.3f}, {control.point_b[1]:.3f}) "
        f"e_x:{control.error[0]:.3f}, e_y:{control.error[1]:.3f} "
        f"Cmd(step/s): L={control.cmd_wheel_steps_s[0]:.0f}, R={control.cmd_wheel_steps_s[1]:.0f}    ",
        end="",
    )


def update_theta_control(
    control,
    elapsed,
    dt,
    steps,
    initial_steps,
    yaw_deg,
    odometry_type,
):
    delta_steps = steps - initial_steps

    control.elapsed = elapsed
    control.dt = dt
    control.steps[:] = steps
    control.delta_steps[:] = delta_steps
    control.distance = np.sum(delta_steps) * METERS_PER_STEP / 2.0
    control.delta_s = control.odom_state.delta_s
    control.yaw_deg = yaw_deg
    control.yaw_rad = math.radians(yaw_deg)
    control.control_theta_source = "wheel" if odometry_type == "WHEEL" else "imu"
    control.control_theta_rad = control.odom_state.theta_rad


def run_theta_controller(
    controller_type,
    calculate_command,
    *,
    target_distance,
    forward_speed_mps,
    yaw_reference_deg,
    control_memory=None,
):
    session = None
    control_memory = control_memory or ControlMemory()
    odom_state = RobotState()
    initial_steps = None

    try:
        session = start_controller_session(controller_type)
        clock = start_control_clock()
        yaw_reference_rad = math.radians(yaw_reference_deg)
        control = ThetaControl(
            odom_state=odom_state,
            yaw_reference_rad=yaw_reference_rad,
            v_feedforward_mps=forward_speed_mps,
        )

        print("\n[*] Yaw-hold feedforward test running... (Press CTRL+C to stop)")
        print(f"[*] Target distance: {target_distance:.3f} m")
        print(f"[*] Feedforward speed: {forward_speed_mps:.3f} m/s")
        print(f"[*] Yaw reference: {yaw_reference_deg:.2f} deg")
        print(f"[*] Control theta source: {'wheel odometry' if session.odometry_type == 'WHEEL' else 'IMU yaw'}\n")

        while True:
            now = wait_for_next_cycle(clock)
            if now is None:
                continue

            serial_data = read_latest_pico_data(session.ser)
            data = serial_data["odom"]

            if serial_data["tof"] is not None:
                odom_state.tof_measurements[:] = np.asarray(serial_data["tof"], dtype=float)

            if data is None:
                time.sleep(SERIAL_IDLE_SLEEP_S)
                continue

            elapsed, dt = update_loop_timing(clock, now)
            steps_l, steps_r, yaw_deg = data
            steps = np.array([steps_l, steps_r], dtype=float)

            if initial_steps is None:
                initial_steps = steps.copy()

            update_robot_odometry(odom_state, steps, yaw_deg, session.odometry_type)
            update_theta_control(
                control,
                elapsed,
                dt,
                steps,
                initial_steps,
                yaw_deg,
                session.odometry_type,
            )
            calculate_command(control, control_memory)

            if abs(control.distance) >= abs(target_distance):
                send_velocity(session.ser, 0.0, 0.0)
                print("\n[*] Target distance reached.")
                break

            send_velocity(session.ser, control.cmd_wheel_steps_s[0], control.cmd_wheel_steps_s[1])
            session.log_writer.record(control)
            print_theta_status(control)

    except KeyboardInterrupt:
        print("\n\n[!] Manual interruption.")
    finally:
        close_controller_session(session)


def print_theta_status(control):
    odom_state = control.odom_state
    print(
        f"\rDist:{control.distance:.3f} m "
        f"Yaw:{control.yaw_deg:.2f} deg "
        f"WheelTh:{math.degrees(odom_state.theta_wheel_rad):.2f} deg "
        f"e_yaw:{math.degrees(control.yaw_error_rad):.2f} deg "
        f"v:{control.cmd_unicycle[0]:.3f} m/s "
        f"omega:{control.cmd_unicycle[1]:.3f} rad/s "
        f"L:{control.cmd_wheel_steps_s[0]:.0f} R:{control.cmd_wheel_steps_s[1]:.0f} step/s   ",
        end="",
    )


def run_selected_controller(controller):
    module = importlib.import_module(controller.module_name)
    runner_type = controller.runner_type

    if runner_type == "xy_regulation":
        target = module.setup_interface()
        run_xy_controller(
            module.CONTROLLER_TYPE,
            target,
            module.calculate_io_linearization,
            stop_on_target=True,
        )
        return

    if runner_type == "trajectory_tracking":
        start_position, trajectory_mode = module.setup_interface()
        run_xy_controller(
            module.CONTROLLER_TYPE,
            start_position,
            module.calculate_trajectory_tracking,
            stop_on_target=False,
            controller_mode=trajectory_mode,
            extra_log_metadata={
                "trajectory_mode": trajectory_mode,
                "trajectory_b_point_p": module.TRAJECTORY_B_POINT_P,
            },
        )
        return

    if runner_type == "theta":
        target_distance, forward_speed_mps, yaw_reference_deg = module.setup_interface()
        run_theta_controller(
            module.CONTROLLER_TYPE,
            module.calculate_yaw_hold_command,
            target_distance=target_distance,
            forward_speed_mps=forward_speed_mps,
            yaw_reference_deg=yaw_reference_deg,
        )
        return

    raise ValueError(f"Unsupported runner type: {runner_type!r}")


def main():
    controller = choose_controller()

    if controller is None:
        print("[*] No controller selected.")
        return

    print(f"[*] Selected controller: {controller.label}")
    run_selected_controller(controller)


if __name__ == "__main__":
    main()
