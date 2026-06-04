import csv
import math
import re
from pathlib import Path

import robot_config


DEFAULT_LOG_ROOT = Path("/home/rico_bot/RICObot/log")
EXPERIMENT_NUMBER_RE = re.compile(r"_exp_(\d+)\.csv$", re.IGNORECASE)

XY_FIELDNAMES = [
    "time_s",
    "dt_s",
    "phase",
    "steps_left",
    "steps_right",
    "yaw_deg",
    "x",
    "y",
    "wheel_x",
    "wheel_y",
    "x_imu",
    "y_imu",
    "theta_imu_rad",
    "theta_imu_deg",
    "x_wheel",
    "y_wheel",
    "theta_rad",
    "theta_deg",
    "wheel_theta_rad",
    "wheel_theta_deg",
    "target_x",
    "target_y",
    "b_point_x",
    "b_point_y",
    "error_x",
    "error_y",
    "position_error",
    "raw_v_mps",
    "raw_omega_rad_s",
    "cmd_v_mps",
    "cmd_omega_rad_s",
    "cmd_left_steps_s",
    "cmd_right_steps_s",
    "cmd_left_mps",
    "cmd_right_mps",
]

TRAJECTORY_TRACKING_FIELDNAMES = list(XY_FIELDNAMES)

THETA_FIELDNAMES = [
    "time_s",
    "dt_s",
    "steps_left",
    "steps_right",
    "delta_steps_left",
    "delta_steps_right",
    "distance",
    "delta_s",
    "x",
    "y",
    "wheel_x",
    "wheel_y",
    "wheel_theta_rad",
    "wheel_theta_deg",
    "yaw_deg",
    "yaw_rad",
    "x_imu",
    "y_imu",
    "theta_imu_rad",
    "theta_imu_deg",
    "x_wheel",
    "y_wheel",
    "theta_wheel_rad",
    "theta_wheel_deg",
    "control_theta_source",
    "control_theta_rad",
    "control_theta_deg",
    "control_error_deg",
    "control_error_rad",
    "yaw_reference_deg",
    "yaw_reference_rad",
    "yaw_error_deg",
    "yaw_error_rad",
    "v_feedforward_mps",
    "omega_feedback_rad_s",
    "cmd_v_mps",
    "cmd_omega_rad_s",
    "cmd_left_mps",
    "cmd_right_mps",
    "cmd_left_steps_s",
    "cmd_right_steps_s",
]

CONTROLLER_LOG_SCHEMAS = {
    "XY": {
        "fieldnames": XY_FIELDNAMES,
        "gain_names": ("K_X", "K_Y"),
    },
    "TRAJECTORY_TRACKING": {
        "fieldnames": TRAJECTORY_TRACKING_FIELDNAMES,
        "gain_names": ("K_TRAJECTORY_X", "K_TRAJECTORY_Y"),
    },
    "THETA": {
        "fieldnames": THETA_FIELDNAMES,
        "gain_names": ("K_YAW_P",),
    },
}


class CsvExperimentLogger:
    def __init__(self, file_handle, writer, file_path, metadata):
        self.enabled = True
        self.file_handle = file_handle
        self.writer = writer
        self.file_path = file_path
        self.metadata = metadata

    def record(self, control):
        controller_type = self.metadata["controller_type"]

        if controller_type == "XY":
            self.writerow(build_xy_row(control))
        elif controller_type == "TRAJECTORY_TRACKING":
            self.writerow(build_xy_row(control))
        elif controller_type == "THETA":
            self.writerow(build_theta_row(control))
        else:
            raise ValueError(f"Unsupported controller type for logging: {controller_type}")

    def writerow(self, row):
        complete_row = dict(row)
        complete_row.update(self.metadata)
        self.writer.writerow(complete_row)

    def close(self):
        if self.file_handle is not None and not self.file_handle.closed:
            self.file_handle.close()
            return True

        return False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class NullExperimentLogger:
    enabled = False
    file_path = None
    metadata = {}

    def record(self, control):
        return None

    def writerow(self, row):
        return None

    def close(self):
        return False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return None


def create_controller_logger(
    controller_type,
    odometry_type,
    fieldnames=None,
    gain_names=None,
    extra_gains=None,
    extra_metadata=None,
    log_root=None,
    config_module=robot_config,
):
    controller_token = normalize_controller_type(controller_type)
    schema = CONTROLLER_LOG_SCHEMAS.get(controller_token, {})
    selected_fieldnames = fieldnames if fieldnames is not None else schema.get("fieldnames")
    selected_gain_names = gain_names if gain_names is not None else schema.get("gain_names")

    if selected_fieldnames is None:
        raise ValueError(
            f"No CSV fieldnames configured for controller type {controller_type!r}. "
            "Pass fieldnames for a new controller or add it to CONTROLLER_LOG_SCHEMAS."
        )

    return create_csv_logger(
        controller_type=controller_token,
        odometry_type=odometry_type,
        fieldnames=selected_fieldnames,
        gain_names=selected_gain_names,
        extra_gains=extra_gains,
        extra_metadata=extra_metadata,
        log_root=log_root,
        config_module=config_module,
    )


def create_csv_logger(
    controller_type,
    odometry_type,
    fieldnames,
    gain_names=None,
    extra_gains=None,
    extra_metadata=None,
    log_root=None,
    config_module=robot_config,
):
    if not getattr(config_module, "LOG_DATA_ENABLED", True):
        return NullExperimentLogger()

    controller_token = normalize_controller_type(controller_type)
    odometry_token = normalize_odometry_type(odometry_type)
    gain_values = read_gain_values(gain_names, extra_gains, config_module)

    root_dir = resolve_log_root(log_root, config_module)
    log_dir = root_dir / f"{controller_token}_{odometry_token}"
    log_dir.mkdir(parents=True, exist_ok=True)

    experiment_number = next_experiment_number(log_dir)
    file_path = build_log_file_path(
        log_dir,
        controller_token,
        odometry_token,
        gain_values,
        experiment_number,
    )

    while file_path.exists():
        experiment_number += 1
        file_path = build_log_file_path(
            log_dir,
            controller_token,
            odometry_token,
            gain_values,
            experiment_number,
        )

    metadata = build_metadata(
        controller_token,
        odometry_token,
        gain_values,
        experiment_number,
        extra_metadata,
    )
    csv_fieldnames = merge_fieldnames(metadata.keys(), fieldnames)
    file_handle = file_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(file_handle, fieldnames=csv_fieldnames)
    writer.writeheader()

    print(f"[*] Logging to: {file_path}")
    return CsvExperimentLogger(file_handle, writer, file_path, metadata)


def build_xy_row(control):
    robot_state = control.robot_state

    return {
        "time_s": control.elapsed,
        "dt_s": control.dt,
        "phase": control.phase,
        "steps_left": control.steps[0],
        "steps_right": control.steps[1],
        "yaw_deg": control.yaw_deg,
        "x": robot_state.position[0],
        "y": robot_state.position[1],
        "wheel_x": robot_state.wheel_position[0],
        "wheel_y": robot_state.wheel_position[1],
        "x_imu": robot_state.imu_position[0],
        "y_imu": robot_state.imu_position[1],
        "theta_imu_rad": robot_state.theta_imu_rad,
        "theta_imu_deg": math.degrees(robot_state.theta_imu_rad),
        "x_wheel": robot_state.wheel_position[0],
        "y_wheel": robot_state.wheel_position[1],
        "theta_rad": robot_state.theta_rad,
        "theta_deg": math.degrees(robot_state.theta_rad),
        "wheel_theta_rad": robot_state.theta_wheel_rad,
        "wheel_theta_deg": math.degrees(robot_state.theta_wheel_rad),
        "target_x": control.target[0],
        "target_y": control.target[1],
        "b_point_x": control.point_b[0],
        "b_point_y": control.point_b[1],
        "error_x": control.error[0],
        "error_y": control.error[1],
        "position_error": control.position_error,
        "raw_v_mps": control.raw_unicycle[0],
        "raw_omega_rad_s": control.raw_unicycle[1],
        "cmd_v_mps": control.cmd_unicycle[0],
        "cmd_omega_rad_s": control.cmd_unicycle[1],
        "cmd_left_steps_s": control.cmd_wheel_steps_s[0],
        "cmd_right_steps_s": control.cmd_wheel_steps_s[1],
        "cmd_left_mps": control.cmd_wheel_mps[0],
        "cmd_right_mps": control.cmd_wheel_mps[1],
    }

def build_theta_row(control):
    odom_state = control.odom_state

    return {
        "time_s": control.elapsed,
        "dt_s": control.dt,
        "steps_left": control.steps[0],
        "steps_right": control.steps[1],
        "delta_steps_left": control.delta_steps[0],
        "delta_steps_right": control.delta_steps[1],
        "distance": control.distance,
        "delta_s": control.delta_s,
        "x": odom_state.position[0],
        "y": odom_state.position[1],
        "wheel_x": odom_state.wheel_position[0],
        "wheel_y": odom_state.wheel_position[1],
        "wheel_theta_rad": odom_state.theta_wheel_rad,
        "wheel_theta_deg": math.degrees(odom_state.theta_wheel_rad),
        "yaw_deg": control.yaw_deg,
        "yaw_rad": control.yaw_rad,
        "x_imu": odom_state.imu_position[0],
        "y_imu": odom_state.imu_position[1],
        "theta_imu_rad": odom_state.theta_imu_rad,
        "theta_imu_deg": math.degrees(odom_state.theta_imu_rad),
        "x_wheel": odom_state.wheel_position[0],
        "y_wheel": odom_state.wheel_position[1],
        "theta_wheel_rad": odom_state.theta_wheel_rad,
        "theta_wheel_deg": math.degrees(odom_state.theta_wheel_rad),
        "control_theta_source": control.control_theta_source,
        "control_theta_rad": control.control_theta_rad,
        "control_theta_deg": math.degrees(control.control_theta_rad),
        "control_error_deg": math.degrees(control.yaw_error_rad),
        "control_error_rad": control.yaw_error_rad,
        "yaw_reference_deg": math.degrees(control.yaw_reference_rad),
        "yaw_reference_rad": control.yaw_reference_rad,
        "yaw_error_deg": math.degrees(control.yaw_error_rad),
        "yaw_error_rad": control.yaw_error_rad,
        "v_feedforward_mps": control.v_feedforward_mps,
        "omega_feedback_rad_s": control.omega_feedback_rad_s,
        "cmd_v_mps": control.cmd_unicycle[0],
        "cmd_omega_rad_s": control.cmd_unicycle[1],
        "cmd_left_mps": control.cmd_wheel_mps[0],
        "cmd_right_mps": control.cmd_wheel_mps[1],
        "cmd_left_steps_s": control.cmd_wheel_steps_s[0],
        "cmd_right_steps_s": control.cmd_wheel_steps_s[1],
    }


def normalize_controller_type(controller_type):
    token = safe_token(controller_type)
    aliases = {
        "TRAJ": "TRAJECTORY_TRACKING",
        "TRAJECTORY": "TRAJECTORY_TRACKING",
        "TRAJECTORY_TRACKING": "TRAJECTORY_TRACKING",
        "TRACKING": "TRAJECTORY_TRACKING",
        "THETA": "THETA",
        "YAW": "THETA",
        "ANGULAR": "THETA",
        "XY": "XY",
    }
    return aliases.get(token, token)


def normalize_odometry_type(odometry_type):
    token = safe_token(odometry_type)

    if token in {"ODOM", "ODOMETRY", "WHEEL", "WHEELS", "RUOTE", "ENCODER", "ENCODERS"}:
        return "WHEEL"

    if token in {"IMU", "YAW", "INERTIAL"}:
        return "IMU"

    return token


def resolve_log_root(log_root, config_module):
    if log_root is not None:
        return Path(log_root)

    configured_root = getattr(config_module, "LOG_ROOT_DIRECTORY", None)

    if configured_root:
        return Path(configured_root)

    return DEFAULT_LOG_ROOT


def read_gain_values(gain_names, extra_gains, config_module):
    gain_values = {}

    for name in gain_names or ():
        if not hasattr(config_module, name):
            raise AttributeError(f"Gain {name!r} is not defined in robot_config.py")

        gain_values[name] = getattr(config_module, name)

    for name, value in (extra_gains or {}).items():
        gain_values[name] = value

    return gain_values


def next_experiment_number(log_dir):
    highest_number = 0

    for file_path in log_dir.glob("*.csv"):
        match = EXPERIMENT_NUMBER_RE.search(file_path.name)

        if match is not None:
            highest_number = max(highest_number, int(match.group(1)))

    return highest_number + 1


def build_log_file_path(log_dir, controller_token, odometry_token, gain_values, experiment_number):
    name_parts = [controller_token, odometry_token]
    gain_token = build_gain_token(gain_values)

    if gain_token:
        name_parts.append(gain_token)

    name_parts.append(f"exp_{experiment_number:03d}")
    return log_dir / ("_".join(name_parts) + ".csv")


def build_metadata(controller_token, odometry_token, gain_values, experiment_number, extra_metadata):
    metadata = {
        "controller_type": controller_token,
        "odometry_type": odometry_token,
    }

    for name, value in gain_values.items():
        metadata[name] = value

    metadata["experiment_number"] = experiment_number

    for name, value in (extra_metadata or {}).items():
        metadata[name] = value

    return metadata


def merge_fieldnames(metadata_fieldnames, data_fieldnames):
    merged = []

    for name in list(metadata_fieldnames) + list(data_fieldnames):
        if name not in merged:
            merged.append(name)

    return merged


def build_gain_token(gain_values):
    return "_".join(
        f"{safe_token(name)}_{format_value_for_filename(value)}"
        for name, value in gain_values.items()
    )


def safe_token(value):
    token = str(value).strip().upper().replace("-", "_").replace(" ", "_")
    token = re.sub(r"[^A-Z0-9_]+", "_", token)
    token = re.sub(r"_+", "_", token).strip("_")
    return token or "NA"


def format_value_for_filename(value):
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"

    if isinstance(value, float):
        token = f"{value:.6g}"
    else:
        token = str(value)

    token = token.strip().replace("-", "m").replace("+", "")
    token = token.replace(".", "p")
    token = re.sub(r"[^A-Za-z0-9_]+", "_", token)
    token = re.sub(r"_+", "_", token).strip("_")
    return token or "NA"
