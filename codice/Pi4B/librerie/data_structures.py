from dataclasses import dataclass, field

import numpy as np

from robot_config import TOF_VECTOR_SIZE


def vector2():
    return np.zeros(2, dtype=float)


def tof_vector():
    return np.full(TOF_VECTOR_SIZE, np.nan, dtype=float)


@dataclass
class RobotState:
    position: np.ndarray = field(default_factory=vector2)
    theta_rad: float = 0.0

    imu_position: np.ndarray = field(default_factory=vector2)
    theta_imu_rad: float = 0.0

    wheel_position: np.ndarray = field(default_factory=vector2)
    theta_wheel_rad: float = 0.0

    prev_steps: np.ndarray = field(default_factory=vector2)
    first_cycle: bool = True

    delta_s: float = 0.0
    delta_theta_wheel_rad: float = 0.0

    tof_measurements: np.ndarray = field(default_factory=tof_vector)


@dataclass(frozen=True)
class ControllerDefinition:
    label: str
    module_name: str
    runner_type: str


@dataclass
class ControllerSession:
    odometry_type: str
    ser: object
    log_writer: object


@dataclass
class ControlClock:
    start_time: float
    last_update_time: float
    next_update_time: float


@dataclass
class ControlMemory:
    prev_wheel_mps: np.ndarray = field(default_factory=vector2)


@dataclass
class XYControl:
    elapsed: float = 0.0
    dt: float = 0.0
    phase: str = ""
    steps: np.ndarray = field(default_factory=vector2)
    yaw_deg: float = 0.0
    robot_state: RobotState = None
    target: np.ndarray = field(default_factory=vector2)
    controller_mode: str = ""
    point_b: np.ndarray = field(default_factory=vector2)
    error: np.ndarray = field(default_factory=vector2)
    position_error: float = 0.0

    raw_unicycle: np.ndarray = field(default_factory=vector2)
    cmd_wheel_steps_s: np.ndarray = field(default_factory=vector2)
    cmd_unicycle: np.ndarray = field(default_factory=vector2)
    cmd_wheel_mps: np.ndarray = field(default_factory=vector2)


@dataclass
class ThetaControl:
    elapsed: float = 0.0
    dt: float = 0.0
    steps: np.ndarray = field(default_factory=vector2)
    delta_steps: np.ndarray = field(default_factory=vector2)
    distance: float = 0.0
    delta_s: float = 0.0
    odom_state: RobotState = None
    yaw_deg: float = 0.0
    yaw_rad: float = 0.0
    control_theta_source: str = ""
    control_theta_rad: float = 0.0
    yaw_reference_rad: float = 0.0
    yaw_error_rad: float = 0.0

    v_feedforward_mps: float = 0.0
    omega_feedback_rad_s: float = 0.0
    cmd_unicycle: np.ndarray = field(default_factory=vector2)
    cmd_wheel_mps: np.ndarray = field(default_factory=vector2)
    cmd_wheel_steps_s: np.ndarray = field(default_factory=vector2)
