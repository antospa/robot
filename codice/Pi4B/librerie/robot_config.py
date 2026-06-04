import math

# ==============================================================================
# Serial communication
# ==============================================================================

PICO_USB_VID = 0x2E8A
USB_BAUD_RATE = 115200
SERIAL_TIMEOUT_S = 0.1
SERIAL_IDLE_SLEEP_S = 0.01
PICO_STATUS_TIMEOUT_S = 2.0

TOF_SENSOR_COUNT = 8
TOF_GRID_SIZE = 8
TOF_COLUMN_COUNT = TOF_GRID_SIZE
TOF_VECTOR_SIZE = TOF_SENSOR_COUNT * TOF_COLUMN_COUNT
REQUIRE_TOF_READY = False

CONTROL_LOOP_HZ = 30.0
CONTROL_LOOP_PERIOD_S = 1.0 / CONTROL_LOOP_HZ

# Motor wiring is fixed in the Pico config: logical left/right now match the
# physical left/right wheels. Raspberry-side commands must not be swapped.
SWAP_MOTOR_COMMANDS = False

# ==============================================================================
# Robot parameters
# ==============================================================================

WHEEL_RADIUS = 0.0305
MICROSTEPPING_FACTOR = 16.0
STEPS_PER_REV = 200.0 * MICROSTEPPING_FACTOR
TRACK_WIDTH = 0.108
METERS_PER_STEP = (2.0 * math.pi * WHEEL_RADIUS) / STEPS_PER_REV

# ==============================================================================
# B-point controller parameters
# ==============================================================================

B_POINT_P = 0.050
K_X = 25.0
K_Y = 25.0
POSITION_TOLERANCE_M = 0.010
MAX_SPEED_MPS = 0.25
MAX_SPEED_STEP_S = MAX_SPEED_MPS / METERS_PER_STEP

# ==============================================================================
# Trajectory controller parameters
# ==============================================================================

TRAJECTORY_B_POINT_P = 0.100
K_TRAJECTORY_X = 5.0
K_TRAJECTORY_Y = 5.0
TRAJECTORY_X_SPEED_MPS = 0.05
TRAJECTORY_Y_AMPLITUDE_M = 0.4
TRAJECTORY_PERIOD_S = 50.0

# ==============================================================================
# Acceleration ramp
# ==============================================================================

MAX_WHEEL_ACCEL_MPS2 = 0.25
MAX_WHEEL_ACCEL_STEP_S2 = MAX_WHEEL_ACCEL_MPS2 / METERS_PER_STEP
MAX_RAMP_DT_S = 0.1

# ==============================================================================
# Theta controller gains
# ==============================================================================

K_YAW_P = 25.0

# ==============================================================================
# Logging
# ==============================================================================

LOG_DATA_ENABLED = True
LOG_ROOT_DIRECTORY = "/home/rico_bot/RICObot/log"
LOG_DIRECTORY_NAME = "log"
LOG_FILE_PREFIX = "navigation"
