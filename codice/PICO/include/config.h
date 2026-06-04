// Logical motor pin definition.
// The identity test confirmed that these logical groups match the physical
// wheels: L is the physical left wheel, R is the physical right wheel.
#ifndef CONFIG_H
#define CONFIG_H

#define DIR_L_DRIVER 8
#define STEP_L_DRIVER 9
#define EN_L_DRIVER 10
#define FAULT_L_DRIVER 11

#define DIR_R_DRIVER 0
#define STEP_R_DRIVER 1
#define EN_R_DRIVER 2
#define FAULT_R_DRIVER 3

// Direction inversion for each logical motor.
// Positive speed is forward for both physical wheels with these settings.
#define INVERT_L_DIR true
#define INVERT_R_DIR false

// pin definition for Imu
#define IMU_SDA 16
#define IMU_SCL 17

// pin definition for Mux
#define MUX_SDA 18
#define MUX_SCL 19

// serial communication
#define USB_BAUD_RATE 115200

// Pico clock
#define PICO_OVERCLOCK_ENABLED 1
#define PICO_SYS_CLOCK_KHZ 300000

// Parameters for control
#define MAX_SPEED 10000.0
#define IMU_SAMPLE_RATE 5000 // us, BNO08x report interval

// ToF feature switches. Keep the code compiled, but make each expensive stage
// explicit so controller tests can isolate timing effects without commenting code.
#define TOF_INIT_ENABLED 1
#define TOF_READ_ENABLED 1
#define TOF_SERIAL_SEND_ENABLED 0

#define TOF_SENSOR_COUNT 8
#define TOF_GRID_SIZE 8 // set to 4 for 4x4, or 8 for 8x8
#define TOF_COLUMN_COUNT TOF_GRID_SIZE
#define TOF_RESOLUTION (TOF_GRID_SIZE * TOF_GRID_SIZE)
#define TOF_SAMPLE_FREQ 15 // hz, VL53L5CX report interval

#define IMU_READ_PERIOD_US 10000 // (us) 100 Hz
#define USB_SEND_STATE_PERIOD_US 10000 // (us) 100 Hz

#define TOF_READ_PERIOD_US 1000000/3 // (us) one ToF sensor poll every 10 ms when enabled
#define USB_SEND_TOF_PERIOD_US 200000 // (us) 5 Hz ToF serial stream when enabled

#endif
