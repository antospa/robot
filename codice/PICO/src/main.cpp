#include <Arduino.h>
#include <Wire.h>
#include "hardware/clocks.h"

#include "hardware/clocks.h"

#include "config.h"
#include "Motor_driver.hpp"
#include "Odometry.hpp"
#include "Comunication.hpp"
#include "IMU.hpp"
#include "TOF_mux.hpp"

Adafruit_BNO08x bno08x(-1);
SparkFun_VL53L5CX TOF_sensor[TOF_SENSOR_COUNT];

IMU_Measurement latestImu = {0.0f};
float imuYawOffsetDeg = 0.0f;

int16_t TOF_1D[TOF_SENSOR_COUNT][TOF_COLUMN_COUNT];
bool imuReady = false;
bool tofReady[TOF_SENSOR_COUNT] = {false};

volatile float targetMotorLeftStepsS = 0.0f;
volatile float targetMotorRightStepsS = 0.0f;
volatile uint32_t motorCommandSequence = 0;
volatile bool motorsReady = false;

void setMotorTargetFromCore0(float leftStepsS, float rightStepsS) {
    targetMotorLeftStepsS = leftStepsS;
    targetMotorRightStepsS = rightStepsS;
    motorCommandSequence++;
}

// ------------------------------------------------------------------------------ //

void setup() {
#if PICO_OVERCLOCK_ENABLED
    set_sys_clock_khz(PICO_SYS_CLOCK_KHZ, true);
#endif


    Serial.begin(USB_BAUD_RATE);
    delay(2000);

    Wire.setSDA(IMU_SDA);
    Wire.setSCL(IMU_SCL);
    Wire.begin();
    Wire.setClock(400000);

    Wire1.setSDA(MUX_SDA);
    Wire1.setSCL(MUX_SCL);
    Wire1.begin();
    Wire1.setClock(400000);

    imuReady = initIMU(bno08x);

#if TOF_INIT_ENABLED
    initTOF(TOF_sensor, Wire1, tofReady);
#endif
}

// ------------------------------------------------------------------------------ //

void setup1() {
    initMotors();
    setMotorTargetFromCore0(0.0f, 0.0f);
    motorsReady = true;
}

// ------------------------------------------------------------------------------ //

void loop1() {
    static uint32_t lastMotorCommandSequence = 0;

    if (!motorsReady) {
        return;
    }

    if (lastMotorCommandSequence != motorCommandSequence) {
        MotorsSetSpeed(targetMotorLeftStepsS, targetMotorRightStepsS);
        lastMotorCommandSequence = motorCommandSequence;
    }

    MotorsRun();
}

// ------------------------------------------------------------------------------ //

void loop() {
    static uint32_t lastImuReadUs = 0;
    static uint32_t lastUsbSendUs = 0;
    static uint32_t lastUsbSendUsTof = 0;
    static uint32_t lastTofReadUs = 0;

    uint32_t nowUs = micros();

    VelocityCommand newCmd;
    if (USB_receive(newCmd)) {
        if (newCmd.zero_imu) {
            imuYawOffsetDeg = latestImu.yaw;
        } else if (newCmd.status_request) {
            USB_send_status(imuReady, tofReady);
        } else {
            setMotorTargetFromCore0(newCmd.w_l, newCmd.w_r);
        }
    }

    // 1. Reading IMU
    if (nowUs - lastImuReadUs >= IMU_READ_PERIOD_US) {
        lastImuReadUs = nowUs;
        if (imuReady) {
            latestImu = readIMU(bno08x);
        }
    }

    // 2. Reading ToF sensors
    if (nowUs - lastTofReadUs >= TOF_READ_PERIOD_US) {


        lastTofReadUs = nowUs;
        #if TOF_READ_ENABLED
        readTOF(TOF_sensor, Wire1, tofReady, TOF_1D);

        #endif
        


    }

    // 3. USB communication for state
    if (nowUs - lastUsbSendUs >= USB_SEND_STATE_PERIOD_US) {
        MotorPositions currentPos = get_motors_step();
        float zeroedYawDeg = normalizeYawDeg(latestImu.yaw - imuYawOffsetDeg);
        USB_send_state(currentPos.left, currentPos.right, zeroedYawDeg);
        lastUsbSendUs = nowUs;
    }

    // 4. USB communication for TOF
    if (nowUs - lastUsbSendUsTof >= USB_SEND_TOF_PERIOD_US) {
        #if TOF_SERIAL_SEND_ENABLED
            USB_send_TOF(TOF_1D);
        #endif
        lastUsbSendUsTof = nowUs;
    }
}
