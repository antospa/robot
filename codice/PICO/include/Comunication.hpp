#ifndef COMUNICATION_HPP
#define COMUNICATION_HPP

#include <Arduino.h>
#include <string.h>

#include "config.h"

struct VelocityCommand {
    float w_l;
    float w_r;
    bool zero_imu;
    bool status_request;
};

// ------------------------------------------------------------------------------ //

inline void USB_send_TOF(const int16_t TOF_1D[TOF_SENSOR_COUNT][TOF_COLUMN_COUNT]) {
    Serial.print("TOF,");
    Serial.print(TOF_SENSOR_COUNT);
    Serial.print(",");
    Serial.print(TOF_COLUMN_COUNT);
    Serial.print(",");

    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        for (int j = 0; j < TOF_COLUMN_COUNT; j++) {
            Serial.print(TOF_1D[i][j]);

            if (i < TOF_SENSOR_COUNT - 1 || j < TOF_COLUMN_COUNT - 1) {
                Serial.print(",");
            }
        }
    }

    Serial.println();
}

// ------------------------------------------------------------------------------ //

inline void USB_send_status(bool imu_ready, const bool tof_ready[TOF_SENSOR_COUNT]) {
    Serial.print("STATUS,IMU,");
    Serial.print(imu_ready ? 1 : 0);
    Serial.print(",TOF,");
    Serial.print(TOF_SENSOR_COUNT);
    Serial.print(",");

    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        Serial.print(tof_ready[i] ? 1 : 0);

        if (i < TOF_SENSOR_COUNT - 1) {
            Serial.print(",");
        }
    }

    Serial.println();
}

// ------------------------------------------------------------------------------ //

inline void USB_send_state(float left, float right, float theta) {
    Serial.print("ODOM,");
    Serial.print(left, 2);
    Serial.print(",");
    Serial.print(right, 2);
    Serial.print(",");
    Serial.println(theta, 2);
}

// ------------------------------------------------------------------------------ //

inline bool USB_receive(VelocityCommand &cmd) {
    static char buffer[32];
    static size_t index = 0;
    static bool receiving = false;

    bool gotNewMessage = false;

    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '<') {
            index = 0;
            receiving = true;
            continue;
        }

        if (receiving) {
            if (c == '>') {
                buffer[index] = '\0';
                receiving = false;
                gotNewMessage = true;
            } else {
                buffer[index++] = c;
            }

            if (index >= sizeof(buffer) - 1) {
                Serial.println("Error: Command too long");
                receiving = false;
            }
        }
    }

    if (gotNewMessage) {
        if (strcmp(buffer, "ZERO_IMU") == 0) {
            cmd.w_l = 0.0f;
            cmd.w_r = 0.0f;
            cmd.zero_imu = true;
            cmd.status_request = false;
            return true;
        }

        if (strcmp(buffer, "STATUS") == 0) {
            cmd.w_l = 0.0f;
            cmd.w_r = 0.0f;
            cmd.zero_imu = false;
            cmd.status_request = true;
            return true;
        }

        char *comma = strchr(buffer, ',');
        if (comma != nullptr) {
            *comma = '\0';
            cmd.w_l = atof(buffer);
            cmd.w_r = atof(comma + 1);
            cmd.zero_imu = false;
            cmd.status_request = false;
            return true;
        }
    }

    return false;
}

#endif
