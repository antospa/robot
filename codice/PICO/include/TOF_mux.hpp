#ifndef TOF_MUX_HPP
#define TOF_MUX_HPP

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "config.h"

#ifndef TOF_DEBUG_PRINT_MATRIX
#define TOF_DEBUG_PRINT_MATRIX 0
#endif

// ------------------------------------------------------------------------------ //

inline void printTOFMatrix(int sensor_index, const int16_t matrix[TOF_RESOLUTION]) {
    Serial.print("TOF ");
    Serial.print(sensor_index);
    Serial.println(" rotated matrix:");

    for (int row = 0; row < TOF_GRID_SIZE; row++) {
        for (int col = 0; col < TOF_GRID_SIZE; col++) {
            Serial.print(matrix[(row * TOF_GRID_SIZE) + col]);

            if (col < TOF_GRID_SIZE - 1) {
                Serial.print("\t");
            }
        }

        Serial.println();
    }
}

// ------------------------------------------------------------------------------ //

inline void initTOF(SparkFun_VL53L5CX sensor[TOF_SENSOR_COUNT], TwoWire &i2c_bus, bool tof_ready[TOF_SENSOR_COUNT]) {
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        tof_ready[i] = false;

        i2c_bus.beginTransmission(0x70);
        i2c_bus.write(1 << i);
        i2c_bus.endTransmission();

        if (!sensor[i].begin(0x29, i2c_bus)) {
            Serial.print("Failed to find VL53L5CX sensor ");
            Serial.println(i);
            delay(200);
        } else {
            sensor[i].setResolution(TOF_RESOLUTION);
            sensor[i].setRangingFrequency(TOF_SAMPLE_FREQ);
            sensor[i].startRanging();
            tof_ready[i] = true;

            Serial.print("Sensor ToF ");
            Serial.print(i);
            Serial.println(" ready");
        }
    }
}

// ------------------------------------------------------------------------------ //

// Round-robin: legge UN solo sensore. Preserva l'ultimo valore valido se !isDataReady.
inline void readTOFSingle(
    SparkFun_VL53L5CX sensor[TOF_SENSOR_COUNT],
    TwoWire &i2c_bus,
    const bool tof_ready[TOF_SENSOR_COUNT],
    int16_t TOF_1D[TOF_SENSOR_COUNT][TOF_COLUMN_COUNT],
    int i
) {
    if (!tof_ready[i]) {
        return;
    }

    i2c_bus.beginTransmission(0x70);
    i2c_bus.write(1 << i);
    i2c_bus.endTransmission();

    if (!sensor[i].isDataReady()) {
        return; // keep last valid reading
    }

    VL53L5CX_ResultsData data;
    sensor[i].getRangingData(&data);

    int16_t distances[TOF_RESOLUTION];

    for (int cell = 0; cell < TOF_RESOLUTION; cell++) {
        int original_row = cell / TOF_GRID_SIZE;
        int original_col = cell % TOF_GRID_SIZE;
        int rotated_index = ((TOF_GRID_SIZE - 1 - original_col) * TOF_GRID_SIZE) + original_row;
        distances[rotated_index] = data.distance_mm[cell];
    }

#if TOF_DEBUG_PRINT_MATRIX
    printTOFMatrix(i, distances);
#endif

    for (int col = 0; col < TOF_COLUMN_COUNT; col++) {
        int16_t minimum = distances[col];

        for (int row = 1; row < TOF_GRID_SIZE; row++) {
            int16_t current_value = distances[(row * TOF_GRID_SIZE) + col];
            if (current_value < minimum) {
                minimum = current_value;
            }
        }

        TOF_1D[i][TOF_COLUMN_COUNT - 1 - col] = minimum;
    }
}

// Wrapper bloccante (tutti e 8 i sensori). Evita nel loop principale.
inline void readTOF(
    SparkFun_VL53L5CX sensor[TOF_SENSOR_COUNT],
    TwoWire &i2c_bus,
    const bool tof_ready[TOF_SENSOR_COUNT],
    int16_t TOF_1D[TOF_SENSOR_COUNT][TOF_COLUMN_COUNT]
) {
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        readTOFSingle(sensor, i2c_bus, tof_ready, TOF_1D, i);
    }
}

#endif
