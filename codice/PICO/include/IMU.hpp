  #include <Adafruit_BNO08x.h>
  #include <Arduino.h>




  // Variable declaration
  struct IMU_Measurement {
      float yaw;
      float pitch;
      float roll;

      float accelX;
      float accelY;
      float accelZ;

  };

  struct euler_t {
    float yaw;
    float pitch;
    float roll;
  } ypr;

  
  // ------------------------------------------------------------------------------ //

  float normalizeYawDeg(float yawDeg) {
    while (yawDeg > 180.0f) {
        yawDeg -= 360.0f;
    }
    while (yawDeg <= -180.0f) {
        yawDeg += 360.0f;
    }

    return yawDeg;
}


  // ------------------------------------------------------------------------------ //

  // Conversion from quaternion to Euler
  inline void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

      float sqr = sq(qr);
      float sqi = sq(qi);
      float sqj = sq(qj);
      float sqk = sq(qk);

      ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
      ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

      if (degrees) {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
      }
  }


  // ------------------------------------------------------------------------------ //

  // IMU init function
  inline bool initIMU(Adafruit_BNO08x  &bno08x) {
    while (!bno08x.begin_I2C()) {
      Serial.println("Failed to find BNO085 - retrying in 1 second...");
      delay(1000);
    }

    Serial.println("BNO085 Found!");

    //  if (!bno08x.enableReport(SH2_ACCELEROMETER, IMU_SAMPLE_RATE)) {
    //      Serial.println("Could not enable accelerometer");
    //    }

      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_SAMPLE_RATE)) {
          Serial.println("Could not enable rotation vector");
          return false;
      }

      return true;
    }

  // ------------------------------------------------------------------------------ //

  // IMU measurement reading function
  inline IMU_Measurement readIMU(Adafruit_BNO08x &bno08x) {
      static IMU_Measurement lastData = {0.0}; 
      
      sh2_SensorValue_t temp_packet;
      sh2_SensorValue_t last_packet;
      bool new_packet = false;

      while (bno08x.getSensorEvent(&temp_packet)) {
          if (temp_packet.sensorId == SH2_GAME_ROTATION_VECTOR) {
              last_packet = temp_packet; 
              new_packet = true;
          }
      }
      if (new_packet) {
          quaternionToEuler(last_packet.un.gameRotationVector.real, 
                            last_packet.un.gameRotationVector.i, 
                            last_packet.un.gameRotationVector.j, 
                            last_packet.un.gameRotationVector.k, 
                            &ypr, true);
          
          lastData.yaw = ypr.yaw;
      }
      
      return lastData;
  }
