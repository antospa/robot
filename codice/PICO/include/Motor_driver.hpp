#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <AccelStepper.h>
#include <math.h> 
#include "config.h"

// ----------------------------------------------------------------------------------------

// Motor objects declaration
inline AccelStepper motorL(1, STEP_L_DRIVER, DIR_L_DRIVER);
inline AccelStepper motorR(1, STEP_R_DRIVER, DIR_R_DRIVER);


// ----------------------------------------------------------------------------------------

// Motors init function
inline void initMotors() {
    motorL.setCurrentPosition(0);
    motorR.setCurrentPosition(0);
    motorL.setPinsInverted(INVERT_L_DIR, false, false);
    motorR.setPinsInverted(INVERT_R_DIR, false, false);

    motorL.setMaxSpeed(MAX_SPEED);
    motorR.setMaxSpeed(MAX_SPEED);
}

// Motors run function
inline void MotorsRun() {
    
    motorL.runSpeed();
    motorR.runSpeed();
}


inline void MotorsSetSpeed(float omega_l, float omega_r) {

    motorL.setSpeed(omega_l);
    motorR.setSpeed(omega_r);

}
#endif
