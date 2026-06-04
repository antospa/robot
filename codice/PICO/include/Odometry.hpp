#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP
#include "Motor_driver.hpp"

struct MotorPositions {
    float left;
    float right;
};

inline MotorPositions get_motors_step() {
    MotorPositions pos;
    pos.left = motorL.currentPosition();
    pos.right = motorR.currentPosition();
    
    return pos;
}
#endif