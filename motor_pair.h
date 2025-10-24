#ifndef _MOTOR_PAIR_H_
#define _MOTOR_PAIR_H_

#include "motor.h"

namespace ev3 {
    class MotorPair {
        ev3::Motor& left_motor;
        ev3::Motor& right_motor;
        double wheel_diameter;
        double axle_track;

    public:
        MotorPair(ev3::Motor& left_motor, ev3::Motor& right_motor, double wheel_diameter, double axle_track): left_motor(left_motor), right_motor(right_motor), wheel_diameter(wheel_diameter), axle_track(axle_track) {}
}

#endif // _MOTOR_PAIR_H_
