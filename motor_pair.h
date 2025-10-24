#ifndef _MOTOR_PAIR_H_
#define _MOTOR_PAIR_H_

#include "motor.h"
constexpr const double PI = 3.1415926535;

namespace ev3 {
    class MotorPair {
        ev3::Motor& left_motor;
        ev3::Motor& right_motor;
        double wheel_diameter;
        double axle_track;

    public:
        MotorPair(ev3::Motor& left_motor, ev3::Motor& right_motor, double wheel_diameter, double axle_track): left_motor(left_motor), right_motor(right_motor), wheel_diameter(wheel_diameter), axle_track(axle_track) {}
        
        void straight(double mm, bool_t blocking); // template statement
        void straight(double mm) { this->straight(mm, true); }
        void straight(int32_t mm) { this->straight(static_cast<double>(mm), true); }
        void straight(int32_t mm, bool_t blocking) { this->straight(static_cast<double>(mm), blocking); }
        void straight(double mm, bool_t blocking) {
            int32_t angle = static_cast<int32_t>(mm * 360. / (PI * this->wheel_diameter));
            left_motor.block_tick();
            right_motor.block_tick();
            left_motor.run_angle(angle, false);
            right_motor.run_angle(angle, false);

            if (blocking) {
                while (!left_motor.done || !right_motor.done) {
                    tslp_tsk(10*1000);
                }
            }
        }

        void turn(double deg, bool_t blocking); // template statement
        void turn(double deg) { this->turn(deg, true); }
        void turn(int32_t deg) { this->turn(static_cast<double>(deg), true); }
        void turn(int32_t deg, bool_t blocking) { this->turn(static_cast<double>(deg), blocking); }
        void turn(double deg, bool_t blocking) {
            //int32_t angle = static_cast<int32_t>(this->axle_track*PI*deg/360. * 360. / (PI*this->wheel_diameter) );
            int32_t angle = static_cast<int32_t>(this->axle_track*deg/this->wheel_diameter );
            left_motor.block_tick();
            right_motor.block_tick();
            left_motor.run_angle(angle, false);
            right_motor.run_angle(-angle, false);

            if (blocking) {
                while (!left_motor.done || !right_motor.done) {
                    tslp_tsk(10*1000);
                }
            }
        }

        bool_t done() {
            return left_motor.done && right_motor.done;
        }
    
}
#endif // _MOTOR_PAIR_H_
