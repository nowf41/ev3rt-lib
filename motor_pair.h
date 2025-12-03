#ifndef _MOTOR_PAIR_H_
#define _MOTOR_PAIR_H_
#define DEBUG

#include "utils.h"

#include "ev3api.h"

namespace ev3 {
    enum MotorControlStates {
        kSpeedIncrease=0,
        kSpeedKeep=1,
        kSpeedDecrease=2,
        kAdjusting=3,
        kBroken=4
    };

    class MotorPair {
    public:
        // metadata
        motor_port_t left_motor_port, right_motor_port;
        double wheel_diameter, axle_track;
        double specific_max_speed = 990.; // deg/s. equals to 165rpm
        double min_speed = 100.;

        // Daikei speeding data
        double max_accel = 990.;
        MotorControlStates state = kBroken;
        int32_t speeding_start_at = 0.; // deg. 0 is an invalid value.
        int32_t speeding_end_at = 0; // deg. 0 is an invalid value.
        bool_t blocking = false;

        // target data
        double target_speed_l = 0.; // deg/s. can be minus value.
        double target_speed_r = 0.; // deg/s. can be minus value.
        double target_angle_l = 0.; // deg. this value will be added everytime.
        double target_angle_r = 0.; // deg. this value will be added everytime.
        double stop_before = 60.;

        // PI data
        double left_power_correction_val = 0.; // deg/s
        double right_power_correction_val = 0.; // deg/s

        // control data
        double now_speed_l = 0.; // deg/s. can be minus value.
        double now_speed_r = 0.; // deg/s. can be minus value.

        // actual value
        utils::Ema<double> left_motor_actual_speed;
        utils::Ema<double> right_motor_actual_speed;
        int32_t latest_left_motor_angle = 0;
        int32_t latest_right_motor_angle = 0;
        uint64_t latest_tim = 0;

        double left_motor_rec[1000];
        double right_motor_rec[1000];
        int next_rec_at = 0;
        
    public:
        MotorPair(
            motor_port_t left_motor_port, motor_port_t right_motor_port,
            double wheel_diameter, double axle_track, // mm
            double max_accel // deg/s
        );

        void doTick();

        void runForever(double speed);

        void runTarget(double speed, double target);

        /// @brief Runs the motor pair for specified angle.
        /// @param speed [deg/s] speed for the action. if it's under zero, the motor turns reversedly.
        /// @param angle [deg/s] angle for the action, will be set to |angle|.
        void runAngle(double speed, double angle);

        MotorControlStates get_now_state();
    };
}
#endif // _MOTOR_PAIR_H_
