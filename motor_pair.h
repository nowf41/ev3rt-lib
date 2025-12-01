#ifndef _MOTOR_PAIR_H_
#define _MOTOR_PAIR_H_
#define DEBUG

#include "motor.h"
#include "ema.h"
#include "utils.h"
constexpr const double PI = 3.1415926535;

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
        unsigned int next_rec_at = 0;
        
    public:
        MotorPair(
            motor_port_t left_motor_port, motor_port_t right_motor_port,
            double wheel_diameter, double axle_track, // mm
            double max_accel // deg/s
        ): left_motor_port(left_motor_port), right_motor_port(right_motor_port), wheel_diameter(wheel_diameter), axle_track(axle_track), max_accel(max_accel) {
            assert(wheel_diameter > 0. && axle_track > 0. && max_accel > 0.);
            ev3_motor_config(left_motor_port, LARGE_MOTOR); ev3_motor_config(right_motor_port, LARGE_MOTOR);
            
            ev3_motor_reset_counts(left_motor_port); ev3_motor_reset_counts(right_motor_port);
            
            this->left_motor_actual_speed = this->right_motor_actual_speed = utils::Ema<double>(0.8, 0.);
        }

        void doTick() {
            if (blocking) return;

            uint64_t now_tim; get_tim(&now_tim);
            int32_t now_motor_angle_l = ev3_motor_get_counts(this->left_motor_port);
            int32_t now_motor_angle_r = ev3_motor_get_counts(this->right_motor_port);
            this->left_motor_actual_speed.add(static_cast<double>(now_motor_angle_l - this->latest_left_motor_angle) * (1000000./static_cast<double>(now_tim-this->latest_tim)));
            this->right_motor_actual_speed.add(static_cast<double>(now_motor_angle_r - this->latest_right_motor_angle) * (1000000./static_cast<double>(now_tim-this->latest_tim)));

            // === do feedback control ===


            // === judge the change of state ===
            
            // if the motor should start running
            if (this->state == kBroken && ((this->target_angle_l - now_motor_angle_l >= 0) ^ (this->target_speed_l >= 0)) == 0) {
                this->state = kSpeedIncrease;
            }

            // kIncrease -> kSpeedKeep is implemented at INCREASE function

            // if the motor should start speeding down
            if (this->state == kSpeedKeep && (abs(this->target_angle_l - now_motor_angle_l) <= abs(this->speeding_end_at - this->speeding_start_at) + this->stop_before * (this->left_motor_actual_speed.get()/this->specific_max_speed))) {
                this->state = kSpeedDecrease;
            }
            if (this->state == kSpeedIncrease && (abs(this->target_angle_l - now_motor_angle_l) <= abs(now_motor_angle_l - this->speeding_start_at) + this->stop_before * (this->left_motor_actual_speed.get()/this->specific_max_speed))) {
                this->state = kSpeedDecrease;
            }

            // kSpeedDecrease -> kAdjusting -> kBroken is implemented at each function
            if (this->target_speed_l > 0 && this->target_angle_l <= now_motor_angle_l || this->target_speed_l < 0 && this->target_angle_l >= now_motor_angle_l) {
                this->now_speed_l = this->now_speed_r = 0.;
            }
            
            
            // === do the actual calculation ===
            switch (state) {
                case kSpeedIncrease: {
                    double next_l = this->now_speed_l + this->max_accel * double(now_tim - this->latest_tim) * (this->target_speed_l >= 0 ? 1 : -1) / 1000000;
                    if (this->target_speed_l >= 0 && next_l >= this->target_speed_l || this->target_speed_l < 0 && next_l <= this->target_speed_l)  {
                        next_l = this->target_speed_l;
                        this->speeding_end_at = now_motor_angle_l;
                        this->state = kSpeedKeep;
                    }
                    this->now_speed_l = next_l;

                    double next_r = this->now_speed_r + this->max_accel * double(now_tim - this->latest_tim) * (this->target_speed_r >= 0 ? 1 : -1) / 1000000;
                    if (this->target_speed_r >= 0 && next_r >= this->target_speed_r || this->target_speed_r < 0 && next_r <= this->target_speed_r) {
                        next_r = this->target_speed_r;
                    }
                    this->now_speed_r = next_r;
                    break;
                }

                case kSpeedKeep: break;

                case kSpeedDecrease: {
                    double next_l = this->now_speed_l - this->max_accel * double(now_tim - this->latest_tim) * (this->target_speed_l >= 0 ? 1 : -1) / 1000000;
                    if (this->target_speed_l >= 0 && next_l <= this->min_speed || this->target_speed_l < 0 && next_l >= this->min_speed)  {
                        next_l = this->target_speed_l;
                        this->state = kAdjusting;
                    }
                    this->now_speed_l = next_l;

                    double next_r = this->now_speed_r - this->max_accel * double(now_tim - this->latest_tim) * (this->target_speed_r >= 0 ? 1 : -1) / 1000000;
                    if (this->target_speed_r >= 0 && next_r <= this->min_speed || this->target_speed_r < 0 && next_r >= this->min_speed) {
                        next_r = this->target_speed_r;
                    }
                    this->now_speed_r = next_r;
                    break;
                }

                case kAdjusting: {
                    if (this->target_speed_l >= 0 && now_motor_angle_l >= this->target_angle_l) {
                        ev3_motor_stop(this->left_motor_port, true);
                        now_speed_l = 0.;
                    }
                    if (this->target_speed_r >= 0 && now_motor_angle_r >= this->target_angle_r) {
                        ev3_motor_stop(this->right_motor_port, true);
                        now_speed_r = 0.;
                    }
                    if (now_speed_l == 0 && now_speed_r == 0) this->state = kBroken;
                    break;
                }

                case kBroken: break;
            }


            // === finalize ===
            ev3_motor_set_power(this->left_motor_port, utils::clamp<int>((this->now_speed_l+this->left_power_correction_val)*100/this->specific_max_speed, -100, 100));
            ev3_motor_set_power(this->right_motor_port, utils::clamp<int>((this->now_speed_r+this->right_power_correction_val)*100/this->specific_max_speed, -100, 100));
            this->latest_left_motor_angle = now_motor_angle_l;
            this->latest_right_motor_angle = now_motor_angle_r;
            this->latest_tim = now_tim;

            this->left_motor_rec[this->next_rec_at] = this->left_motor_actual_speed.get();
            this->right_motor_rec[this->next_rec_at] = this->right_motor_actual_speed.get();
            ++(this->next_rec_at);
            next_rec_at %= 1000;
        }

        void runForever(double speed) { // deg/s
            this->runTarget(speed, speed > 0 ? 1e7 : -1e7); // 実装の簡略化のため1e7をINFとして使用.
        }

        void runTarget(double speed, double target) {
            this->target_speed_l = this->target_speed_r = speed;
            this->target_angle_l = this->target_angle_r = target;
            this->state = kSpeedIncrease;
            get_tim(&(this->latest_tim));
        }

        /// @brief Runs the motor pair for specified angle.
        /// @param speed [deg/s] speed for the action. if it's under zero, the motor turns reversedly.
        /// @param angle [deg/s] angle for the action, will be set to |angle|.
        void runAngle(double speed, double angle) {
            this->target_speed_l = this->target_speed_r = speed;
            this->target_angle_l = ev3_motor_get_counts(this->left_motor_port) + utils::abs(angle) * (speed > 0 ? 1 : -1);
            this->target_angle_r = ev3_motor_get_counts(this->right_motor_port) + utils::abs(angle) * (speed > 0 ? 1 : -1);
            this->state = kSpeedIncrease;
            get_tim(&(this->latest_tim));
        }

        MotorControlStates get_now_state() {
            return this->state;
        }
    };
}
#endif // _MOTOR_PAIR_H_
