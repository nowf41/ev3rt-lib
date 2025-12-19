#include "motor_pair.h"
#include "utils.h"

#include "ev3api.h"

ev3::MotorPair::MotorPair(
    motor_port_t left_motor_port, motor_port_t right_motor_port,
    double wheel_diameter, double axle_track, // mm
    double max_accel, // deg/s
    std::array<double, 1000> *left_motor_rec,
    std::array<double, 1000> *right_motor_rec
) {
    assert(wheel_diameter > 0. && axle_track > 0. && max_accel > 0.);

    this->left_motor_port = left_motor_port;
    this->right_motor_port = right_motor_port;
    this->wheel_diameter = wheel_diameter;
    this->axle_track = axle_track;
    this->max_accel = max_accel;
    this->left_motor_rec = left_motor_rec;
    this->right_motor_rec = right_motor_rec;
    this->next_rec_at = 0;

    ev3_motor_config(left_motor_port, LARGE_MOTOR);
    ev3_motor_config(right_motor_port, LARGE_MOTOR);
    ev3_motor_reset_counts(left_motor_port);
    ev3_motor_reset_counts(right_motor_port);

    this->left_motor_actual_speed = utils::Ema<double>(0.8, 0.);
    this->right_motor_actual_speed = utils::Ema<double>(0.8, 0.);

    this->pid_left = utils::PidCalc(0.5, 0.2, 0.); // kp, ki, kd
    this->pid_right = utils::PidCalc(0.5, 0.2, 0.); // kp, ki, kd
}

void ev3::MotorPair::doTick() {
    if (blocking) return;

    uint64_t now_tim; get_tim(&now_tim);
    int32_t now_motor_angle_l = ev3_motor_get_counts(this->left_motor_port);
    int32_t now_motor_angle_r = ev3_motor_get_counts(this->right_motor_port);
    this->left_motor_actual_speed.add(static_cast<double>(now_motor_angle_l - this->latest_left_motor_angle) * (1000000./static_cast<double>(now_tim-this->latest_tim)));
    this->right_motor_actual_speed.add(static_cast<double>(now_motor_angle_r - this->latest_right_motor_angle) * (1000000./static_cast<double>(now_tim-this->latest_tim)));

    // === do feedback control ===
    
    // add feedback
    pid_left.add(-(this->left_motor_actual_speed.get() - this->now_speed_l), now_tim);
    pid_right.add(-(this->right_motor_actual_speed.get() - this->now_speed_r), now_tim);

    // === judge the change of state ===
    
    // if the motor should start running
    if (this->state == kBroken && ((this->target_angle_l - now_motor_angle_l >= 0) ^ (this->target_speed_l >= 0)) == 0) {
        this->state = kSpeedIncrease;
    }

    // kIncrease -> kSpeedKeep is implemented at INCREASE function

    // if the motor should start speeding down
    if (this->state == kSpeedKeep && (utils::abs(this->target_angle_l - now_motor_angle_l) <= utils::abs(this->speeding_end_at - this->speeding_start_at) + this->stop_before * (this->left_motor_actual_speed.get()/this->specific_max_speed))) {
        this->state = kSpeedDecrease;
    }
    if (this->state == kSpeedIncrease && (utils::abs(this->target_angle_l - now_motor_angle_l) <= utils::abs(now_motor_angle_l - this->speeding_start_at) + this->stop_before * (this->left_motor_actual_speed.get()/this->specific_max_speed))) {
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
    ev3_motor_set_power(this->left_motor_port, utils::clamp<int>(static_cast<int>((this->now_speed_l+this->pid_left.get())*100./this->specific_max_speed), -100, 100));
    ev3_motor_set_power(this->right_motor_port, utils::clamp<int>(static_cast<int>((this->now_speed_r+this->pid_right.get())*100./this->specific_max_speed), -100, 100));
    this->latest_left_motor_angle = now_motor_angle_l;
    this->latest_right_motor_angle = now_motor_angle_r;
    this->latest_tim = now_tim;

    if (this->next_rec_at < 1000) {
        (*this->left_motor_rec)[this->next_rec_at] = this->left_motor_actual_speed.get();
        (*this->right_motor_rec)[this->next_rec_at] = this->pid_left.get();
        ++(this->next_rec_at);
    }
}

void ev3::MotorPair::runForever(double speed) {
    this->runTarget(speed, speed > 0 ? 1e7 : -1e7); // 実装の簡略化のため1e7をINFとして使用.
}

void ev3::MotorPair::runTarget(double speed, double target) {
    this->blocking = true;
    this->target_speed_l = speed;
    this->target_speed_r = speed;

    this->target_angle_l = target;
    this->target_angle_r = target;

    this->state = ev3::MotorControlStates::kSpeedIncrease;

    get_tim(&(this->latest_tim));

    this->blocking = false;
}

void ev3::MotorPair::runAngle(double speed, double angle) {
    this->blocking = true;

    this->target_speed_l = speed;
    this->target_speed_r = speed;
    
    this->target_angle_l = ev3_motor_get_counts(this->left_motor_port) + utils::abs(angle) * (speed > 0 ? 1 : -1);
    this->target_angle_r = ev3_motor_get_counts(this->right_motor_port) + utils::abs(angle) * (speed > 0 ? 1 : -1);
    
    this->state = ev3::MotorControlStates::kSpeedIncrease;
    get_tim(&(this->latest_tim));

    this->blocking = false;
}

ev3::MotorControlStates ev3::MotorPair::get_now_state() {
    return this->state;
}
