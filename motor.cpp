#include "motor.h"

#include "ev3api.h"
#include "utils.h"

ev3::Motor::Motor(motor_port_t port, motor_type_t type) {
    this->port = port;
    this->type = type;
    this->max_accel = 1000.;
    this->offset = 110;
    this->target_angle = 0;
    this->min_speed = 50.;
    this->speeding_down = false;
    this->now_angle = 0;
    this->speeding_start_angle = 0;
    this->speeding_end_angle = 0;
    this->last_cyc_tim = 0;
    this->speeding_up_done = false;
    this->done = false;
    this->is_speeding_ended = false;
    this->blocking = false;
    this->reverse = false;
    this->k_p = 0.7;
    this->k_i = 0.;
    this->i_val = 0.;
    this->is_i_disabled = false;
    this->now_speed = 0.;
    this->last_deg = 0;
    this->total_tick_count = 0;
    this->n = 0;
    this->specific_max_speed = (type == LARGE_MOTOR ? 960. : 1440.);
    this->target_speed = this->specific_max_speed * 0.9;
    this->ema = utils::Ema<double>(0.85, 0.);

    ev3_motor_config(port,type);
}

void ev3::Motor::do_tick() {
    if (this->blocking) return;
    int32_t now_angle = ev3_motor_get_counts(this->port);
    this->now_angle = now_angle;
    uint64_t now_tim; get_tim(&now_tim);

    ++this->total_tick_count;
    if (!this->done && this->total_tick_count % 10 == 0) {
        // N, now_speed_expected, now_speed_actual 
        printf("%i %f %f ", this->n, this->now_speed, this->ema.get());
    }

    // === change now state to fit to control values ===
    if (this->speeding_start_angle < this->target_angle) {
        this->reverse=false;
        if (this->done) this->now_speed = 0;
        else if (this->is_speeding_ended) {
            if (!this->done && this->target_angle <= now_angle) { // if done
                this->is_speeding_ended = false; // reset 
                printf("stopped at %i and tim %i\n", now_angle, int(now_tim/1000%1000000));
                this->done = true;
            }
        } else if (this->target_angle <= now_angle + (now_angle - this->last_deg)*5) {
            this->now_speed = 0;
            this->is_speeding_ended = true;
            ev3_motor_rotate(this->port, this->target_angle - now_angle, utils::clamp(int(this->now_speed), 0, 100), false);
        } else if (!this->speeding_up_done && (this->target_angle - now_angle) <= (now_angle - this->speeding_start_angle)) {
            this->speeding_up_done = true; this->speeding_end_angle = now_angle;
        } else if (!this->speeding_up_done && (this->target_angle - now_angle) > (now_angle - this->speeding_start_angle)) {
            double speed_new = this->now_speed + this->max_accel * (static_cast<double>(now_tim - this->last_cyc_tim)/(1000.*1000.));
            this->now_speed = utils::clamp(speed_new, 0., this->target_speed);
            if (speed_new >= this->target_speed) {
                this->speeding_up_done = true;
                this->speeding_end_angle = now_angle;
            }
        } else if (this->speeding_up_done && (this->target_angle - now_angle) <= static_cast<int32_t>(this->speeding_end_angle - this->speeding_start_angle + this->offset * (static_cast<double>(this->target_speed) / static_cast<double>(this->specific_max_speed)))) {
            this->speeding_down = true;
            double speed_new = this->now_speed - this->max_accel * (static_cast<double>(now_tim - this->last_cyc_tim)/(1000.*1000.));
            this->now_speed = utils::clamp(speed_new, this->min_speed, this->target_speed);
        }
    } else {
        this->reverse = true;
        if (this->done) this->now_speed = 0;
        else if (this->is_speeding_ended) {
            if (!this->done && this->target_angle >= now_angle) { // if done
                this->is_speeding_ended = false;
                printf("stopped at %i and tim %i\n", now_angle, int(now_tim/1000%1000000));
                this->done = true;
            }
        }
        else if (this->target_angle >= now_angle + (now_angle - this->last_deg)*5) {
            this->now_speed = 0;
            this->is_speeding_ended = true;
            ev3_motor_rotate(this->port, this->target_angle - now_angle, utils::clamp(int(this->now_speed), 0, 100), false);
        }
        else if (!this->speeding_up_done && (now_angle - this->target_angle) <= (this->speeding_start_angle - now_angle)) {
            this->speeding_up_done = true; this->speeding_end_angle = now_angle;
        } else if (!this->speeding_up_done && (now_angle - this->target_angle) > (this->speeding_start_angle - now_angle)) {
            double speed_new = this->now_speed + this->max_accel * (static_cast<double>(now_tim - this->last_cyc_tim)/(1000.*1000.));
            this->now_speed = utils::clamp(speed_new, 0., this->target_speed);
            if (speed_new >= this->target_speed) {
                this->speeding_up_done = true;
                this->speeding_end_angle = now_angle;
            }
        } else if (this->speeding_up_done && (now_angle - this->target_angle) <= static_cast<int32_t>(this->speeding_start_angle - this->speeding_end_angle + this->offset * (static_cast<double>(this->target_speed) / static_cast<double>(this->specific_max_speed)))) {
            this->speeding_down = true;
            double speed_new = this->now_speed - this->max_accel * (static_cast<double>(now_tim - this->last_cyc_tim)/(1000.*1000.));
            this->now_speed = utils::clamp(speed_new, this->min_speed, this->target_speed);
        }

    }

    // === PI control ===
    double e = (this->now_speed - ema.get());
    this->ema.add((now_angle - this->last_deg)*100);
    if (this->speeding_up_done && !this->speeding_down) this->i_val += e;
    double pi = this->k_p * e + this->k_i * this->i_val;

    // === send clamped values to the motor ===
    double total_speed = this->now_speed - pi;
    if (!this->is_speeding_ended) {
        ev3_motor_set_power(this->port, utils::clamp(int(total_speed * 100. / this->specific_max_speed * (this->reverse?-1:1)), -100, 100));
    }

    // === finalize ===
    this->last_cyc_tim = now_tim; 
    this->last_deg = now_angle;
}

void ev3::Motor::run_target(int32_t angle, bool_t blocking) {
    bool_t first_b = this->blocking;
    this->blocking = true;
    this->speeding_start_angle = this->now_angle;
    this->target_angle = angle;
    this->done = false;
    this->speeding_down = false;
    get_tim(&(this->last_cyc_tim));
    this->blocking = first_b;
    if (blocking) {
        while (!this->done) {
            tslp_tsk(10*1000);
        }
    }
}

void ev3::Motor::run_angle(int32_t angle, bool_t blocking) {
    this->run_target(this->now_angle + angle, blocking);
}

int32_t ev3::Motor::get_angle() {
    return this->now_angle;
}

void ev3::Motor::block_tick() { this->blocking = true; }
void ev3::Motor::unblock_tick() { this->blocking = false; }