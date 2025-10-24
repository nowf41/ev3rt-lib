#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "ev3api.h"
#include "ema.h"
#include "utils.h"

#include <algorithm>

namespace ev3 {
    class Motor {
        // control
        motor_port_t port;
        motor_type_t type;

        // limits
        double specific_max_speed; // deg/s
        double max_accel; // deg/s^2
        int32_t offset; // stopping offset

        // control - target
        double target_speed; // deg/s
        int32_t target_angle; // use big value to disable
        double min_speed;

        // sensor datas
        int32_t now_angle;

        // control - internal data
        int32_t speeding_start_angle; // deg
        int32_t speeding_end_angle; // deg
        uint64_t last_cyc_tim; // micro sec
        bool_t speeding_up_done;
        bool_t done;
        
        // control - PI
        utils::Ema<double> ema;
        double k_p;
        double k_i;
        double i_val;
        bool_t is_i_disabled; // true when speeding

        // now state
        double now_speed; // deg/s

        // TODO implement PI control


    public:
        Motor(motor_port_t port, motor_type_t type): port(port), type(type), now_angle(0), max_accel(1000.), offset(110), target_angle(0), min_speed(50.), speeding_start_angle(0), speeding_end_angle(0), last_cyc_tim(0), speeding_up_done(false), k_p(0.), k_i(0.), i_val(0.), is_i_disabled(false), now_speed(0.) {
            ev3_motor_config(port,type);
            this->specific_max_speed = this->target_speed = (type == LARGE_MOTOR ? 960. : 1440.);
            this->ema = utils::Ema<double>(0.85, 0.);
        }

        void do_tick() {
            int32_t now_angle = ev3_motor_get_counts(this->port);
            this->now_angle = now_angle;
            uint64_t now_tim; get_tim(&now_tim);

            // === change now state to fit to control values ===
            if (this->speeding_start_angle < this->target_angle) {
                if (this->done) this->now_speed = 0;
                else if (!this->done && this->target_angle <= now_angle) { // if done 
                    this->now_speed = 0;
                    printf("stopped at %i\n", now_angle);
                    this->done = true;
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
                    double speed_new = this->now_speed - this->max_accel * (static_cast<double>(now_tim - this->last_cyc_tim)/(1000.*1000.));
                    this->now_speed = utils::clamp(speed_new, this->min_speed, this->target_speed);
                }
            }

            // === PI control ===
            // TODO impl

            // === send clamped values to the motor ===
            ev3_motor_set_power(this->port, utils::clamp(int(this->now_speed * 100. / this->specific_max_speed), -100, 100));
            
            // === finalize ===
            this->last_cyc_tim = now_tim; 
        }

        void run_target(int32_t angle) {
            this->speeding_start_angle = this->now_angle;
            this->target_angle = angle;
            this->done = false;
            get_tim(&(this->last_cyc_tim));
        }

        void run_angle(int32_t angle) {
            this->run_target(this->now_angle + angle);
        }

        int32_t get_angle() {
            return this->angle;
        }


    };

}

#endif // _MOTOR_H_
