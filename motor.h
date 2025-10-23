#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "ev3api.h"
#include "ema.h"
#include "utils.h"

#include <algorithm>

namespace ev3 {
    struct motor_control_t {
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

        // control - internal data
        int32_t speeding_start_angle; // deg
        int32_t speeding_end_angle; // deg
        uint64_t last_cyc_tim; // micro sec
        bool_t speeding_up_done;
        
        // control - PI
        utils::Ema<double> ema;
        double k_p;
        double k_i;
        double i_val;
        bool_t is_i_disabled;

        // now state
        double now_speed; // deg/s

        // TODO implement PI control

        motor_control_t(motor_port_t port, motor_type_t type, double specified_max_speed, double max_accel, int32_t offset, double target_speed, int32_t target_angle, double min_speed, int32_t speeding_start_angle, int32_t speeding_end_angle, uint64_t speeding_up_done, int32_t ema_ret, double k_p, double k_i, double i_val, bool_t is_i_disabled, double now_speed) {
            this->port = port;
            this->type = type;
            this->specified_max_speed = specified_max_speed;
            this->max_accel = max_accel;
            this->offset = offset;
            this->target_speed = target_speed;
            this->target_angle = target_angle;
            this->min_speed = min_speed;
            this->speeding_start_angle = speeding_start_angle;
            this->speeding_end_angle = speeding_end_angle;
            // TODO continue writing

        }
    };

    class Motor {
        motor_control_t m;

    public:
        Motor(motor_port_t port, motor_type_t type) {
            ev3_motor_config(port,type);
            this->m = motor_control_t(
                port,
                type,

                (type == LARGE_MOTOR ? 960. : 1440.),
                1000.,
                110,
                
                (type == LARGE_MOTOR ? 960. : 1440.),
                0,
                50.,

                0,
                0,
                0,
                false,

                utils::Ema<double>(0.85),
                0.,
                0.,
                0.,
                false,
                
                0.
            );
        }

        void do_tick() {
            motor_control_t& m = this->m;
            int32_t now_angle = ev3_motor_get_counts(m.port);
            uint64_t now_tim; get_tim(&now_tim);

            // === change now state to fit to control values ===
            if (m.speeding_start_angle < m.target_angle) {
                if (m.target_angle <= now_angle) { // if done 
                    m.now_speed = 0;
                    printf("stopped at %i\n", now_angle);
                    stp_cyc(MOTOR_CONTROL_CYC);
                } else if (!m.speeding_up_done && (m.target_angle - now_angle) <= (now_angle - m.speeding_start_angle)) {
                    m.speeding_up_done = true; m.speeding_end_angle = now_angle;
                } else if (!m.speeding_up_done && (m.target_angle - now_angle) > (now_angle - m.speeding_start_angle)) {
                    double speed_new = m.now_speed + m.max_accel * (static_cast<double>(now_tim - m.last_cyc_tim)/(1000.*1000.));
                    m.now_speed = utils::clamp(speed_new, 0., m.target_speed);
                    if (speed_new >= m.target_speed) {
                        m.speeding_up_done = true;
                        m.speeding_end_angle = now_angle;
                    }
                } else if (m.speeding_up_done && (m.target_angle - now_angle) <= static_cast<int32_t>(m.speeding_end_angle - m.speeding_start_angle + m.offset * (static_cast<double>(m.target_speed) / static_cast<double>(m.specific_max_speed)))) {
                    double speed_new = m.now_speed - m.max_accel * (static_cast<double>(now_tim - m.last_cyc_tim)/(1000.*1000.));
                    m.now_speed = utils::clamp(speed_new, m.min_speed, m.target_speed);
                }
            }

            // === PI control ===
            // TODO impl

            // === send clamped values to the motor ===
            ev3_motor_set_power(m.port, utils::clamp(int(m.now_speed * 100. / m.specific_max_speed), -100, 100));
            
            // === finalize ===
            m.last_cyc_tim = now_tim; 
        }

        void run_angle(int32_t angle) {
            this->m.speeding_start_angle = ev3_motor_get_counts(m.port);
            this->m.target_angle = m.speeding_start_angle + 360 * 10;
            get_tim(&(this->m.last_cyc_tim));
        }
    };

}

#endif // _MOTOR_H_
