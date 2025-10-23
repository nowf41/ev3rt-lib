#include "ev3api.h"
#include "app.h"
#include "utils.h"
#include "ema.h"

#include <algorithm>

struct motor_control_t {
    // control
    motor_port_t port = EV3_PORT_B;
    motor_type_t type = LARGE_MOTOR;

    // limits
    double specific_max_speed = 960.; // deg/s
    double max_accel = 1000.; // deg/s^2
    int32_t offset = 110; // stopping offset

    // control - target
    double target_speed = 960.; // deg/s
    int32_t target_angle = 0; // use big value to disable
    double min_speed = 50.;

    // control - internal data
    int32_t speeding_start_angle = 0; // deg
    int32_t speeding_end_angle = 0; // deg
    uint64_t last_cyc_tim = 0; // micro sec
    bool_t speeding_up_done = false;
    
    // control - PI
    utils::Ema<double> ema = utils::Ema<double>(0.85);
    double k_p = 0.;
    double k_i = 0.;
    double i_val = 0.;
    bool_t is_i_disabled = false;

    // now state
    double now_speed = 0.; // deg/s

    // TODO implement PI control
} m;

void main_task(intptr_t unused) {
    ev3_motor_config(m.port, m.type);
    tslp_tsk(3*1000*1000);
    // set some targets and initialize some variables of the struct

    m.speeding_start_angle = ev3_motor_get_counts(m.port);
    m.target_angle = m.speeding_start_angle + 360 * 10;

    get_tim(&(m.last_cyc_tim));
    sta_cyc(MOTOR_CONTROL_CYC);
    tslp_tsk(60*1000*1000);
    stp_cyc(MOTOR_CONTROL_CYC);
}

void motor_control_task(intptr_t unused) {
    int32_t now_angle = ev3_motor_get_counts(m.port);
    uint64_t now_tim; get_tim(&now_tim);

    // === change now state to fit to control values ===
    if (m.speeding_start_angle < m.target_angle) {
        if (m.target_angle <= now_angle) { // if done
            // TODO FIX: difference <10deg may happen
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

    // === send clamped values to the motor ===
    ev3_motor_set_power(m.port, utils::clamp(int(m.now_speed * 100. / m.specific_max_speed), -100, 100));
    
    // === finalize ===
    m.last_cyc_tim = now_tim;
}
