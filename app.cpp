#include "ev3api.h"
#include "app.h"
#include "motor.h"
#include "motor_pair.h"

ev3::Motor motor_l(EV3_PORT_D, LARGE_MOTOR);
ev3::Motor motor_r(EV3_PORT_C, LARGE_MOTOR);
void main_task(intptr_t unused) {
    motor_l.target_speed = motor_l.specific_max_speed * 0.8;
    motor_r.target_speed = motor_r.specific_max_speed * 0.8;
    motor_l.k_p = motor_r.k_p = 0.;
    motor_l.n = 0;
    motor_r.n = 1;
    
    ev3::MotorPair p(motor_l, motor_r, 56, 180); 
    tslp_tsk(3*1000*1000);
    sta_cyc(MOTOR_CONTROL_CYC);
    p.straight(1000., true);
    tslp_tsk(60*1000*1000);
    stp_cyc(MOTOR_CONTROL_CYC);
}

void motor_control_task(intptr_t unused) {
    motor_l.do_tick();
    motor_r.do_tick();
}

