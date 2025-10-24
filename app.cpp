#include "ev3api.h"
#include "app.h"
#include "motor.h"

ev3::Motor motor_l(EV3_PORT_B, LARGE_MOTOR), motor_r(EV3_PORT_C, LARGE_MOTOR);
void main_task(intptr_t unused) {
    tslp_tsk(3*1000*1000);
    motor_r.run_angle(3600);
    motor_l.run_angle(3600);

    sta_cyc(MOTOR_CONTROL_CYC);
    tslp_tsk(60*1000*1000);
    stp_cyc(MOTOR_CONTROL_CYC);
}

void motor_control_task(intptr_t unused) {
    motor_r.do_tick();
    motor_l.do_tick();
}

