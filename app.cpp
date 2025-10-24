#include "ev3api.h"
#include "app.h"
#include "motor.h"
#include "motor_pair.h"

ev3::Motor motor_l(EV3_PORT_B, LARGE_MOTOR), motor_r(EV3_PORT_C, LARGE_MOTOR);
ev3::MotorPair p(motor_l, motor_r, ___, ___); 
void main_task(intptr_t unused) {
    tslp_tsk(3*1000*1000);
    sta_cyc(MOTOR_CONTROL_CYC);
    motor_l.block_tick();
    motor_r.block_tick();
    motor_l.run_angle(-360, false);
    motor_r.run_angle(-360, false);
    motor_l.unblock_tick();
    motor_r.unblock_tick();
    tslp_tsk(60*1000*1000);
    stp_cyc(MOTOR_CONTROL_CYC);
}

void motor_control_task(intptr_t unused) {
    motor_l.do_tick();
    motor_r.do_tick();
}

