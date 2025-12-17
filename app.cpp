#include "ev3api.h"
#include "app.h"
#include "motor_pair.h"

#include <array>

static std::array<double, 1000> ar1, ar2;

ev3::MotorPair pair(EV3_PORT_D, EV3_PORT_C, 56., 182., 990., &ar1, &ar2);
void main_task(intptr_t unused) {
    sta_cyc(MOTOR_CONTROL_CYC);
    tslp_tsk(5000000);
    get_tim(&(pair.latest_tim));
    pair.runTarget(-990., -3600.);

    while (pair.get_now_state() != ev3::kBroken) tslp_tsk(10 * 1000);
    tslp_tsk(100 * 1000);
    stp_cyc(MOTOR_CONTROL_CYC);

    for (int i = 0; i < 1000; i++) {
        char buf1[40];
        sprintf(buf1, "<%f  %f>", ar1[i], ar2[i]);
        syslog(LOG_NOTICE, buf1);
    }
    printf("\nFIN\n");
}

void motor_control_task(intptr_t unused) {
    pair.doTick();
}
