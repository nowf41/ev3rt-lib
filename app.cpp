#include "ev3api.h"
#include "app.h"
#include "motor_pair.h"

#include <array>

static std::array<double, 1000> ar1, ar2;

// TODO 制御目標値と出力とPID補正値とを見比べて、吹き飛んでいる原因を探す
// TODO PIDを遅らせる？

ev3::MotorPair pair(EV3_PORT_D, EV3_PORT_C, 56., 182., 990., &ar1, &ar2);
void main_task(intptr_t unused) {
    tslp_tsk(5000000);
    get_tim(&(pair.latest_tim));
    sta_cyc(MOTOR_CONTROL_CYC);
    tslp_tsk(20 * 1000);
    pair.runTarget(700., 2000.);

    while (pair.get_now_state() != ev3::kBroken) tslp_tsk(10 * 1000);
    tslp_tsk(100 * 1000);
    stp_cyc(MOTOR_CONTROL_CYC);
    ev3_motor_stop(EV3_PORT_D, true);
    ev3_motor_stop(EV3_PORT_C, true);

    for (int i = 0; i < 1000; i++) {
        char buf[40];
        sprintf(buf, "<%f %f>", ar1[i], ar2[i]);
        syslog(LOG_NOTICE, buf);
        tslp_tsk(10 * 1000);
    }
    syslog(LOG_NOTICE, "\nFIN\n");
}

void motor_control_task(intptr_t unused) {
    pair.doTick();
}
