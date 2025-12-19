#ifndef _PIDCALC_H_
#define _PIDCALC_H_

#include "utils.h"

namespace utils {
    static inline double us_to_sec(uint64_t t) {
        return static_cast<double>(t) / 1000000.;
    }

    class PidCalc {
        double kp, ki, kd;
        double p_val;
        double i_val;
        double d_val;
        double recent_val = 0.;
        uint64_t last_tim;

    public:
        PidCalc() {}

        PidCalc(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), p_val(0.), i_val(0.), d_val(0.), last_tim(0) {}

        void add(double val, uint64_t tim) {
            if (last_tim == 0) last_tim = tim;
            double dt = us_to_sec(tim - last_tim);
            this->p_val = val * this->kp;
            this->i_val += val * ki;
            this->d_val = (val - this->recent_val) * dt;
            this->recent_val = val;

            this->last_tim = tim;
        }

        double get() {
            return this->p_val + this->i_val + this->d_val * kd;
        }
    };
}

#endif // _PIDCALC_H_