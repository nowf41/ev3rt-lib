#ifndef _PIDCALC_H_
#define _PIDCALC_H_

#include "utils.h"

#ifndef bool_t
#include <stdbool.h>
using bool_t = bool;
#endif

namespace utils {
    static inline double us_to_sec(uint64_t t) {
        return static_cast<double>(t) / 1000000.;
    }

    class PidCalc {
    public:
        double kp, ki, kd;
        double p_val;
        double i_val;
        double d_val;
        double recent_val = 0.;
        bool_t i_stat; // falseで積分演算を停止する.
        uint64_t last_tim;

        PidCalc() {}

        PidCalc(double kp, double ki, double kd, double begin_i_val) : kp(kp), ki(ki), kd(kd), p_val(0.), i_val(begin_i_val), d_val(0.), i_stat(true), last_tim(0) {
        }

        void add(double val, uint64_t tim) {
            if (last_tim == 0) last_tim = tim;
            double dt = us_to_sec(tim - last_tim);
            this->p_val = val * this->kp;
            if (this->i_stat == true) this->i_val += val * ki;
            this->d_val = (val - this->recent_val) * dt;
            this->recent_val = val;

            this->last_tim = tim;
        }

        void set_i_status(bool_t st) {
            this->i_stat = st;
        }

        double get() {
            return this->p_val + this->i_val + this->d_val * kd;
        }

        double get_i_val() {
            return this->i_val;
        }
    };
}

#endif // _PIDCALC_H_