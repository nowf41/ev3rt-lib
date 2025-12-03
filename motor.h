#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "ev3api.h"
#include "ema.h"
#include "utils.h"

namespace ev3 {
    class Motor {
    public:
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
        bool_t is_speeding_ended;
        bool_t blocking;
        bool_t reverse;
        
        // control - PI
        utils::Ema<double> ema; // now_speed: deg/s
        double k_p;
        double k_i;
        double i_val;
        bool_t is_i_disabled; // true when speeding
        bool_t speeding_down;

        // now state
        double now_speed; // deg/s

        // last state
        int32_t last_deg;
        // TODO implement PI control

        // DEBUG
        int32_t total_tick_count;
        int32_t n;
        
        Motor(motor_port_t port, motor_type_t type);

        void do_tick();

        void run_target(int32_t angle, bool_t blocking);

        void run_angle(int32_t angle, bool_t blocking);

        int32_t get_angle();

        void block_tick();
        void unblock_tick();
    };

}

#endif // _MOTOR_H_
