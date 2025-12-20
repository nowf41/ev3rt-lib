// マシン固有の値を定義する. 最終的には引数として取るか保存するように設定する.

#ifndef _MACHINE_VAL_H_
#define _MACHINE_VAL_H_

namespace machine_val {
    static const double wheel_diameter = 56.;
    static const double axle_track = 182.;
    static const double motor_max_speed = 800.;
    static const double i_val_start = -200.;
}
#endif