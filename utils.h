// THIS PROGRAM IS HEADER ONLY.
#ifndef _UTILS_H_
#define _UTILS_H_

namespace utils {

    template <typename T>
    T clamp(T v, T low, T high) {
        if (v < low) {
            return low;
        } else if (high < v) {
            return high;
        } else {
            return v;
        }
    }

    template <typename T>
    T abs(T val) {
        if (val >= 0) {
            return val;
        } else {
            return -val;
        }
    }

}

#endif // _UTILS_H_
