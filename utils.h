#ifndef _UTILS_H_
#define _UTILS_H_

#include <cassert>

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
        if (val >= static_cast<T>(0)) {
            return val;
        } else {
            return -val;
        }
    }

    template <typename T>
    class Ema {
    private:
        T ret;
        T val;

    public:
        Ema() {}
        Ema(T ret, T val) : ret(ret), val(val) {}
        void add(T value) {
            this->val = this->val*(this->ret) + value*(static_cast<T>(1)-(this->ret));
        }
        T get() {
            return this->val;
        }
    };
}

#endif // _UTILS_H_
