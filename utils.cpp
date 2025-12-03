#include "utils.h"

template <typename T>
T utils::clamp(T v, T low, T high) {
    if (v < low) {
        return low;
    } else if (high < v) {
        return high;
    } else {
        return v;
    }
}

template<typename T>
T utils::abs(T val) {
    if (val >= static_cast<T>(0)) {
        return val;
    } else {
        return -val;
    }
}

template int utils::clamp<int>(int v, int low, int high);
template double utils::clamp<double>(double v, double low, double high);
template double utils::abs<double>(double val);
template long utils::abs<long>(long val);
