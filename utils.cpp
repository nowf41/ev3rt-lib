#include "utils.h"

#include <cassert>

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

template <typename T>
utils::Ema<T>::Ema() {}

template <typename T>
utils::Ema<T>::Ema(T ret, T val) {
    this->ret=ret;
    this->val=val;
    assert(static_cast<T>(0) < ret && ret < static_cast<T>(1));
}

template <typename T>
void utils::Ema<T>::add(T value) {
    this->val = this->val*(this->ret) + value*(static_cast<T>(1)-(this->ret));
}

template <typename T>
T utils::Ema<T>::get() {
    return this->val;
}

template int utils::clamp<int>(int v, int low, int high);
template double utils::clamp<double>(double v, double low, double high);
template double utils::abs<double>(double val);
template long utils::abs<long>(long val);
template class utils::Ema<double>;
