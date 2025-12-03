#include "ema.h"

#include <cassert>

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

template class utils::Ema<double>;
