#ifndef _UTILS_H_
#define _UTILS_H_

namespace utils {
    template <typename T>
    T clamp(T v, T low, T high);

    template <typename T>
    T abs(T val);

    template <typename T>
    class Ema {
    private:
        T val;
        T ret;

    public:
        Ema();
        Ema(T ret, T val);
        void add(T value);
        T get();
    };
}

#endif // _UTILS_H_
