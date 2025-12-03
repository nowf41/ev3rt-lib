#ifndef _EMA_H_
#define _EMA_H_

namespace utils {
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

#endif // _EMA_H_

