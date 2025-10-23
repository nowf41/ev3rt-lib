#ifndef _EMA_H_
#define _EMA_H_

namespace utils {
    template <typename T>
    class Ema {
    private:
        T val;
        T ret;

    public:
        Ema(T ret): ret(ret) {
            assert(static_cast<T>(0) < ret && ret < static_cast<T>(1));
        }
        
        void add(T value) {
            this->val = this->val*this->ret + value*(1-this->ret);
        }

        T get() {
            return val;
        }
    };


}

#endif // _EMA_H_

