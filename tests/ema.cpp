#include "CppUTest/TestHarness.h"

#include "utils.h"

TEST_GROUP(EMA) {
};

TEST(EMA, LongTest1) {
    utils::Ema<double> e(0.8, 0.);

    e.add(0.9); DOUBLES_EQUAL(0.2*0.9, e.get(), 1e-7);
    e.add(0.8); DOUBLES_EQUAL(0.2*0.9*0.8+0.2*0.8, e.get(), 1e-7);
    e.add(1.8); DOUBLES_EQUAL((0.2*0.9*0.8+0.2*0.8)*0.8+1.8*0.2, e.get(), 1e-7);
}
