#include "CppUTest/TestHarness.h"

#include "pidcalc.h"

TEST_GROUP(PidCalc) {
};

TEST(PidCalc, PropotionalTest1) {
    utils::PidCalc calc(1., 0., 0., 0.8);
    calc.add(0., 1U * 1000U * 1000U);
    DOUBLES_EQUAL(0., calc.get(), 1e-7);
    CHECK_EQUAL(calc.last_tim, 1U * 1000U * 1000U);
    calc.add(1., 2U * 1000U * 1000U);
    DOUBLES_EQUAL(1., calc.get(), 1e-7);
}

TEST(PidCalc, IntegralTest1) {
    utils::PidCalc calc(0., 1., 0., 0.8);
    calc.add(0., 1U * 1000U * 1000U);
    calc.add(1., 2U * 1000U * 1000U);
    DOUBLES_EQUAL(1. * (1. - 0.8), calc.get(), 1e-7);
}

TEST(PidCalc, DerivativeTest1) {
    utils::PidCalc calc(0., 0., 1., 0.8);
    calc.add(0., 1U * 1000U * 1000U);
    calc.add(1., 2U * 1000U * 1000U);
    DOUBLES_EQUAL(1., calc.get(), 1e-7);
}

TEST(PidCalc, CombinedTest1) {
    utils::PidCalc calc(1., 1., 1., 0.8);
    calc.add(0., 1U * 1000U * 1000U);
    calc.add(1., 2U * 1000U * 1000U);
    DOUBLES_EQUAL(1. + (1. * (1. - 0.8)) + 1., calc.get(), 1e-7);
}