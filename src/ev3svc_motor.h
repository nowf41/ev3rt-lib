#pragma once
#include "ev3svc_common.h"
#include "ev3svc_motor_svc.h"

#include <cmath>

#ifdef EV3SVC_TERMINATE_TASK_ON_INVALID_PARAMETER
#define checkparam(cond) assert(cond)
#else
#define checkparam(cond) if (!cond) {return ERROR_CODE::INVALID_PARAM;} 
#endif

/// @brief Must be called before using ev3svc::Motor class.
inline void _ev3svc_motor_initialize() noexcept;

namespace ev3svc {
  class Motor {
   private:
    kMotorPort port_;
    kMotorType type_;
    int32_t tacho_count_d_;

   public:
    /// @brief Initializes a motor class and connects the motor on the software.
    /// @param port the port that connects the motor
    /// @param type the type of the motor
    /// @warning Terminates the task when the type is NONE.
    Motor(kMotorPort port, kMotorType type) noexcept;

    /// @brief Disconnects the motor on the software.
    ~Motor() noexcept;

    /// @brief Stops the motor.
    /// @param mode the stop mode to use
    void stop(kMotorStopMode mode) const noexcept;
    
    /// @brief Gets the current tacho count of the motor.
    /// @return The current tacho count
    int get_tacho_count() const noexcept;

    /// @brief Resets the tacho count of the motor (on this class).
    void reset_tacho_count() noexcept;

    /// @brief Resets the tacho count of the motor (resets the variables on the driver).
    void reset_motor_hard() noexcept;

    /// @brief Sets the power of the motor.
    /// @param power The power level to set, ranging from -100 to 100.
    /// @note The power will be clamped to -100 to 100 if it exceeds the range.
    void set_power(int power) const noexcept;

    /// @brief Runs the motor for a specified angle by jumping to the target speed immediately.
    /// @param power The power level to set, ranging from 0 to 100.
    /// @param angle The angle to rotate the motor.
    /// @param mode The stop mode to use after running.
    ERROR_CODE run_angle_instant(unsigned int power, int angle, kMotorStopMode mode) const noexcept;

    /// @brief Runs the motor for a specified angle by accelerating and decelerating smoothly.
    /// @param power The power level to set, ranging from 0 to 100.
    /// @param angle The angle to rotate the motor.
    /// @param mode The stop mode to use after running.
    ERROR_CODE run_angle(unsigned int power, int angle, kMotorStopMode mode) const noexcept;

    /// @brief Runs the motor for a specified time by jumping to the target speed immediately.
    /// @param power The power level to set, ranging from -100 to 100.
    /// @param time_ms The time to run the motor in milliseconds.
    /// @param mode The stop mode to use after running.
    ERROR_CODE run_time_instant(int power, unsigned int time_ms, kMotorStopMode mode) const noexcept;

    /// @brief Runs the motor for a specified time by accelerating and decelerating smoothly.
    /// @param power The power level to set, ranging from -100 to 100.
    /// @param time_ms The time to run the motor in milliseconds.
    /// @param mode The stop mode to use after running.
    /// @note power will be clamped between -100 to 100 if it exceeds the range.
    ERROR_CODE run_time(int power, unsigned int time_ms, kMotorStopMode mode) const noexcept;

    /// @brief Gets the current tacho count of the motor.
    /// @return The current tacho count.
    int get_angle() const noexcept;

    /// @brief Gets the current speed of the motor.
    /// @return The current speed.
    int get_speed() const noexcept;

    /// @brief Checks if the motor has completed its current operation.
    /// @return True if the motor is done, false otherwise.
    bool is_done() const noexcept;

    /// @brief Waits until the motor has completed its current operation.
    void wait_for_complete() const noexcept;
  };

  class MotorPair {
    ev3svc::kMotorPort port_l_;
    ev3svc::kMotorPort port_r_;
    ev3svc::kMotorType type_l_;
    ev3svc::kMotorType type_r_;
    
    public:
    MotorPair(ev3svc::kMotorPort port_l, ev3svc::kMotorType type_l, ev3svc::kMotorPort port_r, ev3svc::kMotorType type_r);

    ~MotorPair() noexcept;

    /// @brief Runs the motors for a specified angle by jumping to the target speed immediately.
    /// @param power The power level to set, ranging from 0 to 100.
    /// @param angle The angle to run the motors.
    /// @param turn_rate The turn rate, ranging from -200 to 200. If below -100 or above 100, two motors runs in opposite directions, otherwise both motors run in the same direction, with different speed.
    /// @param mode The stop mode to use after running.
    /// @return An error code indicating the success or failure of the operation.
    ERROR_CODE run_angle_instant(unsigned int power, int angle, int turn_rate, kMotorStopMode mode) const noexcept;

    /// @brief Runs the motors for a specified time by jumping to the target speed immediately.
    /// @param power The power level to set, ranging from 0 to 100.
    /// @param time_ms The time to run the motors in milliseconds.
    /// @param turn_rate The turn rate, ranging from -200 to 200. If below -100 or above 100, two motors runs in opposite directions, otherwise both motors run in the same direction, with different speed.
    /// @param mode The stop mode to use after running.
    /// @note power will be clamped between 0 to 100 if it exceeds the range.
    ERROR_CODE run_time_instant(unsigned int power, unsigned int time_ms, int turn_rate, kMotorStopMode mode) const noexcept;

    /// @brief Checks if the motors have completed its current operation.
    /// @return True if both motors are done, false otherwise.
    bool is_done() const noexcept;

    /// @brief Waits until the motors have completed its current operation.
    void wait_for_complete() const noexcept;
  };
}
