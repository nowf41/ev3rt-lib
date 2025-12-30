#pragma once
#include "platform_interface_layer.h"
#include "ev3svc_common.h"

#include <stdint.h>

namespace ev3svc {
  namespace svc {
    namespace {
      inline void call_motor_svc(const void* cmd, uint32_t size) {
        motor_command(cmd, size);
      }

      struct StepPower {
        uint8_t cmd;
        uint8_t nos;
        int8_t power;
        int32_t step1;
        int32_t step2;
        int32_t step3;
        uint8_t brake;
      };

      struct TimePower {
        uint8_t cmd;
        uint8_t nos;
        int8_t power;
        uint32_t time1;
        uint32_t time2;
        uint32_t time3;
        uint8_t brake;
      };

      struct StepSpeed {
        uint8_t cmd;
        uint8_t nos;
        int8_t speed;
        int32_t step1;
        int32_t step2;
        int32_t step3;
        uint8_t brake;
      };

      struct TimeSpeed {
        uint8_t cmd;
        uint8_t nos;
        int8_t speed;
        uint32_t time1;
        uint32_t time2;
        uint32_t time3;
        uint8_t brake;
      };

      struct StepSync {
        uint8_t cmd;
        uint8_t nos;
        int8_t speed;
        int16_t turn;
        int32_t step;
        uint8_t brake;
      };

      struct TimeSync {
        uint8_t cmd;
        uint8_t nos;
        int8_t speed;
        int16_t turn;
        uint32_t time;
        uint8_t brake;
      };
    }

    /// @brief Sets the motors' types.
    /// @param type_a the type of motor connected to port A
    /// @param type_b the type of motor connected to port B
    /// @param type_c the type of motor connected to port C
    /// @param type_d the type of motor connected to port D
    inline void set_type(ev3svc::kMotorType type_a, ev3svc::kMotorType type_b, ev3svc::kMotorType type_c, ev3svc::kMotorType type_d) {
      int8_t buf[5];
      buf[0] = 0xA1;
      buf[1] = static_cast<uint8_t>(type_a);
      buf[2] = static_cast<uint8_t>(type_b);
      buf[3] = static_cast<uint8_t>(type_c);
      buf[4] = static_cast<uint8_t>(type_d);

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Resets the tacho count if the motor is used NOT as a tacho sensor.
    /// @param port the port of the target motor
    inline void output_reset_tacho(ev3svc::kMotorPort port) {
      int8_t buf[2];
      buf[0] = 0xA2;
      buf[1] = 1 << static_cast<uint8_t>(port);

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Resets the tacho count if the motor is used as a tacho sensor.
    /// @param port the port of the target motor
    inline void input_reset_tacho(ev3svc::kMotorPort port) {
      int8_t buf[2];
      buf[0] = 0xA2;
      buf[1] = 1 << static_cast<uint8_t>(port);

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Stops the motor.
    /// @param port the port of the target motor
    /// @param mode the stop mode
    inline void stop(ev3svc::kMotorPort port, ev3svc::kMotorStopMode mode) {
      int8_t buf[3];
      buf[0] = 0xA3;
      buf[1] = 1 << static_cast<uint8_t>(port);
      buf[2] = static_cast<uint8_t>(mode);

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Sets the PWM of the motor. Doesn't start the motor.
    /// @param port the port of the target motor
    /// @param duty -100 <= value <= 100. PWM value
    inline void set_power_unadjusted(ev3svc::kMotorPort port, int8_t duty) {
      int8_t buf[3];
      buf[0] = 0xA4;
      buf[1] = 1 << static_cast<uint8_t>(port);
      buf[2] = duty;

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Sets the adjusted speed of the motor. Doesn't start the motor.
    /// @param port the port of the target motor
    /// @param speed -100 <= value <= 100. Speed value
    inline void set_power_adjusted(ev3svc::kMotorPort port, int8_t speed) {
      int8_t buf[3];
      buf[0] = 0xA5;
      buf[1] = 1 << static_cast<uint8_t>(port);
      buf[2] = speed;

      call_motor_svc(buf, sizeof(buf));
    }

    inline void start(ev3svc::kMotorPort port) {
      int8_t buf[2];
      buf[0] = 0xA6;
      buf[1] = 1 << static_cast<uint8_t>(port);

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Sets the polarity of the motor.
    /// @param port the port of the target motor
    /// @param polarity
    ///     - 0: flip
    ///     - 1: NON_INV
    ///     - -1: INV
    inline void set_polarity(ev3svc::kMotorPort port, int8_t polarity) {
      int8_t buf[3];
      buf[0] = 0xA7;
      buf[1] = static_cast<uint8_t>(1 << static_cast<uint8_t>(port));
      buf[2] = polarity;

      call_motor_svc(buf, sizeof(buf));
    }

    /// @brief Invalid command. Do nothing.
    inline void opOUTPUT_POSITION() {}

    /// @brief Runs the motor with trapezoidal profile (unadjusted power).
    /// @param port the port of the target motor
    /// @param power -100 <= value <= 100. Target PWM value
    /// @param step1 distance to accelerate [deg]
    /// @param step2 distance to keep constant speed [deg]
    /// @param step3 distance to decelerate [deg]
    /// @param brake stop mode after reaching the target position
    inline void trapezoidal_step_unadjusted(ev3svc::kMotorPort port, int8_t power, int32_t step1, int32_t step2, int32_t step3, ev3svc::kMotorStopMode brake) {
      StepPower cmd = {
        0xAC, // cmd
        static_cast<uint8_t>(1 << static_cast<uint8_t>(port)), // nos
        power,
        step1,
        step2,
        step3,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    /// @brief Runs the motor with trapezoidal profile (unadjusted power).
    /// @param port the port of the target motor
    /// @param power -100 <= value <= 100. Target PWM value
    /// @param time1 time to accelerate [msec]
    /// @param time2 time to keep constant speed [msec]
    /// @param time3 time to decelerate [msec]
    /// @param brake stop mode after reaching the target position
    inline void trapezoidal_time_unadjusted(ev3svc::kMotorPort port, int8_t power, uint32_t time1, uint32_t time2, uint32_t time3, ev3svc::kMotorStopMode brake) {
      TimePower cmd = {
        0xAD, // cmd
        static_cast<uint8_t>(1 << static_cast<uint8_t>(port)), // nos
        power,
        time1,
        time2,
        time3,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    /// @brief Runs the motor with trapezoidal profile (adjusted speed).
    /// @param port the port of the target motor
    /// @param speed -100 <= value <= 100. Target speed value
    /// @param step1 distance to accelerate [deg]
    /// @param step2 distance to keep constant speed [deg]
    /// @param step3 distance to decelerate [deg]
    /// @param brake stop mode after reaching the target position
    inline void trapezoidal_step_adjusted(ev3svc::kMotorPort port, int8_t speed, int32_t step1, int32_t step2, int32_t step3, ev3svc::kMotorStopMode brake) {
      StepSpeed cmd = {
        0xAE, // cmd
        static_cast<uint8_t>(1 << static_cast<uint8_t>(port)), // nos
        speed,
        step1,
        step2,
        step3,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    /// @brief Runs the motor with trapezoidal profile (adjusted speed).
    /// @param port the port of the target motor
    /// @param speed -100 <= value <= 100. Target speed value
    /// @param time1 time to accelerate [msec]
    /// @param time2 time to keep constant speed [msec]
    /// @param time3 time to decelerate [msec]
    /// @param brake stop mode after reaching the target position
    inline void trapezoidal_time_adjusted(ev3svc::kMotorPort port, int8_t speed, uint32_t time1, uint32_t time2, uint32_t time3, ev3svc::kMotorStopMode brake) {
      TimeSpeed cmd = {
        0xAF, // cmd
        static_cast<uint8_t>(1 << static_cast<uint8_t>(port)), // nos
        speed,
        time1,
        time2,
        time3,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    /// @brief Runs the motor synchronously with adjusted speed for two motors by jumping up the motor speed immediately
    /// @param port1 the port of the first target motor
    /// @param port2 the port of the second target motor
    /// @param speed -100 <= value <= 100. Target speed value
    /// @param turn -200 <= value <= 200. Turn ratio. For example: 100 speed + 30 turn = left 100, right 70
    ///     - value == 0: both motor rotates in the same actual speed
    ///     - value > 0: turn right
    ///     - value < 0: turn left
    ///     - |value| > 100: motors rotate in opposite direction
    /// @param step distance to move [deg]
    /// @param brake stop mode after reaching the target position
    inline void sync_step_adjusted(ev3svc::kMotorPort port1, ev3svc::kMotorPort port2, int8_t speed, int16_t turn, int32_t step, ev3svc::kMotorStopMode brake) {
      uint8_t nos = (1 << static_cast<uint8_t>(port1)) | (1 << static_cast<uint8_t>(port2));
      StepSync cmd = {
        0xB0, // cmd
        nos,
        speed,
        turn,
        step,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    /// @brief Runs the motor synchronously with adjusted speed for two motors by jumping up the motor speed immediately
    /// @param port1 the port of the first target motor
    /// @param port2 the port of the second target motor
    /// @param speed -100 <= value <= 100. Target speed value
    /// @param turn -200 <= value <= 200. Turn ratio. For example: 100 speed + 30 turn = left 100, right 70
    ///     - value == 0: both motor rotates in the same actual speed
    ///     - value > 0: turn right
    ///     - value < 0: turn left
    ///     - |value| > 100: motors rotate in opposite direction
    /// @param time time to run the motors [msec]
    /// @param brake stop mode after reaching the target position
    inline void sync_time_adjusted(ev3svc::kMotorPort port1, ev3svc::kMotorPort port2, int8_t speed, int16_t turn, uint32_t time, ev3svc::kMotorStopMode brake) {
      uint8_t nos = (1 << static_cast<uint8_t>(port1)) | (1 << static_cast<uint8_t>(port2));
      TimeSync cmd = {
        0xB1, // cmd
        nos,
        speed,
        turn,
        time,
        static_cast<uint8_t>(brake)
      };

      call_motor_svc(&cmd, sizeof(cmd));
    }

    inline ER get_brick_info(brickinfo_t* info) {
      return fetch_brick_info(info);
    }
  }
}