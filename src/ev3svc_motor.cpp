// This file is to use shared 'static' function.
#include "ev3svc_motor.h"
#include "ev3svc_common.h"

// === Static variables ===

static brickinfo_t _brick_info;
const static volatile uint8_t* motor_ready;
const static volatile motor_data_t* motor_data;

static ev3svc::kMotorType _states[4] = {
  ev3svc::kMotorType::NONE,
  ev3svc::kMotorType::NONE,
  ev3svc::kMotorType::NONE,
  ev3svc::kMotorType::NONE
};

static int reference_count[4] = {0, 0, 0, 0};



// === Internal functions ===

int _get_motor_max_speed(ev3svc::kMotorType t) {
    switch (t) {
      case ev3svc::kMotorType::LARGE:
      case ev3svc::kMotorType::UNADJUSTED_LARGE:
        return 990;
      case ev3svc::kMotorType::MEDIUM:
      case ev3svc::kMotorType::UNADJUSTED_MEDIUM:
        return 1470;
      default:
        return 0;
    }
  }

static bool is_adjusted_motor_type(ev3svc::kMotorType type) {
  switch (type) {
    case ev3svc::kMotorType::NONE:
    case ev3svc::kMotorType::UNADJUSTED_LARGE:
    case ev3svc::kMotorType::UNADJUSTED_MEDIUM:
      return false;
      break;
    default:
      return true;
      break;
  }
}

const int MOTOR_FULL_SPEEDING_TIME_MS = 300;

inline void _ev3svc_motor_initialize() noexcept {
  ev3svc::svc::set_type(ev3svc::kMotorType::NONE, ev3svc::kMotorType::NONE, ev3svc::kMotorType::NONE, ev3svc::kMotorType::NONE);
  ev3svc::svc::get_brick_info(&_brick_info);
  motor_ready = _brick_info.motor_ready;
  motor_data = _brick_info.motor_data;
}



// === ev3svc::Motor implementation ===
ev3svc::Motor::Motor(kMotorPort port, kMotorType type) noexcept : port_(port), type_(type) {
  switch (type) {
    case kMotorType::NONE:
      assert(false); // this is an exceptional case: the program must not keep running if this happens.
      break;
    case kMotorType::UNADJUSTED_MEDIUM:
    case kMotorType::UNADJUSTED_LARGE:
      if (type == kMotorType::UNADJUSTED_LARGE) type = kMotorType::LARGE;
      else type = kMotorType::MEDIUM;
      // fall through
    default:
      ++reference_count[static_cast<int>(port)];
      if (_states[static_cast<int>(port)] == type) {
        // ok
      } else {
        _states[static_cast<int>(port)] = type;
        ev3svc::svc::set_type(_states[0], _states[1], _states[2], _states[3]);
      }
      break;
  }
}

ev3svc::Motor::~Motor() noexcept {
  reference_count[static_cast<int>(port_)]--;
  if (reference_count[static_cast<int>(port_)] == 0) _states[static_cast<int>(port_)] = kMotorType::NONE;
  ev3svc::svc::set_type(_states[0], _states[1], _states[2], _states[3]);
}

void ev3svc::Motor::stop(kMotorStopMode mode) const noexcept {
  ev3svc::svc::stop(port_, mode);
}

int ev3svc::Motor::get_tacho_count() const noexcept {
  int result = static_cast<int>(*(_brick_info.motor_data[static_cast<int>(port_)].tachoSensor));
  result -= tacho_count_d_;
  return result;
}

void ev3svc::Motor::reset_tacho_count() noexcept {
  tacho_count_d_ += get_tacho_count();
}

void ev3svc::Motor::reset_motor_hard() noexcept {
  ev3svc::svc::output_reset_tacho(port_);
  ev3svc::svc::input_reset_tacho(port_);
  tacho_count_d_ = 0;
}

void ev3svc::Motor::set_power(int power) const noexcept {
  if (power > 100) power = 100;
  else if (power < -100) power = -100;

  if (type_ == kMotorType::UNADJUSTED_LARGE || type_ == kMotorType::UNADJUSTED_MEDIUM) {
    ev3svc::svc::set_power_unadjusted(port_, static_cast<int8_t>(power));
  } else {
    ev3svc::svc::set_power_adjusted(port_, static_cast<int8_t>(power));
  }
}

ERROR_CODE ev3svc::Motor::run_angle_instant(unsigned int power, int angle, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(power));
#endif
  if (angle == 0) return;
  if (power > 100) power = 100;

  if (type_ == kMotorType::UNADJUSTED_LARGE || type_ == kMotorType::UNADJUSTED_MEDIUM) {
    ev3svc::svc::trapezoidal_step_unadjusted(port_, static_cast<int8_t>(power * (angle > 0 ? 1 : -1)), static_cast<int32_t>(0), static_cast<int32_t>(angle), static_cast<int32_t>(0), mode);
  } else {
    ev3svc::svc::trapezoidal_step_adjusted(port_, static_cast<int8_t>(power * (angle > 0 ? 1 : -1)), static_cast<int32_t>(0), static_cast<int32_t>(angle), static_cast<int32_t>(0), mode);
  }

  return ERROR_CODE::SUCCESS;
}

ev3svc::ERROR_CODE ev3svc::Motor::run_angle(unsigned int power, int angle, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(power));
#endif
  if (angle == 0) return ERROR_CODE::SUCCESS;
  if (power > 100) power = 100;

  int step1_need_time = MOTOR_FULL_SPEEDING_TIME_MS * power / 100; // <=300
  int step1_target_speed = _get_motor_max_speed(type_); // <= 2000
  int step1_need_angle = step1_need_time * step1_target_speed / 2 / 1000; // integral
  if (step1_need_angle * 2 < angle) {
    // adjust power
    int adjusted_power = static_cast<int>(std::round(sqrt(abs(angle) * 100. / MOTOR_FULL_SPEEDING_TIME_MS))); // integral
    power = adjusted_power;
    step1_need_angle = angle / 2;
  }

  // マイナス方向への進行もこのコードでOK.
  if (type_ == kMotorType::UNADJUSTED_LARGE || type_ == kMotorType::UNADJUSTED_MEDIUM) {
    ev3svc::svc::trapezoidal_step_unadjusted(port_, static_cast<int8_t>(power * (angle > 0 ? 1 : -1)), static_cast<int32_t>(step1_need_angle), static_cast<int32_t>(angle - step1_need_angle * 2), static_cast<int32_t>(step1_need_angle), mode);
  } else {
    ev3svc::svc::trapezoidal_step_adjusted(port_, static_cast<int8_t>(power * (angle > 0 ? 1 : -1)), static_cast<int32_t>(step1_need_angle), static_cast<int32_t>(angle - step1_need_angle * 2), static_cast<int32_t>(step1_need_angle), mode);
  }

  return ERROR_CODE::SUCCESS;
}

ev3svc::ERROR_CODE ev3svc::Motor::run_time_instant(int power, unsigned int time_ms, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(time_ms));
#endif
  if (time_ms == 0) return ERROR_CODE::SUCCESS;
  if (power > 100) power = 100;
  else if (power < -100) power = -100;

  if (type_ == kMotorType::UNADJUSTED_LARGE || type_ == kMotorType::UNADJUSTED_MEDIUM) {
    ev3svc::svc::trapezoidal_time_unadjusted(port_, static_cast<int8_t>(power), static_cast<uint32_t>(0), static_cast<uint32_t>(time_ms), static_cast<uint32_t>(0), mode);
  } else {
    ev3svc::svc::trapezoidal_time_adjusted(port_, static_cast<int8_t>(power), static_cast<uint32_t>(0), static_cast<uint32_t>(time_ms), static_cast<uint32_t>(0), mode);
  }

  return ERROR_CODE::SUCCESS;
}

ev3svc::ERROR_CODE ev3svc::Motor::run_time(int power, unsigned int time_ms, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(time_ms));
#endif
  if (time_ms == 0) return ERROR_CODE::SUCCESS;
  if (power > 100) power = 100;
  else if (power < -100) power = -100;

  int step1_time = MOTOR_FULL_SPEEDING_TIME_MS * power / 100;
  if (step1_time * 2 > time_ms) {
    power = time_ms / 2 * 100 / MOTOR_FULL_SPEEDING_TIME_MS;
    step1_time = time_ms / 2;
  }

  if (type_ == kMotorType::UNADJUSTED_LARGE || type_ == kMotorType::UNADJUSTED_MEDIUM) {
    ev3svc::svc::trapezoidal_time_unadjusted(port_, static_cast<int8_t>(power), static_cast<uint32_t>(step1_time), static_cast<uint32_t>(time_ms - step1_time * 2), static_cast<uint32_t>(step1_time), mode);
  } else {
    ev3svc::svc::trapezoidal_time_adjusted(port_, static_cast<int8_t>(power), static_cast<uint32_t>(step1_time), static_cast<uint32_t>(time_ms - step1_time * 2), static_cast<uint32_t>(step1_time), mode);
  }

  return ERROR_CODE::SUCCESS;
}

int ev3svc::Motor::get_angle() const noexcept {
  return static_cast<int>(*motor_data[static_cast<int>(port_)].tachoSensor);
}

int ev3svc::Motor::get_speed() const noexcept {
  return static_cast<int>(*motor_data[static_cast<int>(port_)].speed);
}

bool ev3svc::Motor::is_done() const noexcept {
  return ((*motor_ready & (1 << static_cast<int>(port_))) == 0);
}

void ev3svc::Motor::wait_for_complete() const noexcept {
  while (!is_done()) tslp_tsk(1 * 1000);
}

ev3svc::MotorPair::MotorPair(ev3svc::kMotorPort port_l, ev3svc::kMotorType type_l, ev3svc::kMotorPort port_r, ev3svc::kMotorType type_r) : port_l_(port_l), port_r_(port_r), type_l_(type_l), type_r_(type_r) {
  assert(port_l != port_r && is_adjusted_motor_type(type_l) && is_adjusted_motor_type(type_r)); // only adjusted motors are supported to sync
  _states[static_cast<int>(port_l_)] = type_l_;
  ++reference_count[static_cast<int>(port_l_)];
  _states[static_cast<int>(port_r_)] = type_r_;
  ++reference_count[static_cast<int>(port_r_)];
  ev3svc::svc::set_type(_states[0], _states[1], _states[2], _states[3]);
}

ev3svc::MotorPair::~MotorPair() noexcept {
  --reference_count[static_cast<int>(port_l_)];
  --reference_count[static_cast<int>(port_r_)];
  if (reference_count[static_cast<int>(port_l_)] == 0) _states[static_cast<int>(port_l_)] = kMotorType::NONE;
  if (reference_count[static_cast<int>(port_r_)] == 0) _states[static_cast<int>(port_r_)] = kMotorType::NONE;
  ev3svc::svc::set_type(_states[0], _states[1], _states[2], _states[3]);
}

ev3svc::ERROR_CODE ev3svc::MotorPair::run_angle_instant(unsigned int power, int angle, int turn_rate, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(power));
#endif
  if (angle == 0) return ERROR_CODE::SUCCESS;
  if (power > 100) power = 100;
  checkparam(-200 <= turn_rate && turn_rate <= 200);

  ev3svc::svc::sync_step_adjusted(port_l_, port_r_, static_cast<int8_t>(power * (angle > 0 ? 1 : -1)), static_cast<int16_t>(turn_rate), static_cast<int32_t>(angle), mode);
  return ERROR_CODE::SUCCESS;
}

ev3svc::ERROR_CODE ev3svc::MotorPair::run_time_instant(unsigned int power, unsigned int time_ms, int turn_rate, kMotorStopMode mode) const noexcept {
#ifndef EV3SVC_SKIP_UNSIGNED_CHECK
  checkparam(!does_top_bit_stand(time_ms));
  checkparam(!does_top_bit_stand(power));
#endif
  checkparam(power != 0);
  if (time_ms == 0) return ERROR_CODE::SUCCESS;
  if (power > 100) power = 100;
  checkparam(-200 <= turn_rate && turn_rate <= 200);

  ev3svc::svc::sync_time_adjusted(port_l_, port_r_, static_cast<int8_t>(power), static_cast<int16_t>(turn_rate), static_cast<uint32_t>(time_ms), mode);
}

bool ev3svc::MotorPair::is_done() const noexcept {
  return (*motor_ready & ((1 << static_cast<int>(port_l_)) | (1 << static_cast<int>(port_r_)))) == 0;
}

void ev3svc::MotorPair::wait_for_complete() const noexcept {
  while (!is_done()) tslp_tsk(1 * 1000);
}
