#pragma once

#include <stdint.h>

namespace ev3svc {
  enum class kMotorType {
    NONE = (int)kPortTypeInternal::TYPE_NONE,
    LARGE = (int)kPortTypeInternal::TYPE_TACHO, // NXT motor & EV3 L motor
    MEDIUM = (int)kPortTypeInternal::TYPE_MINITACHO, // EV3 M motor
    UNADJUSTED_LARGE = 200, // this value must not handed to supervisor-call.
    UNADJUSTED_MEDIUM = 201, // this value must not handed to supervisor-call.
  };
  
  enum class kMotorPort {
    A = 0,
    B = 1,
    C = 2,
    D = 3
  };

  enum class kPortTypeInternal {
  //  TYPE_KEEP                     =   0,  //!< Type value that won't change type in byte codes
    TYPE_NXT_TOUCH                =   1,  //!< Device is NXT touch sensor
    TYPE_NXT_LIGHT                =   2,  //!< Device is NXT light sensor
    TYPE_NXT_SOUND                =   3,  //!< Device is NXT sound sensor
    TYPE_NXT_COLOR                =   4,  //!< Device is NXT color sensor

    TYPE_TACHO                    =   7,  //!< Device is a tacho motor
    TYPE_MINITACHO                =   8,  //!< Device is a mini tacho motor
    TYPE_NEWTACHO                 =   9,  //!< Device is a new tacho motor

    TYPE_THIRD_PARTY_START        =  50,
    TYPE_THIRD_PARTY_END          =  99,

    TYPE_IIC_UNKNOWN              = 100,

    TYPE_NXT_TEST                 = 101,  //!< Device is a NXT ADC test sensor

    TYPE_NXT_IIC                  = 123,  //!< Device is NXT IIC sensor
    TYPE_TERMINAL                 = 124,  //!< Port is connected to a terminal
    TYPE_UNKNOWN                  = 125,  //!< Port not empty but type has not been determined
    TYPE_NONE                     = 126,  //!< Port empty or not available
    TYPE_ERROR                    = 127,  //!< Port not empty and type is invalid
  };

  enum class kMotorStopMode {
    COAST = 0,
    BREAK = 1
  };

  template <typename T>
  bool does_top_bit_stand(T v) {
    if (sizeof(T) == 0) return false;
    return (v >> (sizeof(T) * 8 - 1)) != 0;
  }

  enum class ERROR_CODE {
    SUCCESS = 0,
    INVALID_PARAM = -1,
  };
}
