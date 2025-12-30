#include "ev3svc.h"

void _app_init_task(intptr_t unused) {
  _ev3svc_motor_initialize();
}