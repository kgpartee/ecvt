#include <Arduino.h>
#include "motor.h"
#include "pins.h"

#define Kp 0.4
#define Ki 0.01
#define Kd 0.0
#define MAX_I_TERM 10000

void setup_pid_task();