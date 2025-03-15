#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// #define POS_Kp 3.8 
#define POS_Kp 4 
#define POS_Ki 0 
#define POS_Kd 30 
#define POS_MAX_I_TERM 50


#define RPM_Kp 0.0007




void setup_pid_task();