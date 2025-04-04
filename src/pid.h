#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// #define POS_Kp 3.8 
#define POS_Kp 1.0 
#define POS_Ki 0 
#define POS_Kd 0 //30 
#define POS_MAX_I_TERM 50


#define RPM_Kp 0.001




void setup_pid_task();