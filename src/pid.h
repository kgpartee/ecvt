#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// #define POS_Kp 5  //0.6
// #define POS_Ki 0 //0.4
// #define POS_Kd 40 //4.8
// #define POS_MAX_I_TERM 50

#define POS_Kp 3.8 //0.6
#define POS_Ki 0 //0.4
#define POS_Kd 30 //30.5
#define POS_MAX_I_TERM 50


#define RPM_Kp 0.0025




void setup_pid_task();