#include <Arduino.h>
#include "motor.h"
#include "pins.h"

#define POS_Kp 3.5  //0.6
#define POS_Ki 0.2 //0.4
#define POS_Kd 15 //4.8
#define POS_MAX_I_TERM 50

// #define POS_Kp 0.37  //0.6
// #define POS_Ki 0 //0.4
// #define POS_Kd 0 //4.8
// #define POS_MAX_I_TERM 50

#define VEL_Kp 10
#define VEL_Ki 0
#define VEL_Kd 0.0
#define VEL_MAX_I_TERM 10000

void setup_pid_task();