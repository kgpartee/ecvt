//test

#include <Arduino.h>
#include "motor.h"
#include "hall_sensor.h"
#include "potentiometer.h"
#include "pins.h"

// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


// E-cvt automatic mode skeleton code
// Nominal restricted Kohler idle is 1700rpm, max is 3800rpm, and peak power is at 3000rpm.

// Positions range from 0 to 1000, and are scaled later to encoder input range.

int manualPosOne; 
int manualPosTwo; 



//ints for position pid gains;
//ints for engine rpm pid gains;
//int engine_RPM; // Calculated from hall sensor
// int engineRPMavg; //rpm after multisampling
// int numSamples = 0; // number of samples over which the average is taken
// int encoderReading; // ADC input from linear potentiometer
//float position; // Sheave position, scale it to between 0-1000.
int speed; 

int pid_loop_delay = 100000; // how often the timer triggers in us

// uint16_t pot_v; 
uint16_t total_v; //measuremetn across potentiometer used as resistor
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)






int setpoint_pos;//determined by engine rpm
// pid loop coefficients
int kp_pos = 1;
int ki_pos = 1;
int kd_pos = 1;

int kp_RPM = 1;
int ki_RPM = 1;
int kd_RPM = 1; 

float last_error_pos = 0;
float error_pos = 0;
float change_error_pos = 0;
float total_error_pos = 0;
float pid_term_pos = 0;

float last_error_RPM = 0;
float error_RPM = 0;
float change_error_RPM = 0;
float total_error_RPM = 0;
float pid_term_RPM = 0;
 
int v;

// if ypu dont get the naming conventions of my timer variables i don't either it's just the same conventions ilya uses 
// and in ilya we trust prayer hands emoji x3

// i dont really get the timer either so anyone reading this should expect Many Issues



// put function declarations here:

// Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
//void IRAM_ATTR hall_interrupt() {
    // v = analogRead(HALL_OUTPUT_PIN);
    // read_hall(v);
//    return;
//}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  setup_hall();
  setup_potentiometer();
  // pinMode(potPinTwo, INPUT);
  // pinMode(potPowerPin, OUTPUT);

  // pinMode(brake_pin, OUTPUT);

  // pinMode(calibration_button, INPUT);
  // pinMode(limit_switch_one, INPUT);
  // pinMode(limit_switch_one_power, OUTPUT);
  // pinMode(limit_switch_two, INPUT);
  // pinMode(limit_switch_two_power, OUTPUT);
  // pinMode(manualButtonOne, INPUT); 
  // pinMode(manualButtonTwo, INPUT); 
  // pinMode(buttonPower, OUTPUT);

  //initialize variable values, will def need to add more
  // digitalWrite(brake_pin, HIGH); // set brake off
 
  // digitalWrite(potPowerPin, HIGH); // power potentiometer at voltage that wont break MCU (3.3V)
  // digitalWrite(limit_switch_one_power, HIGH);
  // digitalWrite(limit_switch_two_power, HIGH);
  // digitalWrite(buttonPower, HIGH); 

  // interrupt set up
  // attachInterrupt(digitalPinToInterrupt(HALL_OUTPUT_PIN), hall_interrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(calibration_button), calibration_interrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(limit_switch_one), end_case_one, RISING);
  // attachInterrupt(digitalPinToInterrupt(limit_switch_two), end_case_two, RISING);

  //hardware timer setup
  // rpm_timer = timerBegin(1, 80, true);
  // timerAttachInterrupt(rpm_timer, &hall_reset, true);
  // timerAlarmWrite(rpm_timer, RPM_TIMER_DELAY, true);
  // timerAlarmEnable(rpm_timer);


  // pid_timer_RPM = timerBegin(2, 80, true);
  // timerAttachInterrupt(pid_timer_RPM, &pid_loop_RPM, true);
  // timerAlarmWrite(pid_timer_RPM, pid_loop_delay, true);
  // timerAlarmEnable(pid_timer_RPM);

  // pid_timer_pos = timerBegin(3, 80, true);
  // timerAttachInterrupt(pid_timer_pos, &pid_loop_pos, true);
  // timerAlarmWrite(pid_timer_pos, pid_loop_delay, true);
  // timerAlarmEnable(pid_timer_pos);

}

void loop() {
  // put your main code here, to run repeatedly:

}

// put function definitions here:








