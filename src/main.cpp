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
float kp_pos = 0.1;
float ki_pos = 0;
float kd_pos = 0;

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

// // Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
// hw_timer_t * hall_timer = NULL; 
// portMUX_TYPE hall_timer_MUX = portMUX_INITIALIZER_UNLOCKED;

// void IRAM_ATTR hall_interrupt() {
//   portENTER_CRITICAL_ISR(&hall_timer_MUX);
//     v = analogRead(HALL_OUTPUT_PIN);
//     read_hall(v);

//   portEXIT_CRITICAL_ISR(&hall_timer_MUX);
// }


hw_timer_t * pid_timer_pos = NULL; 
portMUX_TYPE pid_timer_pos_MUX = portMUX_INITIALIZER_UNLOCKED;

void pos_pid_calc() {
  error_pos = setpoint_pos - get_position();

  change_error_pos = error_pos - last_error_pos; 
  total_error_pos += error_pos;
  pid_term_pos = (kp_pos * error_pos) + (ki_pos * total_error_pos) + (kd_pos * change_error_pos);

  last_error_pos = error_pos; 
}

void IRAM_ATTR pid_loop_pos() {
  portENTER_CRITICAL_ISR(&pid_timer_pos_MUX);
    //setpoint = pidTermRPM
    //current = position
    read_pos(); //find the current sheave position scaled 1-1000

    // //constrain travel before limit switches 
    // if(get_position() > HIGH_POS) {
    //   setpoint_pos = HIGH_POS; 
    //   //setDirection();
    //   pos_pid_calc();
    //   //ledcWrite(pwmPin, pidTermPos);
    //   set_direction_speed(pid_term_pos);
    // }
    // else if(get_position()<LOW_POS){
    //   setpoint_pos = LOW_POS; 
    //   //setDirection();
    //   pos_pid_calc();
    //   //ledcWrite(pwmPin, pidTermPos);
    //   set_direction_speed(pid_term_pos);
    // }
    // else if(get_rpm() < IDLE_RPM){
    //   setpoint_pos = IDLE_POS; 
    //   //setDirection();
    //   pos_pid_calc(); 
    //   //ledcWrite(pwmPin, pidTermPos);
    //   set_direction_speed(pid_term_pos);
    // }
    // unconstrained travel
    // else{
      setpoint_pos = 2048; 
      //setDirection();
      pos_pid_calc();
      //ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pid_term_pos);
    //}

  portEXIT_CRITICAL_ISR(&pid_timer_pos_MUX);
}


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
  // attachInterrupt(digitalPinToInterrupt(calibration_button), calibration_interrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(limit_switch_one), end_case_one, RISING);
  // attachInterrupt(digitalPinToInterrupt(limit_switch_two), end_case_two, RISING);

  // hall_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(hall_timer, &hall_interrupt, true);
  // timerAlarmWrite(hall_timer, HALL_TIMER_DELAY, true);
  // timerAlarmEnable(hall_timer);




  // pid_timer_RPM = timerBegin(2, 80, true);
  // timerAttachInterrupt(pid_timer_RPM, &pid_loop_RPM, true);
  // timerAlarmWrite(pid_timer_RPM, pid_loop_delay, true);
  // timerAlarmEnable(pid_timer_RPM);


  pid_timer_pos = timerBegin(3, 80, true);
  timerAttachInterrupt(pid_timer_pos, &pid_loop_pos, true);
  timerAlarmWrite(pid_timer_pos, pid_loop_delay, true);
  timerAlarmEnable(pid_timer_pos);

}

void loop() {
  // put your main code here, to run repeatedly:
  int pos = get_position();

  Serial.printf(">pos: %d\n", pos);
  Serial.printf(">pidTerm: %f\n", pid_term_pos);
  Serial.printf("analog: %d\n", analogRead(POT_PIN));
  //  ledcWrite(PWM_PIN, 100);
  // analogWrite(13, 100);
  // analogWrite(PWM_PIN, 100);
  
  delay(100);
}

// put function definitions here:

