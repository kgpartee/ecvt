#include "hall_sensor.h"
#include <Arduino.h>
#include "pins.h"

int hall_count;
bool hall_trigger;
volatile float engine_rpm;

// sets pin mode, initializes hall count and trigger variables
void setup_hall() {
    pinMode(HALL_OUTPUT_PIN, INPUT);
    hall_count = 0;
    hall_trigger = false;
}


// returns hall count
int get_hall_count(){

  return hall_count;
}


// reads whether the hall sensor value has passed the threshhold
void read_hall(int voltage) {
    if (voltage > THRESHOLD && !hall_trigger) {
      hall_trigger = true;
      hall_count++;


    }
    else if (hall_trigger && voltage <= THRESHOLD) {
      hall_trigger = false;

    }
    return;

}

// uses current hall count and delay since last calculation to calculate rpm
void rpm_calc(){
    engine_rpm = hall_count*60.0f/((1000000.0/RPM_TIMER_DELAY)*NUM_HALL_MAGNETS);
    hall_count = 0;
}

// returns rpm
float get_rpm(){
  return engine_rpm;
}