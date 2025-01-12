#include "hall_sensor.h"
#include <Arduino.h>
#include "pins.h"

int hall_count;
bool hall_trigger;
float engine_rpm;

void setup_hall() {
    pinMode(HALL_OUTPUT_PIN, INPUT);
    hall_count = 0;
    hall_trigger = false;
}

int get_hall_count(){

  return hall_count;
}

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

int rpm_calc(){
    engine_rpm = hall_count*60/(RPM_TIMER_DELAY*NUM_HALL_MAGNETS);
    hall_count = 0;
    return engine_rpm;
}

int get_rpm(){
  return engine_rpm;
}