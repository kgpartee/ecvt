#include "hall_sensor.h"
#include <Arduino.h>



void setup_hall() {
    pinMode(HALL_OUTPUT_PIN, INPUT);
    hall_count = 0;
    hall_trigger = false;
}

void read_hall(int voltage) {
    if (voltage > THRESHOLD && !hall_trigger) {
      hall_trigger = true;
      hall_count++;

    }
    else if (hall_trigger && voltage <= THRESHOLD) {
      hall_trigger = false;
    }
}

