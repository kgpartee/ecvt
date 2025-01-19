#include <Arduino.h>
#include "motor.h"
#include "hall_sensor.h"
#include "potentiometer.h"
#include "pins.h"
#include "pid.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  setup_hall();
  setup_potentiometer();

  setup_pid_task();


  Serial.println("Setup Complete");

}

void loop() {

  delay(100);
}

// put function definitions here:

