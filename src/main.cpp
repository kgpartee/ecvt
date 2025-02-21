#include <Arduino.h>
#include "motor.h"
#include "hall_sensor.h"
#include "potentiometer.h"
#include "pins.h"
#include "pid.h"



// Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
hw_timer_t * hall_timer = NULL; 
portMUX_TYPE hall_timer_MUX = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR hall_interrupt() {
  portENTER_CRITICAL_ISR(&hall_timer_MUX);
    int v = analogRead(HALL_OUTPUT_PIN);

    read_hall(v);
  portEXIT_CRITICAL_ISR(&hall_timer_MUX);
}

// Interrupt to calculate rpm (each time a magnet passes the sensor)
hw_timer_t * rpm_timer = NULL; 
portMUX_TYPE rpm_timer_MUX = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR rpm_interrupt() {
  portENTER_CRITICAL_ISR(&rpm_timer_MUX);
    rpm_calc();
  portEXIT_CRITICAL_ISR(&rpm_timer_MUX);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  Serial.println("setup motor finished");
  setup_hall();
  Serial.println("setup hall finished");
  setup_potentiometer();
  Serial.println("setup potentiometer finished");

  setup_pid_task();
  Serial.println("setup pid finished");

  hall_timer = timerBegin(2, 80, true);
  timerAttachInterrupt(hall_timer, &hall_interrupt, true);
  timerAlarmWrite(hall_timer, HALL_TIMER_DELAY, true);
 

  Serial.println("setup hall timer finished");

  rpm_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(rpm_timer, &rpm_interrupt, true);
  timerAlarmWrite(rpm_timer, RPM_TIMER_DELAY, true);
  

  Serial.println("setup rpm timer finished");
  
  //timerAlarmEnable(rpm_timer);
  timerAlarmEnable(hall_timer);


  Serial.println("timers enabled");
  Serial.println("Setup Complete");

}

void loop() {
  uint16_t v = analogRead(HALL_OUTPUT_PIN);
  read_hall(v);
  uint16_t hall_count = get_hall_count();
  float rpm_print = get_rpm();
  Serial.printf(">hall_output: %d\n", v);
  Serial.printf(">hall_count: %d\n", hall_count);
  Serial.printf(">rpm: %f\n", rpm_print);
  delay(100);
}

// put function definitions here:

