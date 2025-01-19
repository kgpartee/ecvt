#include "motor.h"
#include <Arduino.h>
#include "pins.h"
// functions: motor setup and motor direction 

#define LEDC_CHANNEL 0

void setup_motor() {

  // put your setup code here, to run once:
  
 
  // pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL,1500,8);

  ledcAttachPin(PWM_PIN, LEDC_CHANNEL);
  // ledcAttach(PWM_PIN, 1500, 8);

}


  // motor speed between -1500 and 1500 ** double check, positive is sheaves move together 
void set_direction_speed(int motor_speed) {
  
  if (motor_speed < 0){
    digitalWrite(DIRECTION_PIN, LOW);
    motor_speed = -motor_speed;
  }
  else {
    digitalWrite(DIRECTION_PIN, HIGH);
  }

  if (motor_speed > 255) {
    motor_speed = 255;
  }

    // ledcWrite(LEDC_CHANNEL, 50);
    // analogWrite(PWM_PIN, 100);
}