#define PWM_PIN 33 // SV on driver, sends pulses to the motor 
#define DIRECTION_PIN 25 // F/R on driver, controls the motor's direction

void setup_motor();
void set_direction_speed(int motor_speed);

