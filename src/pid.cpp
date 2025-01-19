#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"

void pid_loop_task(void *pvParameters);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here
}

void pid_loop_task(void *pvParameters)
{

    int pid_term_pos = 0;
    int integral = 0;

    int setpoint = 2048;

    while (1)
    {
        //change setpoint to follow a sin wave
        setpoint = 2048 + 512 * sin(millis() / 1000.0);
        
        int pos = read_pos(); // find the current sheave position scaled 1-1000
        int error = setpoint - pos; // calculate the error
        
        integral += error; // I controller calculation
        if (integral > MAX_I_TERM)
        {
            integral = MAX_I_TERM;
        }
        else if (integral < -MAX_I_TERM)
        {
            integral = -MAX_I_TERM;
        }

        pid_term_pos = error * Kp + integral * Ki; // PI controller calculation


        set_direction_speed(pid_term_pos); // set the motor speed based on the pid term
        
        
        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">setpoint: %d\n", setpoint);
        Serial.printf(">error: %d\n", error);
        Serial.printf(">pidTerm: %d\n", pid_term_pos);
        Serial.printf(">integral: %d\n", integral);
        delay(10);
    }
}