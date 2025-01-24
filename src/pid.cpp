#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"

// SECTION: Global Variables
int _vel_setpoint = 0;

void pid_loop_task(void *pvParameters);
void velocity_pid_loop_task(void *pvParameters);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here

    xTaskCreate(velocity_pid_loop_task,   // Function to implement the task
                "velocity_pid_loop_task", // A name just for humans
                10000,                    // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,                     // Parameters to pass to the task
                1,                        // This priority can be adjusted
                NULL);                    // Task handle. Not used here
}

void pid_loop_task(void *pvParameters)
{

    int result = 0;
    int integral = 0;

    int setpoint = 2048;
    int last_pos = read_pos();

    while (1)
    {
        // change setpoint to follow a sin wave
        setpoint = 2048 + 512 * sin(millis() / 1000.0);

        int pos = read_pos();       // find the current sheave position scaled 1-1000
        int error = setpoint - pos; // calculate the error
        int derivative = last_pos - pos;
        // int derivative = pos - last_pos; // calculate the derivative
        last_pos = pos;

        integral += error; // I controller calculation
        if (integral > POS_MAX_I_TERM)
        {
            integral = POS_MAX_I_TERM;
        }
        else if (integral < -POS_MAX_I_TERM)
        {
            integral = -POS_MAX_I_TERM;
        }

        result = error * POS_Kp + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation
        set_direction_speed(result);                                       // set the motor speed based on the pid term

        _vel_setpoint = result; // set the velocity setpoint based on the pid term

        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">pos_setpoint: %d\n", setpoint);
        // Serial.printf(">error: %d\n", error);
        // Serial.printf(">pidTerm: %d\n", result);
        // Serial.printf(">integral: %d\n", integral);
        delay(5);
    }
}

void velocity_pid_loop_task(void *pvParameters)
{
    int result = 0;
    int integral = 0;

    int vel_setpoint = 0;

    int last_pos = read_pos();

    while (1)
    {
        // change setpoint to follow a sin wave
        vel_setpoint = _vel_setpoint;

        int pos = read_pos();     // find the current sheave position scaled 1-1000
        int vel = pos - last_pos; // calculate the velocity
        last_pos = pos;

        int error = vel_setpoint - vel; // calculate the error

        integral += error; // I controller calculation
        if (integral > VEL_MAX_I_TERM)
        {
            integral = VEL_MAX_I_TERM;
        }
        else if (integral < -VEL_MAX_I_TERM)
        {
            integral = -VEL_MAX_I_TERM;
        }

        // result = error * VEL_Kp; // PI controller calculation
        result = error * VEL_Kp + integral * VEL_Ki; // PI controller calculation

        // set_direction_speed(result); // set the motor speed based on the pid term

        Serial.printf(">vel: %d\n", vel);
        Serial.printf(">vel_setpoint: %d\n", vel_setpoint);
        Serial.printf(">Vel_error: %d\n", error);
        Serial.printf(">PWM: %d\n", result);
        // Serial.printf(">integral: %d\n", integral);
        delay(10);
    }
}