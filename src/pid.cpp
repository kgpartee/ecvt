#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>


// SECTION: Global Variables
int _vel_setpoint = 0;
ESP32Encoder encoder;

void pid_loop_task(void *pvParameters);
void velocity_pid_loop_task(void *pvParameters);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);

    

    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here

}


#define ALPHA 0.8
#define D_ALPHA 0.8


void pid_loop_task(void *pvParameters)
{

    int result = 0;
    float integral = 0;

    int setpoint = 2048;
    int last_pos = read_pos();

    int last_result = 0;

    //moving average filter for the derivative
    float moving_average[5] = {0, 0, 0, 0, 0};  
    int moving_average_index = 0;

    while (1)
    {
        
        // change setpoint to follow a sin wave
        setpoint = 2048 + 512 * sin(millis() / 1000.0);

        int pos = read_pos() * ALPHA + pos * (1 - ALPHA);

        int error = setpoint - pos; // calculate the error

        float derivative = last_pos - pos; // Derivatice calculation
        last_pos = pos;
        moving_average[moving_average_index] = derivative;
        moving_average_index = (moving_average_index + 1) % 5;

        float sum = 0;
        for (int i = 0; i < 5; i++)
        {
            sum += moving_average[i];
        }
        derivative = sum / 5.0;


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
        
        // result = result * PWM_ALPHA + last_result * (1 - PWM_ALPHA);
        set_direction_speed(result);                                // set the motor speed based on the pid term

        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">pos_setpoint: %d\n", setpoint);
        Serial.printf(">Vel_error: %d\n", error);
        Serial.printf(">PWM: %d\n", result > 255 ? 255 : result < -255 ? -255 : result);
        Serial.printf(">derivative: %f\n", derivative * POS_Kd);
        Serial.printf(">integral: %f\n", integral * POS_Ki);
        Serial.printf(">encoder: %d\n", encoder.getCount());
        delay(1);
    }
}

