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

    int analogValue = analogRead(POT_PIN);
    int pos = map(analogValue, 0, 4095, -140, 140);
    encoder.setCount(pos);
    

    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here

}


#define ALPHA 0.8
#define D_ALPHA 0.8

#define clamp(x, min, max) (x < min ? min : x > max ? max : x)
#define lerp(a, b, k) (a + (b - a) * k)

float smoothmin(float a, float b, float k) {
    float h = clamp(0.5 + 0.5 * (a - b) / k, 0, 1);
    return lerp(a, b, h) - k * h * (1 - h);
}

float smoothmax(float a, float b, float k) {
    return -smoothmin(-a, -b, k);
}

#define smoothclamp(x, min, max, k) smoothmin(smoothmax(x, min, k), max, k)

void pid_loop_task(void *pvParameters)
{

    float result = 0;
    float integral = 0;

    float setpoint = 0;
    float last_error = 0;


    //moving average filter for the derivative
    float moving_average[5] = {0, 0, 0, 0, 0};  
    int moving_average_index = 0;

    while (1)
    {
        
        // change setpoint to follow a sin wave
        // setpoint = 2048 + 512 * sin(millis() / 1000.0);
        setpoint = smoothclamp(70 * sin(millis() / 1000.0), -40, 40, 25);


        // int pos = read_pos() * ALPHA + pos * (1 - ALPHA);
        int pos = encoder.getCount();

        float error = setpoint - pos; // calculate the error

        float derivative = error - last_error; // Derivatice calculation
        last_error = error;
        moving_average[moving_average_index] = derivative;
        moving_average_index = (moving_average_index + 1) % 5;

        float sum = 0;
        for (int i = 0; i < 5; i++)
        {
            sum += moving_average[i];
        }
        derivative = sum / 5.0;


        integral += error; // I controller calculation
        integral = clamp(integral, -POS_MAX_I_TERM, POS_MAX_I_TERM);

        result = error * POS_Kp + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation
        
        set_direction_speed((int)result);                                // set the motor speed based on the pid term

        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">pos_setpoint: %f\n", setpoint);
        Serial.printf(">Vel_error: %f\n", error);
        Serial.printf(">PWM: %f\n", result > 255 ? 255 : result < -255 ? -255 : result);
        Serial.printf(">derivative: %f\n", derivative * POS_Kd);
        Serial.printf(">integral: %f\n", integral * POS_Ki);
        delay(1);
    }
}

