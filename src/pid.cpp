#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"

// SECTION: Engine RPM Constants
// #define IDLE_RPM 500
// #define MAX_RPM 1200
// #define SETPOINT_RPM 800
#define IDLE_RPM 2000
#define MAX_RPM 3800
#define SETPOINT_RPM 3000

#define MAX_SHEAVE_SETPOINT 196 // 212
#define IDLE_SHEAVE_SETPOINT -92
#define LOW_SHEAVE_SETPOINT -50

// SECTION: Global Variables
int _vel_setpoint = 0;
ESP32Encoder encoder;

// SECTION: Function Prototypes
void pid_loop_task(void *pvParameters);
float calculate_setpoint(float rpm, float sheave_setpoint, float targetRPM);
float moving_average(float newVal, float *arr, int n, int *index);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);

    int analogValue = analogRead(POT_PIN);
    int pos = map(analogValue, 0, 4095, -230, 230);
    // 3835 = 146
    // 1361 = -176

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


// clamp function: clamps a value between a minimum and maximum
// lerp function: linear interpolation between two values
#define clamp(x, min, max) (x < min ? min : x > max ? max \
                                                    : x)
#define lerp(a, b, k) (a + (b - a) * k)

#define float_map(x, in_min, in_max, out_min, out_max) \
    (out_min + (out_max - out_min) * ((x - in_min) / (in_max - in_min)))


float smoothmin(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5 * (a - b) / k, 0, 1);
    return lerp(a, b, h) - k * h * (1 - h);
}

float smoothmax(float a, float b, float k)
{
    return -smoothmin(-a, -b, k);
}

#define smoothclamp(x, min, max, k) smoothmin(smoothmax(x, min, k), max, k)

void pid_loop_task(void *pvParameters)
{

    float result = 0;
    float integral = 0;

    float setpoint = 0;
    float last_error = 0;

    // moving average filter for the derivative
    float filter_array_derivative[5] = {0, 0, 0, 0, 0};
    int filter_index_derivative = 0;

// moving average filter for the rpm
#define FILTER_SIZE 250
    float filter_array_rpm[FILTER_SIZE];
    for (int i = 0; i < FILTER_SIZE; i++)
        filter_array_rpm[i] = 0;

    int filter_index_rpm = 0;

    while (1)
    {
        float rpm = get_engine_rpm();
        rpm = moving_average(rpm, filter_array_rpm, FILTER_SIZE, &filter_index_rpm);

        // int targetRPM = map(analogRead(36), 0, 4095, 500, 1200);
        setpoint = calculate_setpoint(rpm, setpoint, SETPOINT_RPM);
        // setpoint = map(analogRead(36), 0, 4095, IDLE_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT);
        // Serial.printf(">manualSetpoint: %d\n", map(analogRead(36), 0, 4095, IDLE_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT));

        int pos = encoder.getCount();

        float error = setpoint - pos; // calculate the error

        float derivative = error - last_error; // Derivatice calculation
        last_error = error;

        derivative = moving_average(derivative, filter_array_derivative, 5, &filter_index_derivative);

        integral += error; // I controller calculation
        integral = clamp(integral, -POS_MAX_I_TERM, POS_MAX_I_TERM);

        result = error * POS_Kp + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation

        set_direction_speed((int)result); // set the motor speed based on the pid term

        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">pos_setpoint: %f\n", setpoint);
        Serial.printf(">PWM: %f\n", result > 255 ? 255 : result < -255 ? -255
                                                                       : result);
        // Serial.printf(">analog: %d\n", analogRead(POT_PIN));
        // Serial.printf(">derivative: %f\n", derivative * POS_Kd);
        // Serial.printf(">integral: %f\n", integral * POS_Ki);
        Serial.printf(">rpm: %f\n", rpm);
        // Serial.printf(">targetRPM: %d\n", targetRPM);

        // Serial.printf(">count: %d\n", get_pulse_counter());
        // Serial.printf(">deltaCount: %f\n", deltaCount);
        // Serial.printf(">deltaT: %f\n", deltaT);
        delay(1);
    }
}

float calculate_setpoint(float rpm, float sheave_setpoint, float targetRPM)
{
    // linera interpolation between the idle and max rpm
    if (rpm < IDLE_RPM)
    {
        return IDLE_SHEAVE_SETPOINT;
    }
    float setpoint = float_map(rpm, IDLE_RPM, MAX_RPM, LOW_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT);
    setpoint = clamp(setpoint, LOW_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT);
    return setpoint;

}



// moving average filter
// newVal is the new value to add to the array
// arr is a pointer to the array
// n is the number of elements in the array
// index is a pointer to the index of the array
float moving_average(float newVal, float *arr, int n, int *index)
{
    arr[*index] = newVal;
    *index = (*index + 1) % n;

    float sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += arr[i];
    }

    return sum / n;
}