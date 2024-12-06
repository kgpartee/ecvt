#include <Arduino.h>


// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


// E-cvt automatic mode skeleton code
// Nominal restricted Kohler idle is 1700rpm, max is 3800rpm, and peak power is at 3000rpm.
int idleRPM = 1800; // Threshold RPM for idle to engage
int setpointRPM = 3000; // Engine power-maximizing RPM, and target for PID
int numHallMagnets = 6; // Number of magnets per revolution that trigger hall sensor
int encoderResolution = 4095; // Default 12-bit esp adc resolution, for scaling purposes

// Positions range from 0 to 1000, and are scaled later to encoder input range.
int idlePos = 50; // Sheave position where sheave disengages for idle
int lowPos = 100; // Sheave position for lowest gear possible
int highPos = 900; // Sheave position for highest gear possible
int manualPosOne; 
int manualPosTwo; 


//ints for position pid gains;
//ints for engine rpm pid gains;
int engineRPM; // Calculated from hall sensor
int engineRPMavg; //rpm after multisampling
int numSamples = 0; // number of samples over which the average is taken
int encoderReading; // ADC input from linear potentiometer
int position; // Sheave position, scale it to between 0-1000.
int hallCount = 0; // Hall sensor counts 
int rpmTimerDeltaT = 500; // How often the timer triggers, in seconds, should be faster than pid loop methinks 
int pidLoopDelay; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v; //measuremetn across potentiometer used as resistor
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)

uint8_t potPinOne = 1; // measures at potentiometer slider
uint8_t potPinTwo = 2; //measures across whole potentiometer resistor
uint8_t potPowerPin = 12; //powers potentiometer
uint8_t pwmPin = 3; // sends pulses to the motor 
uint8_t directionPin = 4; // controls the motor's direction
uint8_t brakePin = 5; //starts and stops the motor
uint8_t hallOutputPin = 36; // reads each time the hall sensor passes a magnet
uint8_t calibrationButton = 7; // starts calibration sequence
uint8_t limitSwitchOne = 8; //measures one of the maximum sheave travel positions
uint8_t limitSwitchOnePower = 13; //powers limit switch 1
uint8_t limitSwitchTwo = 9; //measures the other maximum sheave travel position 
uint8_t limitSwitchTwoPower = 14; //powers limit switch 2
uint8_t manualButtonOne = 10; //starts manual mode 
uint8_t manualButtonTwo = 11; //starts manual mode 
uint8_t buttonPower = 15; // powers buttons for calibration and manual modes


int setpointPos;//determined by engine rpm
// pid loop coefficients
int kpPos = 1;
int kiPos = 1;
int kdPos = 1;

int kpRPM = 1;
int kiRPM = 1;
int kdRPM = 1; 

float lastErrorPos = 0;
float errorPos = 0;
float changeErrorPos = 0;
float totalErrorPos = 0;
float pidTermPos = 0;

float lastErrorRPM = 0;
float errorRPM = 0;
float changeErrorRPM = 0;
float totalErrorRPM = 0;
float pidTermRPM = 0;
float threshold = 2000;
bool hall_trigger;

// read hall sensor -> figure out threshold value
// count and reset after six counts
// calculate rpm appropriately w/o multisampling



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(hallOutputPin, INPUT);
  hall_trigger = false;
}

void loop() {
  
    int v = analogRead(hallOutputPin);
    if (v > threshold && !hall_trigger) {
      hall_trigger = true;
      hallCount++;

    }
    else if (v < threshold && hall_trigger){
      hall_trigger = false;
    }

    // if (v == HIGH) {
    //   hallCount++;
    // }
    Serial.print(">hall_output:");
    Serial.println(v);
    Serial.print(">hall_count:");
    Serial.println(hallCount);
}

