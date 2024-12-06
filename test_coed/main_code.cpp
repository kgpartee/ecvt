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
float position; // Sheave position, scale it to between 0-1000.
int speed;
int hallCount = 0; // Hall sensor counts 
int rpmTimerDeltaT = 500; // How often the timer triggers, in seconds, should be faster than pid loop methinks 
int pidLoopDelay; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v; //measuremetn across potentiometer used as resistor
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)

uint8_t potPinOne = 32; // measures at potentiometer slider
uint8_t potPinTwo = 2; //measures across whole potentiometer resistor
uint8_t potPowerPin = 12; //powers potentiometer
uint8_t pwmPin = 33; // SV on driver, sends pulses to the motor 
uint8_t directionPin = 25; // F/R on driver, controls the motor's direction
uint8_t brakePin = 5; //starts and stops the motor
uint8_t hallOutputPin = 1; // reads each time the hall sensor passes a magnet
uint8_t calibrationButton = 7; // starts calibration sequence
uint8_t limitSwitchOne = 8; //measures one of the maximum sheave travel positions
uint8_t limitSwitchOnePower = 13; //powers limit switch 1
uint8_t limitSwitchTwo = 9; //measures the other maximum sheave travel position 
uint8_t limitSwitchTwoPower = 14; //powers limit switch 2
uint8_t manualButtonOne = 10; //starts manual mode 
uint8_t manualButtonTwo = 11; //starts manual mode 
uint8_t buttonPower = 15; // powers buttons for calibration and manual modes
uint8_t enablePin; // runs when high or floating 


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


// if ypu dont get the naming conventions of my timer variables i don't either it's just the same conventions ilya uses 
// and in ilya we trust prayer hands emoji x3

// i dont really get the timer either so anyone reading this should expect Many Issues

hw_timer_t * rpmTimer = NULL; 
portMUX_TYPE rpmTimerMUX = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR hallReset() {
  portENTER_CRITICAL_ISR(&rpmTimerMUX);
    while(numSamples < 10){
    engineRPM += hallCount*60/(rpmTimerDeltaT*numHallMagnets);
    numSamples ++;
    hallCount = 0;
    }
    numSamples = 0; 
    engineRPMavg = engineRPM / 10; 
    engineRPM = 0; 
  portEXIT_CRITICAL_ISR(&rpmTimerMUX);
}

hw_timer_t * pidTimerPos = NULL; 
portMUX_TYPE pidTimerPosMUX = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * pidTimerRPM = NULL; 
portMUX_TYPE pidTimerRPMMUX = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR pidLoopRPM() {
  portENTER_CRITICAL_ISR(&pidTimerRPMMUX);
    
    //setpoint = 3000
    //current = engineRPM

    rpmPidCalc();
  portEXIT_CRITICAL_ISR(&pidTimerRPMMUX);
}

 
void IRAM_ATTR pidLoopPos() {
  portENTER_CRITICAL_ISR(&pidTimerPosMUX);
    //setpoint = pidTermRPM
    //current = position
    readPos(); //find the current sheave position scaled 1-1000

    //constrain travel before limit switches 
    if(position > highPos) {
      setpointPos = highPos; 
      setDirection();
      posPidCalc();
      ledcWrite(pwmPin, pidTermPos);
    }
    else if(position<lowPos){
      setpointPos = lowPos; 
      setDirection();
      posPidCalc();
      ledcWrite(pwmPin, pidTermPos);
    }
    else if(engineRPM < idleRPM){
      setpointPos = idlePos; 
      setDirection();
      posPidCalc(); 
      ledcWrite(pwmPin, pidTermPos);
    }
    // unconstrained travel
    else{
      setpointPos = pidTermRPM; 
      setDirection();
      posPidCalc();
      ledcWrite(pwmPin, pidTermPos);
    }

  portEXIT_CRITICAL_ISR(&pidTimerPosMUX);
}

//add multisampling tiemr 


// put function declarations here:

// Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
void IRAM_ATTR hallInterrupt() {
    hallCount++;
}

// interrupt to run calibration sequence 
void IRAM_ATTR calibrationInterrupt(){

while(bool pressed = LOW) {
  pressed = digitalRead(limitSwitchOne);
  digitalWrite(directionPin, HIGH);
  analogWrite(pwmPin, 50);
}

 pot_v = analogRead(potPinOne); // potentiometer slider
 total_v = analogRead(potPinTwo); // whole resistor
 positionZero = pot_v/total_v; 
digitalWrite(brakePin, LOW); 
delay(50);
digitalWrite(brakePin, HIGH);

while(bool pressed = LOW) {
  pressed = digitalRead(limitSwitchTwo);
  digitalWrite(directionPin, LOW);
  analogWrite(pwmPin, 50);
}

 pot_v = analogRead(potPinOne); // potentiometer slider
 total_v = analogRead(potPinTwo); // whole resistor
 positionThousand = 1000 * total_v / pot_v; 
digitalWrite(brakePin, LOW);
delay(50);
digitalWrite(brakePin, HIGH); 
}


void IRAM_ATTR endCaseOne() {
  setpointPos = highPos; 
  posPidCalc(); 
  ledcWrite(pwmPin, pidTermPos);

}

void IRAM_ATTR endCaseTwo() {
  setpointPos = lowPos; 
  posPidCalc(); 
  ledcWrite(pwmPin, pidTermPos);

}

//hold down button to keep mode on 
void IRAM_ATTR manualModeOne() {
  bool pressed = digitalRead(manualButtonOne);
  while(pressed = HIGH) {
  pressed = digitalRead(manualButtonOne);
  setpointPos = manualPosOne; 
  posPidCalc();
  ledcWrite(pwmPin, pidTermPos);
  }
}

void IRAM_ATTR manualModeTwo() {
  bool pressed = digitalRead(manualButtonTwo);
  while(pressed = HIGH) {
  setpointPos = manualPosTwo;
  posPidCalc();
  ledcWrite(pwmPin, pidTermPos);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  pinMode(potPinOne, INPUT);
  pinMode(potPinTwo, INPUT);
  pinMode(potPowerPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(hallOutputPin, INPUT);
  pinMode(calibrationButton, INPUT);
  pinMode(limitSwitchOne, INPUT);
  pinMode(limitSwitchOnePower, OUTPUT);
  pinMode(limitSwitchTwo, INPUT);
  pinMode(limitSwitchTwoPower, OUTPUT);
  pinMode(manualButtonOne, INPUT); 
  pinMode(manualButtonTwo, INPUT); 
  pinMode(buttonPower, OUTPUT);

  //initialize variable values, will def need to add more
  digitalWrite(brakePin, HIGH); // set brake off
  hallCount = 0; 
  digitalWrite(potPowerPin, HIGH); // power potentiometer at voltage that wont break MCU (3.3V)
  digitalWrite(limitSwitchOnePower, HIGH);
  digitalWrite(limitSwitchTwoPower, HIGH);
  digitalWrite(buttonPower, HIGH); 

  // interrupt set up
  attachInterrupt(digitalPinToInterrupt(hallOutputPin), hallInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(calibrationButton), calibrationInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(limitSwitchOne), endCaseOne, RISING);
  attachInterrupt(digitalPinToInterrupt(limitSwitchTwo), endCaseTwo, RISING);

  //hardware timer setup
  rpmTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(rpmTimer, &hallReset, true);
  timerAlarmWrite(rpmTimer, rpmTimerDeltaT, true);
  timerAlarmEnable(rpmTimer);

  pidTimerRPM = timerBegin(2, 80, true);
  timerAttachInterrupt(pidTimerRPM, &pidLoopRPM, true);
  timerAlarmWrite(pidTimerRPM, pidLoopDelay, true);
  timerAlarmEnable(pidTimerRPM);

  pidTimerPos = timerBegin(3, 80, true);
  timerAttachInterrupt(pidTimerPos, &pidLoopPos, true);
  timerAlarmWrite(pidTimerPos, pidLoopDelay, true);
  timerAlarmEnable(pidTimerPos);

  ledcAttachPin(pwmPin, 0);
  ledcSetup(0,1500,4);

}

void loop() {
  // put your main code here, to run repeatedly:

   
}

// put function definitions here:




void rpmPidCalc() {
errorRPM = setpointRPM - engineRPMavg;

changeErrorRPM = errorRPM - lastErrorRPM;
totalErrorRPM += errorRPM;
pidTermRPM = (kpRPM * errorRPM) + (kiRPM * totalErrorRPM) + (kdRPM * changeErrorRPM);

lastErrorRPM = errorRPM;

}

void posPidCalc() {
  errorPos = setpointPos - position;

  changeErrorPos = errorPos - lastErrorPos; 
  totalErrorPos += errorPos;
  pidTermPos = (kpPos * errorPos) + (kiPos * totalErrorPos) + (kdPos * changeErrorPos);

  lastErrorPos = errorPos; 
}

void readPos() {
pot_v = analogRead(potPinOne); // potentiometer lever
total_v = analogRead(potPinTwo); // whole resistor

// read the voltage across the potentiometer as a proportion, because when the voltage changes as the battery drains, the proportion will remain constant 
encoderReading = pot_v / total_v; // find how far along the resistor the potentiometer is as a fraction
position = positionThousand * (encoderReading - positionZero); // scale to make more workable number based on zero/thousand setpoints from calibration

}

void setDirection() {
  
  if (position < setpointPos){
    digitalWrite(directionPin, LOW);
  }
  else {
    digitalWrite(directionPin, HIGH);
  }

}