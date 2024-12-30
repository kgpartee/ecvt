#include "potentiometer.h"
#include <Arduino.h>

void setup_potentiometer(){
    pinMode(POT_PIN, INPUT);
}

void read_pos() {
    pot_v = analogRead(POT_PIN); // potentiometer lever
// total_v = analogRead(potPinTwo); // whole resistor

// read the voltage across the potentiometer as a proportion, because when the voltage changes as the battery drains, the proportion will remain constant 
   // encoderReading = pot_v / total_v; // find how far along the resistor the potentiometer is as a fraction
    position = pot_v; // scale to make more workable number based on zero/thousand setpoints from calibration or just reset the high/low 

}

