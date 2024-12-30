#define POT_PIN 32 // measures at potentiometer slider

// based on analog read from 0 to 4065 
#define IDLE_POS 100 // Sheave position where sheave disengages for idle
#define LOW_POS 400 // Sheave position for lowest gear possible
#define HIGH_POS 3600 // Sheave position for highest gear possible


int pot_v; // measurement across sliding part of potentiometer
float position; // Sheave position, reads from 0 to 4095, might scale it to between 0-1000.

void setup_potentiometer();
void read_pos();
