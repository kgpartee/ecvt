

// based on analog read from 0 to 4065 
#define IDLE_POS 100 // Sheave position where sheave disengages for idle
#define LOW_POS 400 // Sheave position for lowest gear possible
#define HIGH_POS 3600 // Sheave position for highest gear possible



void setup_potentiometer();
void read_pos();
