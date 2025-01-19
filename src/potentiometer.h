

// based on analog read from 0 to 4065 
#define IDLE_POS 100 // Sheave position where sheave disengages for idle
#define LOW_POS 400 // Sheave position for lowest gear possible
#define HIGH_POS 3600 // Sheave position for highest gear possible

#define POS_MAX 3200 // Maximum value of the potentiometer (most together)
#define POS_MIN 600 // Minimum value of the potentiometer (most apart)


void setup_potentiometer();
int read_pos();
