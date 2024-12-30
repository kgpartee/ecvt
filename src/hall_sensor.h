#define HALL_OUTPUT_PIN 26 // reads each time the hall sensor passes a magnet
#define THRESHOLD 250

#define IDLE_RPM  1800 // Threshold RPM for idle to engage
#define SETPOINT_RPM 3000 // Engine power-maximizing RPM, and target for PID
#define NUM_HALL_MAGNETS 6 // Number of magnets per revolution that trigger hall sensor
#define RPM_TIMER_DELAY 500 // How often the timer triggers, in seconds, should be faster than pid loop i think

int hall_count;
bool hall_trigger;
float engine_rpm;
int v;

void setup_hall();
void read_hall(int voltage);
