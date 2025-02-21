
#define THRESHOLD 250

#define IDLE_RPM  1800 // Threshold RPM for idle to engage
#define SETPOINT_RPM 3000 // Engine power-maximizing RPM, and target for PID
#define NUM_HALL_MAGNETS 6.0f // Number of magnets per revolution that trigger hall sensor
#define RPM_TIMER_DELAY 500000.0f // How often the timer triggers, in 1000000/n times per second, should be faster than pid loop i think
#define HALL_TIMER_DELAY 500



int get_hall_count();
void setup_hall();
void read_hall(int voltage);
void rpm_calc();
float get_rpm();