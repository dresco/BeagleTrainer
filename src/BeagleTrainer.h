// Device operation modes
#define MODE_MANUAL      0x00
#define MODE_ERGO        0x01
#define MODE_SLOPE       0x02
#define MODE_CALIBRATE   0x04

// Default values for power curve
#define DEFAULT_CURVE_A 2.1745916937068
#define DEFAULT_CURVE_B 0.519901309187647
#define DEFAULT_CURVE_C 0.0437150594453467
#define DEFAULT_CURVE_D -0.000261637287368499

// 4 magnets in current configuration
#define PULSE_PER_REV 4.0

// Radius of roller in mm
#define RADIUS 24.95

// How many times per second does the PRU write an event count
// (need to multiply the count value by this to get events/sec)
#define PRU_UPDATE_FREQ 1

#define MAX_SPINDOWN_SAMPLES 50

#define ROLLER_INERTIA 0.005417523193127
#define ROLLER_CIRCUMFERENCE 156.765473414
#define WHEEL_INERTIA   0.079619
#define WHEEL_CIRCUMFERENCE 2098.58

#define C(i) (gsl_vector_get(c,(i)))
#define COV(i,j) (gsl_matrix_get(cov,(i),(j)))

#define VID   0x0fcf
#define PID   0x1008

#define WRITE_TIMEOUT 125
#define READ_TIMEOUT  125
#define ENDPOINT_OUT  0x01             // todo: should we enumerate the endpoints?
#define ENDPOINT_IN   0x81

#define ANT_SYNC_BYTE          0xA4
#define ANT_MAX_MESSAGE_SIZE   12
#define ANT_SYSTEM_RESET       0x4A
#define ANT_SET_NETWORK        0x46
#define ANT_ASSIGN_CHANNEL     0x42
#define ANT_CHANNEL_ID         0x51
#define ANT_CHANNEL_PERIOD     0x43
#define ANT_CHANNEL_FREQUENCY  0x45


#define ANT_OPEN_CHANNEL       0x4B
#define ANT_CLOSE_CHANNEL      0x4C
#define ANT_CHANNEL_1          1
#define ANT_NETWORK_0          0
#define ANT_CHANNEL_TYPE_TX    0x10
#define ANT_SPORT_HR_TYPE      0x78
#define ANT_TRANSMISSION_TYPE  1
#define ANT_SPORT_FREQUENCY    57
#define ANT_SPORT_HR_PERIOD    8070
#define ANT_BROADCAST_DATA     0x4E
#define ANT_STANDARD_POWER     0x10
#define ANT_SPORT_POWER_PERIOD 8182
#define ANT_SPORT_POWER_TYPE 11

