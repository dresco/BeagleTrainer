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

// 16 high/low transitions on 32 segment encoder disk
#define PULSE_PER_REV 32.0

// Radius of roller in mm
#define RADIUS 24.95

// How many times per second does the PRU write an event count
// (need to multiply the count value by this to get events/sec)
#define PRU_UPDATE_FREQ 4

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

#define DISPLAY_MAX_MSG_LINES   10
#define DISPLAY_MAX_MSG_SIZE    80


