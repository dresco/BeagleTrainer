#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <libusb.h>
#include <math.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <gsl/gsl_multifit.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include "BeagleTrainer.h"

static volatile unsigned int is_sigint = 0;
volatile unsigned int runSpeedThread = 0;
volatile unsigned int runAntThread = 0;
volatile double currentSpeed;
volatile unsigned int currentMode = MODE_MANUAL;

volatile double power_curve_a = DEFAULT_CURVE_A;
volatile double power_curve_b = DEFAULT_CURVE_B;
volatile double power_curve_c = DEFAULT_CURVE_C;
volatile double power_curve_d = DEFAULT_CURVE_D;

// rx_data buffer needs to be bigger than a single ANT message to handle multiple messages
// even though we are not parsing them properly yet (max buffer size of endpoint is 64)
// todo: should we lookup the buffer sizes?
// todo: remove ant network key before public commit
const unsigned char key[8] = { 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45 };

unsigned char tx_data[ANT_MAX_MESSAGE_SIZE+1]; // include sync byte at front
unsigned char rx_data[ANT_MAX_MESSAGE_SIZE+1+50]; // include sync byte at front
int tx_len, rx_len;
int tx_bytes, rx_bytes;

static struct libusb_device_handle *devh = NULL;
uint16_t vendor_id = VID;
uint16_t product_id = PID;

typedef enum { IDLE,                // Not in spindown mode
               WAIT,                // Waiting for speed > 50
               READY,               // Speed > 50, waiting for deccel
               RUNNING,             // Decelerating, storing measurements
               OK,                  // Finished, curve fitting successful
               ERR                  // Error, perhaps perhaps speed increased?
             } SpinDownState;

typedef struct { double time;
                 double speed;
                 double speed_fitted;
                 double ke;
                 double ke_fitted;
                 double ke_rate_of_change;
               } SpinDownData;

typedef struct { double x;
                 double y;
               } FittingData;

struct TimerInfo
{
    int timer_fd;
    unsigned long long wakeups_missed;
};

double GetSpeed(unsigned int *pruData)
{
    uint32_t events = pruData[0];
    uint32_t frequency = events * PRU_UPDATE_FREQ;

    double rps = (double)frequency / PULSE_PER_REV;
    double mps = rps * 2.0 * M_PI * RADIUS / 1000.0;
    double kph = mps * 3.6;

    // Print out the value received from the PRU code
    // printf("%u events, %.0f RPM, %.2f km/h\r\n", events, rpm, kph);

    return(kph);
}

int curve_fit_quad(uint8_t count, FittingData f_data[], double *aa, double *bb, double *cc)
{
    int i, n;
    double xi, yi, chisq;
    gsl_matrix *X, *cov;
    gsl_vector *y, *c;

    n = count;

    X = gsl_matrix_alloc (n, 3);
    y = gsl_vector_alloc (n);

    c = gsl_vector_alloc (3);
    cov = gsl_matrix_alloc (3, 3);

    for (i = 0; i < n; i++)
    {
        xi = f_data[i].x;
        yi = f_data[i].y;

        //printf ("%g %g\n", xi, yi);

        gsl_matrix_set (X, i, 0, 1.0);
        gsl_matrix_set (X, i, 1, xi);
        gsl_matrix_set (X, i, 2, xi*xi);

        gsl_vector_set (y, i, yi);
    }

    gsl_multifit_linear_workspace * work
          = gsl_multifit_linear_alloc (n, 3);
    gsl_multifit_linear (X, y, c, cov,
                          &chisq, work);
    gsl_multifit_linear_free (work);

    printf ("# best fit: Y = %g + %g X + %g X^2\n",
            C(0), C(1), C(2));

    printf ("# covariance matrix:\n");
    printf ("[ %+.5e, %+.5e, %+.5e  \n",
               COV(0,0), COV(0,1), COV(0,2));
    printf ("  %+.5e, %+.5e, %+.5e  \n",
               COV(1,0), COV(1,1), COV(1,2));
    printf ("  %+.5e, %+.5e, %+.5e ]\n",
               COV(2,0), COV(2,1), COV(2,2));
    printf ("# chisq = %g\n", chisq);

    *aa = C(0);
    *bb = C(1);
    *cc = C(2);

    gsl_matrix_free (X);
    gsl_vector_free (y);
    gsl_vector_free (c);
    gsl_matrix_free (cov);

    return 0;
}

int curve_fit_cubic(uint8_t count, FittingData f_data[], double *aa, double *bb, double *cc, double *dd)
{
    int i, n;
    double xi, yi, chisq;
    gsl_matrix *X, *cov;
    gsl_vector *y, *c;

    n = count;

    X = gsl_matrix_alloc (n, 4);
    y = gsl_vector_alloc (n);

    c = gsl_vector_alloc (4);
    cov = gsl_matrix_alloc (4, 4);

    for (i = 0; i < n; i++)
    {
        xi = f_data[i].x;
        yi = f_data[i].y;

        //printf ("%g %g\n", xi, yi);

        gsl_matrix_set (X, i, 0, 1.0);
        gsl_matrix_set (X, i, 1, xi);
        gsl_matrix_set (X, i, 2, xi*xi);
        gsl_matrix_set (X, i, 3, xi*xi*xi);

        gsl_vector_set (y, i, yi);
    }

    gsl_multifit_linear_workspace * work
          = gsl_multifit_linear_alloc (n, 4);
    gsl_multifit_linear (X, y, c, cov,
                          &chisq, work);
    gsl_multifit_linear_free (work);

    printf ("# best fit: Y = %g + %g X + %g X^2 + %g X^3\n",
            C(0), C(1), C(2), C(3));

    printf ("# covariance matrix:\n");
    printf ("[ %+.5e, %+.5e, %+.5e, %+.5e  \n",
               COV(0,0), COV(0,1), COV(0,2), COV(0,3));
    printf ("  %+.5e, %+.5e, %+.5e, %+.5e  \n",
               COV(1,0), COV(1,1), COV(1,2), COV(1,3));
    printf ("  %+.5e, %+.5e, %+.5e, %+.5e ]\n",
               COV(2,0), COV(2,1), COV(2,2), COV(2,3));
    printf ("  %+.5e, %+.5e, %+.5e, %+.5e ]\n",
               COV(3,0), COV(3,1), COV(3,2), COV(3,3));
    printf ("# chisq = %g\n", chisq);

    *aa = C(0);
    *bb = C(1);
    *cc = C(2);
    *dd = C(3);

    gsl_matrix_free (X);
    gsl_vector_free (y);
    gsl_vector_free (c);
    gsl_matrix_free (cov);

    return 0;
}

// construct a message with all data passed (except sync and checksum)
void ANTBuildMessage(const unsigned char len,
                     const unsigned char type,
                     const unsigned char b3,
                     const unsigned char b4,
                     const unsigned char b5,
                     const unsigned char b6,
                     const unsigned char b7,
                     const unsigned char b8,
                     const unsigned char b9,
                     const unsigned char b10,
                     const unsigned char b11)
{
    // timestamp = get_timestamp();

    // encode the message
    tx_data[0] = ANT_SYNC_BYTE;
    tx_data[1] = len; // message payload length
    tx_data[2] = type; // message type
    tx_data[3] = b3;
    tx_data[4] = b4;
    tx_data[5] = b5;
    tx_data[6] = b6;
    tx_data[7] = b7;
    tx_data[8] = b8;
    tx_data[9] = b9;
    tx_data[10] = b10;
    tx_data[11] = b11;

    // compute the checksum and place after data
    unsigned char crc = 0;
    int i;
    for (i=0; i< (len+3); i++) crc ^= tx_data[i];
    tx_data[i] = crc;

    tx_len = i+1;
}

void ANTTransferMessage()
{
    int rc;

    // libusb 1.0 uses the same function for transmit & receive, direction depends on the endpoint
    rc = libusb_bulk_transfer(devh, ENDPOINT_OUT, tx_data, tx_len, &tx_bytes, WRITE_TIMEOUT);
    if (rc < 0)
    {
        printf("libusb: error %i transferring data OUT\n", rc);
    }
    else
    {
        //printf("libusb: transferred %i bytes OUT\n", tx_bytes);
        //printf("OUT: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        //             tx_data[0], tx_data[1], tx_data[2], tx_data[3],
        //             tx_data[4], tx_data[5], tx_data[6], tx_data[7],
        //             tx_data[8], tx_data[9], tx_data[10], tx_data[11],
        //             tx_data[12]);
    }

    // read a response and (mostly) throw it away
    rx_len = sizeof(rx_data);
    rc = libusb_bulk_transfer(devh, ENDPOINT_IN, rx_data, rx_len, &rx_bytes, READ_TIMEOUT);
    if (rc < 0)
    {
        printf("libusb: error %i transferring data IN\n", rc);
    }
    else
    {
        //printf("libusb: transferred %i bytes IN\n", rx_bytes);
        //printf("IN:  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        //             rx_data[0], rx_data[1], rx_data[2], rx_data[3],
        //             rx_data[4], rx_data[5], rx_data[6], rx_data[7],
        //             rx_data[8], rx_data[9], rx_data[10], rx_data[11],
        //             rx_data[12]);
    }
}

void milliSleep(unsigned int interval)
{
    // note: max interval of 1,000 ms (1 second or 1,000 million ns)
    int rc;

    #define NANO_SECOND_MULTIPLIER  1000000  // 1 millisecond = 1,000,000 Nanoseconds
    const long INTERVAL_NS = interval * NANO_SECOND_MULTIPLIER;

    struct timespec sleepValue;
    struct timespec returnTime;

    sleepValue.tv_sec = 0;
    sleepValue.tv_nsec = INTERVAL_NS;
    rc = nanosleep(&sleepValue, &returnTime);
    if (rc == -1)
    {
        perror ("nanosleep");
        return;
    }

}

// signal handler
static void on_sigint(int x)
{
    is_sigint = 1;
}

static int TimerStart(unsigned int period, struct TimerInfo *info)
{
    int rc;
    unsigned int ns;
    unsigned int sec;
    int fd;
    struct itimerspec itval;

    // Create the timer
    fd = timerfd_create (CLOCK_MONOTONIC, 0);
    info->wakeups_missed = 0;
    info->timer_fd = fd;
    if (fd == -1)
    {
        return fd;
    }

    // Make the timer periodic
    sec = period/1000000;
    ns = (period - (sec * 1000000)) * 1000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;

    rc = timerfd_settime (fd, 0, &itval, NULL);

    return rc;
}

static void TimerWait(struct TimerInfo *info)
{
    unsigned long long missed;
    int rc;

    // Wait for next timer event
    rc = read (info->timer_fd, &missed, sizeof (missed));
    if (rc == -1)
    {
        perror ("read timer");
        return;
    }

    // "missed" should always be >= 1, but just to be sure, check it is not 0 anyway
    if (missed > 0)
    {
        info->wakeups_missed += (missed - 1);
    }
}

static void *Speed_Thread(void *arg)
{
    struct TimerInfo info;

    // Initialise the PRU
    printf("Initialising PRU\n");
    prussdrv_init ();

    // Open an event
    if (prussdrv_open(PRU_EVTOUT_0))
    {
        printf("prussdrv_open open failed\n");
        return NULL;
    }

    // Get pointers to PRU0 local memory
    void *pruDataMem;
    prussdrv_map_prumem (PRUSS0_PRU0_DATARAM, &pruDataMem);
    unsigned int *pruData = (unsigned int *) pruDataMem;

    // Execute code on PRU
    // todo: check return code
    printf("Executing speed pru code\n");
    prussdrv_exec_program (0, "../pru/BeagleTrainer.bin");

    // Start the periodic timer @ 100ms
    TimerStart (100000, &info);

    // Run until terminated from main thread
    while (runSpeedThread)
    {
        // Wait for periodic timer to expire
        TimerWait (&info);

        // Broadcast data
        //printf("Speed Thread: timer expired...\n");
        uint32_t events = pruData[0];
        uint32_t frequency = events * PRU_UPDATE_FREQ;

        double rps = (double)frequency / PULSE_PER_REV;
        double rpm = rps * 60.0;
        double mps = rps * 2.0 * M_PI * RADIUS / 1000.0;
        double kph = mps * 3.6;

        currentSpeed = kph;

        // Print out the value received from the PRU code
        //printf("%u events, %.0f RPM, %.2f km/h\r\n", events, rpm, kph);
}

    // cleanup
    printf("Speed Thread: cleanup\n");
    prussdrv_pru_disable (0);
    prussdrv_exit ();

    return NULL;
}

static void *ANT_Thread(void *arg)
{
    struct TimerInfo info;

    uint8_t   power_event_count = 0;
    uint16_t  power_instant = 0;
    uint16_t  power_accumulated = 0;

    // USB setup
    if (libusb_init(NULL) != 0)
    {
        printf("libusb: failed to initialise\n");
        return NULL;
    }

    devh = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);

    if (devh == NULL)
    {
        printf("libusb: failed to open device 0x%04x:0x%04x\n", vendor_id, product_id);
    }
    else
    {
        // don't really care about the result..
        libusb_detach_kernel_driver(devh, 0);

        if (libusb_claim_interface(devh, 0) != 0)
        {
            printf("libusb: failed to claim interface");
        }
        else
        {
            printf("libusb: succesfully claimed interface\n");


            // ANT+ setup

            // reset
            ANTBuildMessage(1, ANT_SYSTEM_RESET, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // ANT docs say wait 500ms after reset before sending any more host commands
            milliSleep(500);

            // set network key
            ANTBuildMessage(9, ANT_SET_NETWORK, ANT_NETWORK_0, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]);
            ANTTransferMessage();

            // assign channel
            ANTBuildMessage(3, ANT_ASSIGN_CHANNEL, ANT_CHANNEL_1, ANT_CHANNEL_TYPE_TX, ANT_NETWORK_0, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // set channel id
            uint16_t device_id = 0xBEEF;
            ANTBuildMessage(5, ANT_CHANNEL_ID, ANT_CHANNEL_1, device_id&0xff, (device_id>>8)&0xff, ANT_SPORT_POWER_TYPE, ANT_TRANSMISSION_TYPE, 0, 0, 0, 0);
            ANTTransferMessage();

            // set rf frequency
            ANTBuildMessage(2, ANT_CHANNEL_FREQUENCY, ANT_CHANNEL_1, ANT_SPORT_FREQUENCY, 0, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // set channel period
            uint16_t period = ANT_SPORT_POWER_PERIOD;
            ANTBuildMessage(3, ANT_CHANNEL_PERIOD, ANT_CHANNEL_1, period&0xff, (period>>8)&0xff, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // set tx power? (optional)

            // open channel
            ANTBuildMessage(1, ANT_OPEN_CHANNEL, ANT_CHANNEL_1, 0, 0, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // Start the periodic timer @ 250ms
            // todo: use stricter ANT power message interval?
            TimerStart (250000, &info);

            // Run until terminated from main thread
            while (runAntThread)
            {
                // Wait for periodic timer to expire
                TimerWait (&info);

                // Broadcast data
                //printf("ANT_Thread: timer expired...\n");

                // Data Page 0x10 message format
                // Byte 0 : Data Page Number - 0x10
                // Byte 1 : Update event count
                // Byte 2 : Pedal power - 0xFF
                // Byte 3 : Instantaneous cadence - 0xFF
                // Byte 4 : Accumulated power (LSB)
                // Byte 5 : Accumulated power (MSB)
                // Byte 6 : Instantaneous power (LSB)
                // Byte 7 : Instantaneous power (MSB)

                // #define POWER   150
                // power_instant = POWER;

                power_event_count++;

                double speed = currentSpeed;

                power_instant = (power_curve_a +
                                (power_curve_b*speed) +
                                (power_curve_c*speed*speed) +
                                (power_curve_d*speed*speed*speed));

                power_accumulated += power_instant;

                ANTBuildMessage(9, ANT_BROADCAST_DATA, ANT_CHANNEL_1, ANT_STANDARD_POWER, power_event_count, 0xFF, 0xFF,
                        power_accumulated&0xff, (power_accumulated>>8)&0xff, power_instant&0xff, (power_instant>>8)&0xff);

                ANTTransferMessage();
            }

            // ANT+ cleanup
            printf("ANT_Thread: cleanup\n");

            // close channel
            ANTBuildMessage(1, ANT_CLOSE_CHANNEL, ANT_CHANNEL_1, 0, 0, 0, 0, 0, 0, 0, 0);
            ANTTransferMessage();

            // USB cleanup
            libusb_release_interface(devh, 0);
        }

        // USB cleanup
        libusb_close(devh);
    }
    libusb_exit(NULL);
    return NULL;
}

void SetupHardware(void)
{
    return;
}

void CleanupHardware(void)
{
    return;
}

void FitCurve(SpinDownData s_data[], uint8_t s_index)
{
    uint8_t  count;
    double   a, b, c, d;

    FittingData  f_data[MAX_SPINDOWN_SAMPLES];

    count = s_index;

    //for (int i = 0; i < s_index; i++)
    //{
    //    printf("%u, %.2f\n", s_data[i].time, s_data[i].speed);
    //}
    //printf("\n");

    // zero out the array of fitting data and re-populate
    memset(f_data, 0, sizeof (f_data));

    for (int i = 0; i < count; i++)
    {
        f_data[i].x = s_data[i].time;
        f_data[i].y = s_data[i].speed;
    }

    // fit the sampled speed vs time data to a quadratic equation (to clean up the data)
    if (curve_fit_quad(count, f_data, &a, &b, &c))
    {
        printf("fitting failed..\n\n");
    }
    else
    {
        printf("first fitting succeeded..\n");
        printf("fn(x) = a + bx + cx^2\n");
        printf(" a = %lf, b = %lf, c = %lf\n\n", a, b, c);
    }

    // store & print out the results
    for (int i = 0; i < s_index; i++)
    {
        s_data[i].speed_fitted = a + (b * s_data[i].time) + (c * s_data[i].time * s_data[i].time);
        //printf("time = %.2f, original speed = %.2f, fitted speed == %.2f\n", s_data[i].time, s_data[i].speed, s_data[i].speed_fitted);
    }

    // calculate the stored kinetic energy at each speed point
    // = 0.5 * 0.079619 * POWER(C2*0.27777777777778*1000/2098.58*2*PI(),2) + 0.5 * 0.005417523193127 * POWER(C2*0.27777777777778*1000/156.765473414*2*PI(),2)

    for (int i = 0; i < s_index; i++)
    {
        double speed_ms = s_data[i].speed_fitted * 0.27777777777778;

        // todo: results varies depending on whether we scale the quotient or divisor.
        //        is this a rounding error? figure this out in due course...

        double wheel_angular_velocity = (speed_ms * 1000 / WHEEL_CIRCUMFERENCE) * 2 * M_PI;
        double roller_angular_velocity = (speed_ms * 1000 / ROLLER_CIRCUMFERENCE) * 2 * M_PI;

        double wheel_kinetic_energy = 0.5 * WHEEL_INERTIA * (wheel_angular_velocity * wheel_angular_velocity);
        double roller_kinetic_energy = 0.5 * ROLLER_INERTIA * (roller_angular_velocity * roller_angular_velocity);
        double total_kinetic_energy = wheel_kinetic_energy + roller_kinetic_energy;

        s_data[i].ke = total_kinetic_energy;
        //printf("kinetic energy @ speed %lf is %lf\n", s_data[i].speed_fitted, s_data[i].ke);
    }

    // zero out the array of fitting data and re-populate
    memset(f_data, 0, sizeof (f_data));

    for (int i = 0; i < count; i++)
    {
        f_data[i].x = s_data[i].time;
        f_data[i].y = s_data[i].ke;
    }

    // fit kinetic energy vs time to a cubic equation
    if (curve_fit_cubic(count, f_data, &a, &b, &c, &d))
    {
        printf("fitting failed..\n\n");
    }
    else
    {
        printf("second fitting succeeded..\n");
        printf("fn(x) = a + bx + cx^2 + dx^3\n");
        printf(" a = %lf, b = %lf, c = %lf, d = %lf\n\n", a, b, c, d);
    }

    // store & print out the results
    for (int i = 0; i < s_index; i++)
    {
        s_data[i].ke_fitted = a + (b * s_data[i].time) + (c * s_data[i].time * s_data[i].time) + (d * s_data[i].time * s_data[i].time * s_data[i].time);
        //printf("time = %.2f, original ke = %.2f, fitted ke == %.2f\n", s_data[i].time, s_data[i].ke, s_data[i].ke_fitted);
    }

    // differentiate to get rate of change of KE at a given time
    for (int i = 0; i < s_index; i++)
    {
        s_data[i].ke_rate_of_change = b + (2 * c * s_data[i].time) + (3 * d * s_data[i].time * s_data[i].time);
        //printf("time = %.2f, ke rate of change == %.2f\n", s_data[i].time, s_data[i].ke_rate_of_change);
    }

    // zero out the array of fitting data and re-populate
    memset(f_data, 0, sizeof (f_data));

    for (int i = 0; i < count; i++)
    {
        f_data[i].x = s_data[i].speed_fitted;
        f_data[i].y = fabs(s_data[i].ke_rate_of_change);
        //printf("fitted speed = %lf, fitted load = %lf\n", f_data[i].x, f_data[i].y);
    }

    // fit rate of change vs speed to a cubic equation
    if (curve_fit_cubic(count, f_data, &a, &b, &c, &d))
    {
        printf("fitting failed..\n\n");
    }
    else
    {
        printf("third fitting succeeded..\n");
        printf("fn(x) = a + bx + cx^2 + dx^3\n");
        printf(" a = %lf, b = %lf, c = %lf, d = %lf\n\n", a, b, c, d);
    }

    for (int i = 0; i < 70; i++)
    {
        //printf("speed = %i, fitted load = %lf\n", i, a + (b*i) + (c*i*i) + (d*i*i*i));
    }


    // use above parameters to model static power requirements
    power_curve_a = a;
    power_curve_a = b;
    power_curve_a = c;
    power_curve_a = d;


    return;

}

static void *Spindown_Thread(void *arg)
{
    struct TimerInfo info;

    SpinDownState s_state = IDLE;
    SpinDownData  s_data[MAX_SPINDOWN_SAMPLES];
    uint8_t       s_index = 0;
    uint8_t       doSpinDown = 1;

    // set resistance to 0

    // Start the periodic timer @ 1s
    TimerStart (1000000, &info);

    // start the spindown process..
    while(doSpinDown && currentMode == MODE_CALIBRATE)
    {
        // Wait for periodic timer to expire
        TimerWait (&info);

        double kph = currentSpeed;

        switch (s_state)
        {
            case IDLE:
                printf("%.2f : ", kph);
                printf("Starting spindown calibration process...\n");

                // zero out the array of spindown entries & set index to start
                memset(s_data, 0, sizeof (s_data));
                s_index = 0;

                s_state = WAIT;
                break;

            case WAIT:
                if (kph > 50)
                {
                    s_state = READY;
                }
                else
                {
                    printf("%.2f : ", kph);
                    printf("Please increase speed to over 50 km/h...\n");
                }
                break;

            case READY:
                if (kph > 50)
                {
                    printf("%.2f : ", kph);
                    printf("Please stop pedaling to initiate calibration...\n");
                }
                else
                {
                    s_state = RUNNING;
                }
                break;

            case RUNNING:
                printf("%.2f : ", kph);
                printf("Calibrating, please wait....\n");

                // get current speed & add to array
                s_data[s_index].time = s_index+1;
                s_data[s_index].speed = kph;

                // check current speed is less than previous, else error
                if (s_index > 0)
                {
                    if (s_data[s_index].speed > s_data[s_index-1].speed)
                    {
                        s_state = ERR;
                        break;
                    }
                }

                // have we reached enough samples, or low enough speed
                if ((s_data[s_index].speed < 10) || (s_index == MAX_SPINDOWN_SAMPLES))
                {
                    s_state = OK;
                    break;
                }

                // ready for next sample
                s_index++;
                break;

            case ERR:
                printf("%.2f : ", kph);
                printf("Error, did you start pedaling? ;)...\n\n");
                s_state = IDLE;
                break;

            case OK:
                printf("%.2f : ", kph);
                printf("Spindown calibration process successful...\n\n");

                int i;
                for (i = 0; i < s_index; i++)
                {
                    printf("%.0f, %.2f\n", s_data[i].time, s_data[i].speed);
                }
                printf("\n");

                // ...and finish
                doSpinDown = 0;
                break;

            default:
                break;
        }

        if (s_state == OK)
        {
            // Spindown completed, now fit data to curve
            FitCurve(s_data, s_index);
        }

        if (currentMode != MODE_CALIBRATE)
        {
            printf("Spindown aborted..\n");
        }
    }
    return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t speed_thread;
    pthread_t ant_thread;
    pthread_t spindown_thread;

    // setup signal handler(s)
    signal(SIGINT, on_sigint);

    // setup hardware
    SetupHardware();

    // Start the speed sensor thread
    runSpeedThread = 1;
    pthread_create (&speed_thread, NULL, Speed_Thread, NULL);

    // start the ANT+ thread
    // todo: start broadcasting ANT data when movement sensed, and timeout/stop when idle
    runAntThread = 1;
    pthread_create (&ant_thread, NULL, ANT_Thread, NULL);

    // Run until Ctrl-C
    while (is_sigint == 0)
    {
        // while testing, just use blocking input for keystrokes to drive the modes
        // todo: at least use termio to avoid waiting for CR, even needs one after Ctrl-C
        printf("currentMode = 0x%02x\n", currentMode);
        printf("main thread sleeping for input..\n");
        char ch = getchar();

        // 'm' for manual resistance control
        if (ch == 'm')
        {
            currentMode = MODE_MANUAL;
            ch = getchar(); // swallow the CR
        }

        // 'e' for ergo
        if (ch == 'e')
        {
            currentMode = MODE_ERGO;
            ch = getchar(); // swallow the CR
        }

        // 's' for slope
        if (ch == 's')
        {
            currentMode = MODE_SLOPE;
            ch = getchar(); // swallow the CR
        }

        // 'c' for spindown/calibrate - as 's' was already taken ;)
        if (ch == 'c')
        {
            currentMode = MODE_CALIBRATE;
            ch = getchar(); // swallow the CR
            // todo: make sure we don't try and start more than one spindown thread
            pthread_create (&spindown_thread, NULL, Spindown_Thread, NULL);
        }
    }

    // wait for threads to finish
    // todo: actually wait..
    printf("\nWaiting for threads to finsh..\n");
    runSpeedThread = runAntThread = 0;
    sleep(1);

    // cleanup hardware
    CleanupHardware();

    exit(0);
}