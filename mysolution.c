// Author : Nicolò Caruso

// Assumptions:
//      Calibration : we assume that the update function will be called for some steps before the liftoff
//      Lift-off :  we assume that the liftoff will happen at the moment of the first and huge delta acceleration.
//      Apogee :    we assume to be the point when the barometer reaches the maximum value.
//      Landing :   we assume that the landing is when the barometer, the acceleration and gyros stabilize
//                  also, it can be assumed that a "spike" on acceleration due to touch-down is detectable.
//      Sensors:    The sensor variance/error is considered, while it is not taken any cleaning procedure for outliers.
//                  Therefore, for now, outliers may trigger liftoff and apogee

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "grader.h"

// Nr. of samples used for calibration phase (compute variance of the sensors)
#define N_CAL 20
// Nr. of samples used
#define N_AVG 5
// Parameter to decide how if a sample is off the precedent mean.
#define N_OFF 5

// --- GLOBAL VARIABLES ---

// Samples data structure
typedef struct{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float baro;
}sample_t;
// Variance of the samples - sensor variance
sample_t variance;
// Mean of the reads - initial state means for calibration and liftoff detection
sample_t mean_reads;

// Typedef for the phases of the launch
typedef enum {CALIBRATION, PRE_LAUNCH, LIFTOFF, ENGINE_OFF, APOGEE_REACHED, PARACHUTE_DEPLOYED, LANDED} phase_t;
phase_t phase;

// Indicates that the calibration is in course
bool calibration_phase;

// Indicates that we are observing the first sample
bool first_read;

// NR. of sample used / counter
long int samples = 1;

// INIT - initializes flags and initial values.
void init() {
    first_read = 1;
    calibration_phase = 1;
    phase = CALIBRATION;
    mean_reads.acc_x = 0;
    mean_reads.acc_y = 0;
    mean_reads.acc_z = 0;
    mean_reads.gyro_x = 0;
    mean_reads.gyro_y = 0;
    mean_reads.gyro_z = 0;
    mean_reads.baro = 0;
    variance.acc_x = 0;
    variance.acc_y = 0;
    variance.acc_z = 0;
    variance.gyro_x = 0;
    variance.gyro_y = 0;
    variance.gyro_z = 0;
    variance.baro = 0;
}

void update (float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
            float gyro_z, float baro)
{
    // In case of first read initializes means and phase to CALIBRATION
    if(first_read)
    {
        printf("CALIBRATION...\n");
        mean_reads.acc_x = acc_x;
        mean_reads.acc_y = acc_y;
        mean_reads.acc_z = acc_z;
        mean_reads.gyro_x = gyro_x;
        mean_reads.gyro_y = gyro_y;
        mean_reads.gyro_z = gyro_z;
        mean_reads.baro = baro;
        first_read = !first_read;
        phase = CALIBRATION;
    }
    // In case of non-first viewed sample
    else{
        switch(phase) {
            case CALIBRATION:
                // For the first N_CAL/2 samples computes the expected mean, naively
                if(samples<=N_CAL/2) {
                    int denum = (1 / samples);
                    mean_reads.acc_x += denum * (acc_x - mean_reads.acc_x);
                    mean_reads.acc_y += denum * (acc_y - mean_reads.acc_y);
                    mean_reads.acc_z += denum * (acc_z - mean_reads.acc_z);
                    mean_reads.gyro_x += denum * (gyro_x - mean_reads.gyro_x);
                    mean_reads.gyro_y += denum * (gyro_x - mean_reads.gyro_y);
                    mean_reads.gyro_z += denum * (gyro_x - mean_reads.gyro_z);
                    mean_reads.baro += denum * (baro - mean_reads.baro);
                }
                // For the successive N_CAL/2 samples, sums the error
                else if(samples<=N_CAL) {
                    variance.acc_x += fabs(acc_x - mean_reads.acc_x);
                    variance.acc_y += fabs(acc_y - mean_reads.acc_y);
                    variance.acc_z += fabs(acc_z - mean_reads.acc_z);
                    variance.gyro_x += fabs(gyro_x - mean_reads.gyro_x);
                    variance.gyro_y += fabs(gyro_y - mean_reads.gyro_y);
                    variance.gyro_z += fabs(gyro_z - mean_reads.gyro_z);
                    variance.baro += fabs(baro - mean_reads.baro);

                    // At the end we average the RSS, Residual Sum of Squares by dividing for N_CAL/2
                    if (samples == N_CAL) {
                        variance.acc_x = variance.acc_x / (N_CAL / 2);
                        variance.acc_y = variance.acc_y / (N_CAL / 2);
                        variance.acc_z = variance.acc_z / (N_CAL / 2);
                        variance.gyro_x = variance.gyro_x / (N_CAL / 2);
                        variance.gyro_y = variance.gyro_y / (N_CAL / 2);
                        variance.gyro_z = variance.gyro_z / (N_CAL / 2);
                        variance.baro = variance.baro / (N_CAL / 2);
                        phase = PRE_LAUNCH;
                        printf("ENDING CALIBRATION\n");
                        printf("Sensor variance found: acc_x %2f, acc_y %2f, acc_z %2f, gyro_x %2f, gyro_y %2f, "
                               "gyro_z %2f, baro %2f \n", variance.acc_x, variance.acc_y, variance.acc_z,
                               variance.gyro_x, variance.gyro_y, variance.gyro_z, variance.baro);
                    }
                }
                break;

            case PRE_LAUNCH:
                // Liftoff when the x acceleration is increased over the sensor noise by a factor of N_OFF
                if(acc_x - mean_reads.acc_x > N_OFF * variance.acc_x)
                {
                    printf("\n%ld - LIFTOFF\n", samples);
                    liftoff();
                    phase = LIFTOFF;
                    return;
                }
                break;

                //TODO: complete the implementation from this point on
            case LIFTOFF:

                phase = ENGINE_OFF;
                break;
            case ENGINE_OFF:
                break;
            case APOGEE_REACHED:
                break;
            case PARACHUTE_DEPLOYED:
                break;
            case LANDED:
                break;

            default:
                return;
        }
    }
    samples++;
}