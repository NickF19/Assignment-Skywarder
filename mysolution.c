// Author : Nicolò Caruso

// Assumptions:
//      Calibration : we assume that the update function will be called for some steps before the liftoff
//      Lift-off :  we assume that the liftoff will happen at the moment of the first and huge delta acceleration.
//      Apogee :    we assume to be the point when the barometer reaches a minimum barometric pressure.
//                  we search for the point in which the mean of the barometric pressure returns to increase.
//                  the apogee is the first (local) minima, considering mean values, of the barometric pressure plot.
//      Landing :   we assume that the landing is when the barometer, the acceleration and gyros stabilize
//                  also, it can be assumed that a "spike" on acceleration due to touch-down is detectable.
//                  here we consider "landing" when the vector is stopped, not when it touches-down.
//      Sensors:    The sensor variance/error is considered, while it is not taken any cleaning procedure for outliers.
//                  Therefore, for now, outliers may trigger liftoff and apogee detection.

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "grader.h"

// Nr. of samples used for calibration phase (compute variance of the sensors)
#define N_CAL 20
// Multiplier of the variance to detect values that differ from the mean values considering the error.
#define N_OFF 5
// Nr. of barometric readings for apogee detection via considering mean value of the samples "sliding window"
#define N_BARO 6
// Multiplier for the landing variance/sensor error, for stabilized rocket detection.
#define LANDING_VAR 2

//  --- GLOBAL VARIABLES ---

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
// Variance of the samples - sensor variance/error
sample_t variance;
// Mean of the reads - initial state means for calibration, liftoff detection and then landing detection
sample_t mean_reads;

// Typedef for the phases of the launch
typedef enum {CALIBRATION, PRE_LAUNCH, LIFTOFF, APOGEE_REACHED, LANDED} phase_t;
phase_t phase;

// Indicates that the calibration process is in course
bool calibration_phase;

// Indicates that we are observing the first sample
bool first_read;

// Flag to indicate that the vector has stopped, no changes in the sensors.
// Used for landing detection
bool stable;

// NR. of sample used / counter
long int samples = 1;

// N baro samples for the APOGEE detection. 2 sliding window arrays for older and newer samples 
float old_baro_samples [N_BARO], new_baro_samples [N_BARO];
float new_baro_mean;
int baro_steps;

//  --- FUNCTIONS ---

// Helper function to update the barometric pressures arrays and means for apogee detection
void baro_means_update(float baro){
    int old_baro_sum=0;
    int new_baro_sum=0;
    for(int i = 0; i<N_BARO-1; i++){
        old_baro_samples[i] = old_baro_samples[i+1];
        old_baro_sum+=old_baro_samples[i];
    }
    old_baro_samples[N_BARO-1] = new_baro_samples[0];
    old_baro_sum += old_baro_samples[N_BARO-1];
    for(int i = 0; i<N_BARO-1; i++){
        new_baro_samples[i] = new_baro_samples[i+1];
        new_baro_sum+=new_baro_samples[i];
    }
    new_baro_samples[N_BARO-1] = baro;
    new_baro_sum+=baro;
    mean_reads.baro = old_baro_sum/N_BARO;
    new_baro_mean = new_baro_sum/N_BARO;
}

// Helper function to update the means of accelerations and gyros.
void acc_gyro_means_update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
                           float gyro_z)
{
    // We use an incremental mean method with "learning rate" to forget past mean.
    // This could also have been implemented as the barometer, with a sliding window of samples.
    mean_reads.acc_x = 0.25*acc_x + 0.75*mean_reads.acc_x;
    mean_reads.acc_y = 0.25*acc_y + 0.75*mean_reads.acc_y;
    mean_reads.acc_z = 0.25*acc_z + 0.75*mean_reads.acc_z;
    mean_reads.gyro_x = 0.25*gyro_x + 0.75*mean_reads.gyro_x;
    mean_reads.gyro_y = 0.25*gyro_y + 0.75*mean_reads.gyro_y;
    mean_reads.gyro_z = 0.25*gyro_z + 0.75*mean_reads.gyro_z;
}

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
    baro_steps = 0;
}

// Update function called at each new sample
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

                    // At the end we average the mean of sums of error by dividing for N_CAL/2
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
                        // Initialize barometric samples as mean vector
                        for (int i=0; i<N_BARO;i++)
                        {
                            old_baro_samples[i] = mean_reads.baro;
                            new_baro_samples[i] = mean_reads.baro;
                        }
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
                }
                baro_means_update(baro);
                break;

            case LIFTOFF:
                // To compute the apogee, we use two "samples window" the old one and the new one
                // By comparing the two windows, we can see if there is a clear increase of the barometric pressure
                // Computing the mean value for the last N_BARO samples
                baro_means_update(baro);
                acc_gyro_means_update(acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z);
                // If the newer mean is increasing, then we have reached the minimum barometric pressure (apogee)
                if(new_baro_mean > mean_reads.baro + N_OFF*variance.baro)
                {
                    printf("\n%ld - APOGEE REACHED\n", samples);
                    printf("\nAPOGEE MEAN BAROMETRIC PRESSURE: %.2f\n", mean_reads.baro);
                    apogee();
                    phase = APOGEE_REACHED;
                    break;
                }
                break;
            case APOGEE_REACHED:
                // Compute the stable flag, 1 if all the sensors are stable (samples within the mean +/- the noise)
                stable = 1;
                stable *= (fabs(acc_x-mean_reads.acc_x) < variance.acc_x*LANDING_VAR);
                stable *= (fabs(acc_y-mean_reads.acc_y) < variance.acc_y*LANDING_VAR);
                stable *= (fabs(acc_z-mean_reads.acc_z) < variance.acc_z*LANDING_VAR);
                stable *= fabs(gyro_x-mean_reads.gyro_x) < variance.gyro_x*LANDING_VAR;
                stable *= fabs(gyro_y-mean_reads.gyro_y) < variance.gyro_y*LANDING_VAR;
                stable *= fabs(gyro_z-mean_reads.gyro_z) < variance.gyro_z*LANDING_VAR;
                stable *= fabs(baro-new_baro_mean) < variance.baro*LANDING_VAR;

                if (stable)
                {
                    printf("\n%ld - STABLE CONFIGURATION, ROCKET LANDED\n", samples);
                    phase = LANDED;
                    landed();
                }
                // Updates the observed sensor sample means
                acc_gyro_means_update(acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z);
                baro_means_update(baro);

                break;

            case LANDED:
                break;

            default:
                return;
        }
    }
    samples++;
}