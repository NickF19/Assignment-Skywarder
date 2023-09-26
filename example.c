#include <stdio.h>

#include "grader.h"

int counter;

void init() { 
    printf("Init!\n"); 

    // Initialize your global variables
    counter = 0;
}

void update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
            float gyro_z, float baro)
{
    printf("Update: %f,%f,%f,%f,%f,%f,%f\n", acc_x, acc_y, acc_z, gyro_x,
           gyro_y, gyro_z, baro);

    // Update your algorithms
    counter++;

    // This is a terrible solution, please do not do this
    if(counter == 500) {
        liftoff();
    } else if(counter == 1000) {
        apogee();
    } else if(counter == 1500) {
        landed();
    }
}