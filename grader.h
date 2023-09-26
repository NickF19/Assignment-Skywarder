#pragma once

// This function should be called by your code when liftoff is detected
void liftoff();

// This function should be called by your code when the apogee is detected
void apogee();

// This function should be called by your code when landing is detected
void landed();

// This function will be called by the grader during initialization
void init();

// This function will be called by the grader for every datapoint read
void update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
            float gyro_z, float baro);