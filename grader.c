// !!!------------------ DO NOT MODIFY THIS CODE ------------------!!!

#include <assert.h>
#include <stdio.h>

#include "grader.h"

void init();
void update(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y,
            float gyro_z, float baro);

typedef enum
{
    STATE_INIT,
    STATE_FLIGHT,
    STATE_DESCENT,
    STATE_LANDED
} state_t;

state_t state = STATE_INIT;
int cur_ts;

int liftoff_ts = 0;
int apogee_ts  = 0;
int landed_ts  = 0;

void liftoff()
{
    assert(state == STATE_INIT);
    liftoff_ts = cur_ts;
    state      = STATE_FLIGHT;
}

void apogee()
{
    assert(state == STATE_FLIGHT);
    apogee_ts = cur_ts;
    state     = STATE_DESCENT;
}

void landed()
{
    assert(state == STATE_DESCENT);
    landed_ts = cur_ts;
    state     = STATE_LANDED;
}

int main()
{
    FILE *f = fopen("sim.csv", "r");

    fscanf(f, "%*s,%*s,%*s,%*s,%*s,%*s,%*s,%*s");

    int ts;
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, baro;

    init();
    while (fscanf(f, "%d,%f,%f,%f,%f,%f,%f,%f", &ts, &acc_x, &acc_y, &acc_z,
                  &gyro_x, &gyro_y, &gyro_z, &baro) == 8 &&
           state != STATE_LANDED)
    {
        cur_ts = ts;

        update(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, baro);
    }

    fclose(f);

    printf("Simulation ended!\n");
    printf("- Liftoff timestamp:   %.3fs\n", liftoff_ts / 1e6);
    printf("- Apogee timestamp:    %.3fs\n", apogee_ts / 1e6);
    printf("- Landeding timestamp: %.3fs\n", landed_ts / 1e6);

    return 0;
}

// !!!------------------ DO NOT MODIFY THIS CODE ------------------!!!