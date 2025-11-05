#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"

/* ------------------------------------------- Structs ------------------------------------------- */
typedef struct state_data {
    float pitch_rad; 
    float roll_rad; 
    float yaw_rad; 
    float altitude_m; 
} state_data_t;

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void estimator_init(void);

/* ------------------------------------------- Constants  ------------------------------------------- */
// Complementary filter parameters
#define TAU                     1.0f
#define DELTA_T                 ((float) SENS_PERIOD_MS)*.001f 

// General Constants
#define ESTIMATOR_PRIORITY      4

#endif /* ESTIMATOR_H */