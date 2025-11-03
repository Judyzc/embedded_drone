#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"

// Public function declarations
void estimator_init(void);

// Complementary filter parameters
#define TAU                     1.0f
#define DELTA_T                 ((float) SENS_PERIOD_MS)*.001f 

// Constants
#define ESTIMATOR_PRIORITY      4

#endif /* ESTIMATOR_H */