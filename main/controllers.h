#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sensors.h"

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void); 

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      SENS_PERIOD_MS*.001             // PID timestep (s)

#define PITCH_KP                1.0
#define PITCH_KI                1.0
#define PITCH_KD                1.0
#define PITCH_LIMIT             1.0

#define PITCH_RATE_KP           1.0
#define PITCH_RATE_KI           1.0
#define PITCH_RATE_KD           1.0
#define PITCH_RATE_LIMIT        1.0

#define ROLL_KP                 1.0
#define ROLL_KI                 1.0
#define ROLL_KD                 1.0
#define ROLL_LIMIT              1.0

#define ROLL_RATE_KP            1.0
#define ROLL_RATE_KI            1.0
#define ROLL_RATE_KD            1.0
#define ROLL_RATE_LIMIT         1.0


#endif /* CONTROLLERS_H */