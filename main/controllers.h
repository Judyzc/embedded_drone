#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sensors.h"
#include "math.h"

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void); 

/* ------------------------------------------- Force/Thrust Parameters ------------------------------------------- */
#define MOTOR_MOMENT_ARM_M      0.041275        // Moment arm from center of drone to center of motor along x and y axes (m)
#define MAX_THRUST_N            75*0.00980665
#define MAX_DUTY_CYCLE_PCT      50.0

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      SENS_PERIOD_MS*.001             // PID timestep (s)
#define DEG_2_RAD               M_PI/180.0

#define PITCH_KP                6.0*DEG_2_RAD
#define PITCH_KI                3.0*DEG_2_RAD
#define PITCH_KD                0.0*DEG_2_RAD
#define PITCH_LIMIT             20.0*DEG_2_RAD

#define PITCH_RATE_KP           255.0*DEG_2_RAD
#define PITCH_RATE_KI           500.0*DEG_2_RAD
#define PITCH_RATE_KD           2.5*DEG_2_RAD
#define PITCH_RATE_LIMIT        33.3*DEG_2_RAD

#define ROLL_KP                 6.0*DEG_2_RAD
#define ROLL_KI                 3.0*DEG_2_RAD
#define ROLL_KD                 0.0*DEG_2_RAD
#define ROLL_LIMIT              20.0*DEG_2_RAD

#define ROLL_RATE_KP            255.0*DEG_2_RAD
#define ROLL_RATE_KI            500.0*DEG_2_RAD
#define ROLL_RATE_KD            2.5*DEG_2_RAD
#define ROLL_RATE_LIMIT         33.3*DEG_2_RAD


#endif /* CONTROLLERS_H */