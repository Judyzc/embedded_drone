#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sensors.h"
#include "math.h"

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void); 

/* ------------------------------------------- Force/Thrust Parameters ------------------------------------------- */
#define MOTOR_MOMENT_ARM_M      0.041275        // Moment arm from center of drone to center of motor along x and y axes (m)
#define MAX_THRUST_N            75*0.00980665
#define MAX_DUTY_CYCLE_PCT      80.0

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      SENS_PERIOD_MS*.001             // PID timestep (s)
#define DEG_2_RAD               M_PI/180.0
#define RAD_2_DEG               180/M_PI

#define PITCH_KP                1.0
#define PITCH_KI                0.0
#define PITCH_KD                0.0
#define PITCH_LIMIT             20.0*RAD_2_DEG

// inner loop pitch rate
#define PITCH_RATE_KP           1600.0         // originally 255
#define PITCH_RATE_KI           2400.0         // originally 500
#define PITCH_RATE_KD           0.0           // originally 2.5
#define PITCH_RATE_LIMIT        75*RAD_2_DEG

#define ROLL_KP                 1.0
#define ROLL_KI                 0.0
#define ROLL_KD                 0.0
#define ROLL_LIMIT              20.0*RAD_2_DEG

// inner loop roll rate
#define ROLL_RATE_KP            1600.0
#define ROLL_RATE_KI            2400.0
#define ROLL_RATE_KD            0.0
#define ROLL_RATE_LIMIT         75*RAD_2_DEG


#endif /* CONTROLLERS_H */