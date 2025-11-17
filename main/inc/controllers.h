#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sensors.h"
#include "math.h"
#include "main.h"

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void); 

/* ------------------------------------------- Force/Thrust Parameters ------------------------------------------- */
#define MOTOR_MOMENT_ARM_M      0.041275        // Moment arm from center of drone to center of motor along x and y axes (m)
#define MAX_THRUST_N            75*0.00980665   // Motors can provide up to 75g of thrust
#define MAX_DUTY_CYCLE_PCT      80.0            // Limit output of motors

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      (SENS_PERIOD_MS*.001)                   // PID timestep (s)
#define DEG_2_RAD               (M_PI/180.0)
#define RAD_2_DEG               (180.0/M_PI)

#define PITCH_KP                2.0           
#define PITCH_KI                0.5           
#define PITCH_KD                0.0
#define PITCH_LIMIT             20.0*RAD_2_DEG

#define PITCH_RATE_KP           450.0         // originally 255
#define PITCH_RATE_KI           750.0         // originally 500
#define PITCH_RATE_KD           0.0           // originally 2.5
#define PITCH_RATE_LIMIT        80*RAD_2_DEG

#define ROLL_KP                 2.0           
#define ROLL_KI                 0.5           
#define ROLL_KD                 0.0
#define ROLL_LIMIT              20.0*RAD_2_DEG

#define ROLL_RATE_KP            450.0      
#define ROLL_RATE_KI            750.0      
#define ROLL_RATE_KD            0.0        
#define ROLL_RATE_LIMIT         80*RAD_2_DEG

#define ALTITUDE_RATE_KP            30.0     
#define ALTITUDE_RATE_KI            50.0      
#define ALTITUDE_RATE_KD            0.0        
#define ALTITUDE_RATE_LIMIT         MAX_THRUST_N

#endif /* CONTROLLERS_H */ 