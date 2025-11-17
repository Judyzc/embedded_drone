#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sensors.h"
#include "math.h"

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(void); 

/* ------------------------------------------- Force/Thrust Parameters ------------------------------------------- */
#define MOTOR_MOMENT_ARM_M      0.041275        // Moment arm from center of drone to center of motor along x and y axes (m)
#define MAX_THRUST_N            (75*0.00980665) // Motors can provide up to 75g of thrust
#define MAX_DUTY_CYCLE_PCT      80.0            // Limit output of motors

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      (SENS_PERIOD_MS*.001)                   // PID timestep (s)
#define DEG_2_RAD               (M_PI/180.0)
#define RAD_2_DEG               (180.0/M_PI)

#define PITCH_KP                .8           
#define PITCH_KI                0.2           
#define PITCH_KD                0.0
#define PITCH_LIMIT             .350

#define PITCH_RATE_KP           .0317       
#define PITCH_RATE_KI           .0577       
#define PITCH_RATE_KD           0.0         
#define PITCH_RATE_LIMIT        2.0

#define ROLL_KP                 .8           
#define ROLL_KI                 0.2           
#define ROLL_KD                 0.0
#define ROLL_LIMIT              .350

#define ROLL_RATE_KP            .0317
#define ROLL_RATE_KI            .0577
#define ROLL_RATE_KD            0.0  
#define ROLL_RATE_LIMIT         2.0

#define ALTITUDE_KP            2.0    
#define ALTITUDE_KI            0.5   
#define ALTITUDE_KD            0.0        
#define ALTITUDE_LIMIT         2.0

#define ALTITUDE_RATE_KP            4.0    
#define ALTITUDE_RATE_KI            1.0   
#define ALTITUDE_RATE_KD            0.5        
#define ALTITUDE_RATE_LIMIT         5.0

#endif /* CONTROLLERS_H */ 