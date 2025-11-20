#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "freertos/queue.h"
#include "sensors.h"
#include "math.h"

/* ------------------------------------------- Force/Thrust Parameters ------------------------------------------- */
#define MOTOR_MOMENT_ARM_M      0.041275        // Moment arm from center of drone to center of motor along x and y axes (m)
#define MAX_THRUST_N            (75*0.00980665) // Motors can provide up to 75g of thrust
#define MAX_DUTY_CYCLE_PCT      100.0            // Limit output of motors

/* ------------------------------------------- PID Tuning ------------------------------------------- */
#define DT                      (SENS_PERIOD_MS*.001)                   // PID timestep (s)

// Attitude cascaded PID parameters
#define VEL_KP                  10.0           
#define VEL_KI                  6.0           
#define VEL_KD                  0.0
#define VEL_LIMIT               2.0

#define ATTITUDE_KP             10.0          
#define ATTITUDE_KI             5.0           
#define ATTITUDE_KD             0.0
#define ATTITUDE_LIMIT          2*M_PI

#define ATTITUDE_RATE_KP        50.0       
#define ATTITUDE_RATE_KI        40.0        // 50.0      
#define ATTITUDE_RATE_KD        0.0         //05.0       
#define ATTITUDE_RATE_LIMIT     100.0

// Yaw PID parameters
#define YAW_RATE_KP        20.0       
#define YAW_RATE_KI        5.0     
#define YAW_RATE_KD        0.0         
#define YAW_RATE_LIMIT     100.0

// Height cascaded PID parameters
#define ALTITUDE_KP             1.0    
#define ALTITUDE_KI             0.5   
#define ALTITUDE_KD             0.0        
#define ALTITUDE_LIMIT          .15

#define ALTITUDE_RATE_KP        60.0    
#define ALTITUDE_RATE_KI        80.0   
#define ALTITUDE_RATE_KD        10.0        
#define ALTITUDE_RATE_LIMIT     100.0

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void controllers_init(QueueHandle_t *pxQueue_state_data); 

#endif /* CONTROLLERS_H */ 