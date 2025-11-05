#ifndef MOTORS_H
#define MOTORS_H

#include "driver/ledc.h"
#include "math.h"

/* ------------------------------------------- Structs ------------------------------------------- */
typedef struct motor_cmds {
    float motor1_duty_cycle_pct;                // pct
    float motor2_duty_cycle_pct;                // pct
    float motor3_duty_cycle_pct;                // pct
    float motor4_duty_cycle_pct;                // pct
} motor_cmds_t; 

/* ------------------------------------------- Constants ------------------------------------------- */
#define PWM_TIMER                   LEDC_TIMER_0
#define PWM_MODE                    LEDC_LOW_SPEED_MODE
#define MOTOR_1_GPIO                (14) // Define the output GPIO
#define MOTOR_2_GPIO                (12) // Define the output GPIO
#define MOTOR_3_GPIO                (13) // Define the output GPIO
#define MOTOR_4_GPIO                (15) // Define the output GPIO
#define MOTOR_1_CHANNEL             LEDC_CHANNEL_0
#define MOTOR_2_CHANNEL             LEDC_CHANNEL_1
#define MOTOR_3_CHANNEL             LEDC_CHANNEL_2
#define MOTOR_4_CHANNEL             LEDC_CHANNEL_3
#define PWM_DUTY_RES                LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define PWM_DUTY                    (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define PWM_FREQUENCY_HZ            (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define MAX_PWM_DUTY                ((int) pow(2.0, PWM_DUTY_RES))-1

#define DUTY_CYCLE_PCT_2_VAL(pct)   (int) (100.0*pct*((float) MAX_PWM_DUTY)) // Convert a duty cyle in percent to its corresponding integer value

/* ------------------------------------------- Public Function Definitions ------------------------------------------- */
void motors_init(void); 
void update_pwm(motor_cmds_t motor_cmds);

#endif /* MOTORS_H */