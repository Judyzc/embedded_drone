#include <stdio.h>
#include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "esp_system.h"
#include "complementary_filter.h"
#include "helper.h"
#include <math.h>

SixAxis comp_filter;

void estimator_init() {
    // Initialize complementary filter 
    CompInit(&comp_filter, DELTA_T, TAU);
    CompAccelUpdate(&comp_filter, 0.0, 0.0, 9.81);          // Assume drone is level when starting
    CompStart(&comp_filter);
}

void estimator_update(estimated_state_t *estimate_s, acc_sample_t *acc_s, gyro_sample_t *gyro_s, baro_sample_t *baro_s) {
    CompAccelUpdate(&comp_filter, acc_s->ax, acc_s->ay, acc_s->az); 
    CompGyroUpdate(&comp_filter, gyro_s->gx, gyro_s->gy, gyro_s->gz); 
    CompUpdate(&comp_filter);

    float pitch, roll; // radians
    CompAnglesGet(&comp_filter, &roll, &pitch); 
    if (pitch > M_PI)
        pitch -= 2.0*M_PI;  
    if (roll > M_PI)
        roll -= 2.0*M_PI; 
    roll *= -1.0;

    estimate_s->altitude = 0;
    
    estimate_s->pitch = pitch;
    estimate_s->pitch_rate = gyro_s->gx;
    
    estimate_s->roll = roll;
    estimate_s->roll_rate = gyro_s->gy; 
    estimate_s->yaw_rate = 0,                   

    estimate_s->timestamp = get_time();

    return;
}