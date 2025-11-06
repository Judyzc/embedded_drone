#include <stdio.h>
#include <stdint.h>
#include "sensors.h"
#include "six_axis_comp_filter.h"

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#define TAU        1.0f
#define DELTA_T    ((float) SENS_PERIOD_MS)*.001f 

extern SixAxis comp_filter;

typedef struct {
    int64_t timestamp; // us
    float roll;   // deg
    float pitch;  // deg
    float yaw;    // deg
    float altitude_mm;
} attitude_t;

typedef struct {
    int64_t timestamp; // us
    float roll; // rad
    float pitch; // rad
    float roll_rate; // rad/s
    float pitch_rate; // rad/s
    float yaw_rate; // rad/s
    float altitude; // m
} estimated_state_t;

void estimator_init();
void estimator_update(
    estimated_state_t *estimate_s, 
    acc_sample_t *acc_s, 
    gyro_sample_t *gyro_s, 
    baro_sample_t *baro_s
);

#endif