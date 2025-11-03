#include <stdio.h>
#include <stdint.h>
#include "sensors.h"

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

typedef struct {
    int64_t timestamp;
    float roll;
    float pitch;
    float yaw;
} estimated_state_t;

void cf_init(estimated_state_t *s);
void cf_update(estimated_state_t *s, const imu_sample_t *imu, const tof_sample_t *tof);

#endif