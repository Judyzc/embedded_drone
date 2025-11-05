#include <stdio.h>
#include "i2c.h"

#ifndef SENSORS_H
#define SENSORS_H

// sensor samples
typedef struct {
    int64_t timestamp; // us
    float accel[3]; // m/s^2
    float gyro[3];  // units??
} imu_sample_t;

typedef struct {
    int64_t timestamp; // us
    float pressure; // Pa or arbitrary units for stub
    float temperature;
    float asl;
} baro_sample_t;

typedef struct {
    int64_t timestamp;
    float distance_mm; // mm
} tof_sample_t;

void mock_imu_read(imu_sample_t *imu_sample);


void baro_init(i2c_master_dev_handle_t* dev_handle);

void mock_baro_read(baro_sample_t *baro_sample);

void baro_read(baro_sample_t *baro_sample);


void mock_tof_read(tof_sample_t *tof_sample);


#endif