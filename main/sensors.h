#include <stdio.h>

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
} baro_sample_t;

typedef struct {
    int64_t timestamp;
    float distance_mm; // mm
} tof_sample_t;

// attitude estimate
typedef struct {
    int64_t timestamp;
    float roll;   // deg
    float pitch;  // deg
    float yaw;    // deg (not fused here)
    float altitude_mm;
} attitude_t;

void mock_imu_read(imu_sample_t *imu_sample);

void mock_baro_read(baro_sample_t *baro_sample);

void mock_tof_read(tof_sample_t *tof_sample);

void imu_task(void *arg);

void baro_task(void *arg);

void tof_task(void *arg);


#endif